// Copyright 2018 navx-rs Developers.
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

// #![allow(dead_code)]
extern crate byteorder;
extern crate crossbeam_channel as channel;
#[macro_use]
extern crate derive_more;
#[macro_use]
extern crate shrinkwraprs;
extern crate parking_lot;
extern crate wpilib;

use wpilib::spi;

#[allow(dead_code)]
mod protocol;
use std::thread;

enum IOMessage {
    ZeroYaw,
}

pub struct AHRS {
    io: channel::Sender<IOMessage>,
    io_thread: Option<thread::JoinHandle<()>>,
    coordinator: StateCoordinator,
}

impl AHRS {
    pub fn get_yaw(&self) -> f32 {
        let ahrs = self.coordinator.lock();
        if self.coordinator.board_yaw_reset_supported() {
            ahrs.yaw
        } else {
            unimplemented!("Currently, only gyros with on-board yaw-reset are supported")
        }
    }

    pub fn zero_yaw(&mut self) {
        if self.coordinator.board_yaw_reset_supported() {
            self.io.send(IOMessage::ZeroYaw);
        } else {
            unimplemented!("Currently, only gyros with on-board yaw-reset are supported")
        }
    }

    //TODO: Figure out what these actualyl need to do or if they need to be specialized by ioProvider
    // fn spi_init(&mut self, port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) {
    //     self.common_init(update_rate_hz);
    //     unimplemented!()
    // }

    // fn common_init(&mut self, update_rate_hz: u8) {
    //     unimplemented!()
    // }
}
impl AHRS {
    pub fn from_spi_minutiae(port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) -> Self {
        let u = Arc::new(Mutex::new(AhrsState::default()));
        let u_ = u.clone();
        let (s, r) = channel::bounded(0);
        let mut io = RegisterIO::new(
            RegisterIOSPI::new(wpilib::spi::Spi::new(port).unwrap(), spi_bitrate),
            update_rate_hz,
            StateCoordinator(u_),
            r,
        );
        let ahrs = Self {
            io: s,
            io_thread: Some(thread::spawn(move || {
                io.run();
            })),
            coordinator: StateCoordinator(u),
        };

        // ahrs.common_init(update_rate_hz)
        ahrs
    }
}

pub trait IOProvider {
    fn is_connected(&self) -> bool;
    fn byte_count(&self) -> i32;
    fn update_count(&self) -> i32;
    fn set_update_rate_hz(&mut self, update_rate: u8);
    fn zero_yaw(&mut self);
    fn zero_displacement(&mut self);
    fn run(&mut self);
    fn stop(&mut self);
    fn enable_logging(&mut self, enable: bool);
}

pub struct RegisterIO<H: RegisterProtocol> {
    io_provider: H,
    update_rate_hz: u8,
    stop: bool,
    raw_data_update: imu::GyroUpdate,
    ahrs_update: ahrs::AHRSUpdate,
    ahrspos_update: ahrs::AHRSPosUpdate,
    board_id: ahrs::BoardID,
    coordinator: StateCoordinator,
    board_state: BoardState,
    last_update_time: Duration,
    byte_count: i32,
    update_count: i32,
    last_sensor_timestamp: u64,
    cmd_chan: channel::Receiver<IOMessage>,
}

use std::time::Duration;

impl<H: RegisterProtocol> RegisterIO<H> {
    pub(crate) fn new(
        io_provider: H,
        update_rate_hz: u8,
        coordinator: StateCoordinator,
        chan: channel::Receiver<IOMessage>,
    ) -> Self {
        RegisterIO {
            io_provider,
            update_rate_hz,
            stop: false,
            raw_data_update: Default::default(),
            ahrs_update: Default::default(),
            ahrspos_update: Default::default(),
            coordinator,
            board_state: Default::default(),
            board_id: Default::default(),
            last_update_time: Default::default(),
            byte_count: Default::default(),
            update_count: Default::default(),
            last_sensor_timestamp: Default::default(),
            cmd_chan: chan,
        }
    }

    fn get_configuration(&mut self) -> bool {
        use self::protocol::registers::*;
        let mut success = false;
        let mut retry_count = 0;
        while retry_count < 3 && !success {
            let mut config = [0u8, NAVX_REG_SENSOR_STATUS_H as u8 + 1];
            if self
                .io_provider
                .read(NAVX_REG_WHOAMI as u8, &mut config[..])
            {
                self.board_id.hw_rev = config[NAVX_REG_HW_REV];
                self.board_id.fw_ver_major = config[NAVX_REG_FW_VER_MAJOR];
                self.board_id.fw_ver_minor = config[NAVX_REG_FW_VER_MINOR];
                self.board_id.type_ = config[NAVX_REG_WHOAMI];
                self.coordinator.set_board_id(&self.board_id);

                self.board_state.cal_status = config[NAVX_REG_CAL_STATUS];
                self.board_state.op_status = config[NAVX_REG_OP_STATUS];
                self.board_state.selftest_status = config[NAVX_REG_SELFTEST_STATUS];
                // these weird casts translate the C++ as written
                // pls kuailabs do more code reviews or something
                self.board_state.sensor_status =
                    registers::dec_prot_u16(&config[NAVX_REG_SENSOR_STATUS_L..]) as i16;
                self.board_state.gyro_fsr_dps =
                    registers::dec_prot_u16(&config[NAVX_REG_GYRO_FSR_DPS_L..]) as i16;
                self.board_state.accel_fsr_g = config[NAVX_REG_ACCEL_FSR_G] as i16;
                self.board_state.update_rate_hz = config[NAVX_REG_UPDATE_RATE_HZ];
                self.board_state.capability_flags =
                    registers::dec_prot_u16(&config[NAVX_REG_CAPABILITY_FLAGS_L..]) as i16;
                self.coordinator.set_board_state(&self.board_state);
                success = true;
            } else {
                success = false;
                thread::sleep(Duration::from_millis(50));
            }
            retry_count += 1;
        }
        return success;
    }

    fn get_current_data(&mut self) {
        use self::registers::*;
        let first_address = NAVX_REG_UPDATE_RATE_HZ as usize;
        let displacement_registers = self.coordinator.displacement_supported();
        let buffer_len: u8;
        let mut curr_data = [0u8, NAVX_REG_LAST as u8 + 1];
        /* If firmware supports displacement data, acquire it - otherwise implement */
        /* similar (but potentially less accurate) calculations on this processor.  */
        if displacement_registers {
            buffer_len = NAVX_REG_LAST as u8 + 1 - first_address as u8;
        } else {
            buffer_len = NAVX_REG_QUAT_OFFSET_Z_H as u8 + 1 - first_address as u8;
        }

        if self
            .io_provider
            .read(first_address as u8, &mut curr_data[..buffer_len as usize])
        {
            let sensor_timestamp: u64 =
                registers::dec_prot_u32(&curr_data[NAVX_REG_TIMESTAMP_L_L - first_address..])
                    as u64;
            if sensor_timestamp == self.last_sensor_timestamp {
                return;
            }
            self.last_sensor_timestamp = sensor_timestamp as u64;
            self.ahrspos_update.base.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
            self.ahrspos_update.base.selftest_status =
                curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
            self.ahrspos_update.base.cal_status = curr_data[NAVX_REG_CAL_STATUS];
            self.ahrspos_update.base.sensor_status =
                curr_data[NAVX_REG_SENSOR_STATUS_L - first_address];
            self.ahrspos_update.base.yaw = registers::dec_prot_signed_hundreths_float(
                &curr_data[NAVX_REG_YAW_L - first_address..],
            );
            self.ahrspos_update.base.pitch = registers::dec_prot_signed_hundreths_float(
                &curr_data[NAVX_REG_PITCH_L - first_address..],
            );
            self.ahrspos_update.base.roll = registers::dec_prot_signed_hundreths_float(
                &curr_data[NAVX_REG_ROLL_L - first_address..],
            );
            self.ahrspos_update.base.compass_heading =
                registers::decodeProtocolUnsignedHundredthsFloat(
                    &curr_data[NAVX_REG_HEADING_L - first_address..],
                );
            self.ahrspos_update.base.mpu_temp = registers::dec_prot_signed_hundreths_float(
                &curr_data[NAVX_REG_MPU_TEMP_C_L - first_address..],
            );
            self.ahrspos_update.base.linear_accel_x =
                registers::decodeProtocolSignedThousandthsFloat(
                    &curr_data[NAVX_REG_LINEAR_ACC_X_L - first_address..],
                );
            self.ahrspos_update.base.linear_accel_y =
                registers::decodeProtocolSignedThousandthsFloat(
                    &curr_data[NAVX_REG_LINEAR_ACC_Y_L - first_address..],
                );
            self.ahrspos_update.base.linear_accel_z =
                registers::decodeProtocolSignedThousandthsFloat(
                    &curr_data[NAVX_REG_LINEAR_ACC_Z_L - first_address..],
                );
            self.ahrspos_update.base.altitude = registers::decodeProtocol1616Float(
                &curr_data[NAVX_REG_ALTITUDE_D_L - first_address..],
            );
            self.ahrspos_update.base.barometric_pressure = registers::decodeProtocol1616Float(
                &curr_data[NAVX_REG_PRESSURE_DL - first_address..],
            );
            self.ahrspos_update.base.fused_heading =
                registers::decodeProtocolUnsignedHundredthsFloat(
                    &curr_data[NAVX_REG_FUSED_HEADING_L - first_address..],
                );
            self.ahrspos_update.base.quat_w =
                registers::dec_prot_i16(&curr_data[NAVX_REG_QUAT_W_L - first_address..]) as f32
                    / 32768.;
            self.ahrspos_update.base.quat_x =
                registers::dec_prot_i16(&curr_data[NAVX_REG_QUAT_X_L - first_address..]) as f32
                    / 32768.;
            self.ahrspos_update.base.quat_y =
                registers::dec_prot_i16(&curr_data[NAVX_REG_QUAT_Y_L - first_address..]) as f32
                    / 32768.;
            self.ahrspos_update.base.quat_z =
                registers::dec_prot_i16(&curr_data[NAVX_REG_QUAT_Z_L - first_address..]) as f32
                    / 32768.;
            if displacement_registers {
                self.ahrspos_update.vel_x = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_VEL_X_I_L - first_address..],
                );
                self.ahrspos_update.vel_y = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_VEL_Y_I_L - first_address..],
                );
                self.ahrspos_update.vel_z = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_VEL_Z_I_L - first_address..],
                );
                self.ahrspos_update.disp_x = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_DISP_X_I_L - first_address..],
                );
                self.ahrspos_update.disp_y = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_DISP_Y_I_L - first_address..],
                );
                self.ahrspos_update.disp_z = registers::decodeProtocol1616Float(
                    &curr_data[NAVX_REG_DISP_Z_I_L - first_address..],
                );
                self.coordinator
                    .set_ahrs_pos(&self.ahrspos_update, sensor_timestamp);
            } else {
                self.ahrs_update.base.op_status = self.ahrspos_update.base.op_status;
                self.ahrs_update.base.selftest_status = self.ahrspos_update.base.selftest_status;
                self.ahrs_update.base.cal_status = self.ahrspos_update.base.cal_status;
                self.ahrs_update.base.sensor_status = self.ahrspos_update.base.sensor_status;
                self.ahrs_update.base.yaw = self.ahrspos_update.base.yaw;
                self.ahrs_update.base.pitch = self.ahrspos_update.base.pitch;
                self.ahrs_update.base.roll = self.ahrspos_update.base.roll;
                self.ahrs_update.base.compass_heading = self.ahrspos_update.base.compass_heading;
                self.ahrs_update.base.mpu_temp = self.ahrspos_update.base.mpu_temp;
                self.ahrs_update.base.linear_accel_x = self.ahrspos_update.base.linear_accel_x;
                self.ahrs_update.base.linear_accel_y = self.ahrspos_update.base.linear_accel_y;
                self.ahrs_update.base.linear_accel_z = self.ahrspos_update.base.linear_accel_z;
                self.ahrs_update.base.altitude = self.ahrspos_update.base.altitude;
                self.ahrs_update.base.barometric_pressure =
                    self.ahrspos_update.base.barometric_pressure;
                self.ahrs_update.base.fused_heading = self.ahrspos_update.base.fused_heading;
                self.coordinator
                    .set_ahrs_data(&self.ahrs_update, sensor_timestamp);
            }

            // again with the weird casts
            self.board_state.cal_status = curr_data[NAVX_REG_CAL_STATUS - first_address];
            self.board_state.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
            self.board_state.selftest_status = curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
            self.board_state.sensor_status =
                registers::dec_prot_u16(&curr_data[NAVX_REG_SENSOR_STATUS_L - first_address..])
                    as i16;
            self.board_state.update_rate_hz = curr_data[NAVX_REG_UPDATE_RATE_HZ - first_address];
            self.board_state.gyro_fsr_dps =
                registers::dec_prot_u16(&curr_data[NAVX_REG_GYRO_FSR_DPS_L - first_address..])
                    as i16;
            self.board_state.accel_fsr_g = curr_data[NAVX_REG_ACCEL_FSR_G - first_address] as i16;
            self.board_state.capability_flags =
                registers::dec_prot_u16(&curr_data[NAVX_REG_CAPABILITY_FLAGS_L - first_address..])
                    as i16;
            self.coordinator.set_board_state(&self.board_state);

            self.raw_data_update.gyro_x =
                registers::dec_prot_i16(&curr_data[NAVX_REG_GYRO_X_L - first_address..]);
            self.raw_data_update.gyro_y =
                registers::dec_prot_i16(&curr_data[NAVX_REG_GYRO_Y_L - first_address..]);
            self.raw_data_update.gyro_z =
                registers::dec_prot_i16(&curr_data[NAVX_REG_GYRO_Z_L - first_address..]);
            self.raw_data_update.accel_x =
                registers::dec_prot_i16(&curr_data[NAVX_REG_ACC_X_L - first_address..]);
            self.raw_data_update.accel_y =
                registers::dec_prot_i16(&curr_data[NAVX_REG_ACC_Y_L - first_address..]);
            self.raw_data_update.accel_z =
                registers::dec_prot_i16(&curr_data[NAVX_REG_ACC_Z_L - first_address..]);
            self.raw_data_update.mag_x =
                registers::dec_prot_i16(&curr_data[NAVX_REG_MAG_X_L - first_address..]);
            self.raw_data_update.temp_c = self.ahrspos_update.base.mpu_temp;
            self.coordinator
                .set_raw_data(&self.raw_data_update, sensor_timestamp);

            self.last_update_time =
                RobotBase::fpga_time_duration().unwrap_or(Duration::from_millis(0));
            self.byte_count += buffer_len as i32;
            self.update_count += 1;
        }
    }

    const IO_TIMEOUT: Duration = Duration::from_millis(1000);
    const DELAY_OVERHEAD_MILLISECONDS: f64 = 4.0;
}

use wpilib::RobotBase;

impl<'a, H: RegisterProtocol> IOProvider for RegisterIO<H> {
    fn is_connected(&self) -> bool {
        RobotBase::fpga_time_duration().unwrap_or(Duration::default()) - self.last_update_time
            < Self::IO_TIMEOUT
    }
    fn byte_count(&self) -> i32 {
        self.byte_count
    }
    fn update_count(&self) -> i32 {
        self.update_count
    }
    fn set_update_rate_hz(&mut self, update_rate: u8) {
        self.io_provider
            .write(registers::NAVX_REG_UPDATE_RATE_HZ as u8, update_rate);
    }
    fn zero_yaw(&mut self) {
        self.io_provider.write(
            registers::NAVX_REG_INTEGRATION_CTL,
            registers::NAVX_INTEGRATION_CTL_RESET_YAW,
        );
        // notify_sink->YawResetComplete();
    }
    fn zero_displacement(&mut self) {
        self.io_provider.write(
            registers::NAVX_REG_INTEGRATION_CTL,
            registers::NAVX_INTEGRATION_CTL_RESET_DISP_X
                | registers::NAVX_INTEGRATION_CTL_RESET_DISP_Y
                | registers::NAVX_INTEGRATION_CTL_RESET_DISP_Z,
        );
    }

    /// Main thread loop
    fn run(&mut self) {
        self.io_provider.init();
        /* Initial Device Configuration */
        let rate = self.update_rate_hz;
        self.set_update_rate_hz(rate);
        self.get_configuration();

        let mut update_rate_ms = 1.0 / self.update_rate_hz as f64;
        if update_rate_ms > Self::DELAY_OVERHEAD_MILLISECONDS {
            update_rate_ms -= Self::DELAY_OVERHEAD_MILLISECONDS;
        }

        /* IO Loop */
        while !self.stop {
            if self.board_state.update_rate_hz != self.update_rate_hz {
                let rate = self.update_rate_hz;
                self.set_update_rate_hz(rate);
            }
            match self.cmd_chan.try_recv() {
                Some(IOMessage::ZeroYaw) => self.zero_yaw(),
                _ => (),
            };
            self.get_current_data();
            thread::sleep(Duration::from_millis(update_rate_ms as u64));
        }
    }

    // if anyone actually uses this, stop will probably become Arc<AtomicBool>
    // TODO that
    fn stop(&mut self) {
        self.stop = true;
    }

    fn enable_logging(&mut self, enable: bool) {
        self.io_provider.enable_logging(enable);
    }
}

pub trait RegisterProtocol {
    fn init(&mut self) -> bool;
    fn write(&mut self, address: u8, value: u8) -> bool;
    fn read(&mut self, first_address: u8, buf: &mut [u8]) -> bool;
    fn shutdown(&mut self) -> bool;
    fn enable_logging(&mut self, enable: bool);
}

use wpilib::spi::Spi;
const MAX_SPI_MSG_LENGTH: usize = 256;
pub struct RegisterIOSPI {
    port: Spi,
    bitrate: u32,
    rx_buf: [u8; MAX_SPI_MSG_LENGTH],
    trace: bool,
}

impl RegisterIOSPI {
    pub fn new(port: Spi, bitrate: u32) -> Self {
        RegisterIOSPI {
            port,
            bitrate,
            rx_buf: [0; MAX_SPI_MSG_LENGTH],
            trace: false,
        }
    }
}

use self::protocol::registers;
use std::mem::size_of_val;

//IDK but they do this
//TODO: look into this
static SPI_EX: Mutex<()> = Mutex::new(());

impl RegisterProtocol for RegisterIOSPI {
    fn init(&mut self) -> bool {
        self.port.set_clock_rate(self.bitrate as f64);
        self.port.set_msb_first();
        self.port.set_sample_data_on_trailing_edge();
        self.port.set_clock_active_low();
        //TODO return result
        self.port.set_chip_select_active_low().unwrap();
        if self.trace {
            println!(
                "navX-MXP:  Initialized SPI communication at bitrate {}\n",
                self.bitrate
            );
        }
        return true;
    }

    fn write(&mut self, address: u8, value: u8) -> bool {
        let _lock = SPI_EX.lock();
        let mut cmd = [0u8; 3];
        // srsly where the f does this come from
        cmd[0] = address | 0x80;
        cmd[1] = value;
        cmd[2] = registers::getCRC(&cmd[..], 2);
        if self.port.write(&cmd[..]) as usize != size_of_val(&cmd) {
            if self.trace {
                //TODO: replace with the proper logging crate
                println!("navX-MXP SPI Write error\n");
            }
            return false; // WRITE ERROR
        }
        return true;
    }

    fn read(&mut self, first_address: u8, buf: &mut [u8]) -> bool {
        let _lock = SPI_EX.lock();
        let mut cmd = [0u8; 3];
        cmd[0] = first_address;
        cmd[1] = buf.len() as u8;
        cmd[2] = registers::getCRC(&cmd[..], 2);
        if self.port.write(&cmd[..]) as usize != size_of_val(&cmd) {
            return false; // WRITE ERROR
        }
        // delay 200 us /* TODO:  What is min. granularity of delay()? */
        // ok fr that comment is from the original source. Why is the actual delay 5x longer than the comment?
        ::std::thread::sleep(::std::time::Duration::from_millis(1));
        if self.port.read(true, &mut self.rx_buf[..buf.len() + 1]) as usize != buf.len() + 1 {
            if self.trace {
                println!("navX-MXP SPI Read error\n");
            }
            return false; // READ ERROR
        }
        let crc = registers::getCRC(&self.rx_buf[..], buf.len() as u8);
        if crc != self.rx_buf[buf.len()] {
            if self.trace {
                println!(
                    "navX-MXP SPI CRC err.  Length:  {}, Got:  {}; Calculated:  {}\n",
                    buf.len(),
                    self.rx_buf[buf.len()],
                    crc
                );
            }
            return false; // CRC ERROR
        } else {
            let len = buf.len();
            buf.copy_from_slice(&self.rx_buf[..len]);
        }
        return true;
    }

    // idk man
    fn shutdown(&mut self) -> bool {
        true
    }
    fn enable_logging(&mut self, enable: bool) {
        self.trace = enable;
    }
}

// ==== Interthread communication stuff ====

use parking_lot::Mutex;

#[derive(Debug, Clone, Default)]
struct BoardState {
    op_status: u8,
    sensor_status: i16,
    cal_status: u8,
    selftest_status: u8,
    capability_flags: i16,
    update_rate_hz: u8,
    accel_fsr_g: i16,
    gyro_fsr_dps: i16,
}

#[derive(Debug, Clone, Default)]
struct AhrsState {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub compass_heading: f32,
    pub world_linear_accel_x: f32,
    pub world_linear_accel_y: f32,
    pub world_linear_accel_z: f32,
    pub mpu_temp_c: f32,
    pub fused_heading: f32,
    pub altitude: f32,
    pub baro_pressure: f32,
    pub is_moving: bool,
    pub is_rotating: bool,
    pub baro_sensor_temp_c: f32,
    pub altitude_valid: bool,
    pub is_magnetometer_calibrated: bool,
    pub magnetic_disturbance: bool,
    pub quaternion_w: f32,
    pub quaternion_x: f32,
    pub quaternion_y: f32,
    pub quaternion_z: f32,

    /* Integrated Data */
    pub velocity: [f32; 3],
    pub displacement: [f32; 3],

    /* Raw Data */
    pub raw_gyro_x: i16,
    pub raw_gyro_y: i16,
    pub raw_gyro_z: i16,
    pub raw_accel_x: i16,
    pub raw_accel_y: i16,
    pub raw_accel_z: i16,
    pub cal_mag_x: i16,
    pub cal_mag_y: i16,
    pub cal_mag_z: i16,

    /* Configuration/Status */
    pub update_rate_hz: u8,
    pub accel_fsr_g: i16,
    pub gyro_fsr_dps: i16,
    pub capability_flags: i16,
    pub op_status: u8,
    pub sensor_status: i16,
    pub cal_status: u8,
    pub selftest_status: u8,

    /* Board ID */
    pub board_type: u8,
    pub hw_rev: u8,
    pub fw_ver_major: u8,
    pub fw_ver_minor: u8,

    pub last_sensor_timestamp: u64,
    pub last_update_time: f64,
}

use self::protocol::ahrs;
use self::protocol::imu;

use parking_lot::MutexGuard;
use std::sync::Arc;

#[derive(Debug, Clone)]
pub struct StateCoordinator(Arc<Mutex<AhrsState>>);

impl StateCoordinator {
    fn lock<'ret, 'me: 'ret>(&'me self) -> MutexGuard<'ret, AhrsState> {
        self.0.lock()
    }

    fn set_ypr(&self, ypr_update: &imu::YPRUpdate, sensor_timestamp: u64) {
        let mut ahrs = self.0.lock();
        ahrs.yaw = ypr_update.yaw;
        ahrs.pitch = ypr_update.pitch;
        ahrs.roll = ypr_update.roll;
        ahrs.compass_heading = ypr_update.compass_heading;
        ahrs.last_sensor_timestamp = sensor_timestamp;
    }

    #[inline(always)]
    fn set_ahrs_base(ahrs: &mut AhrsState, ahrs_update: &ahrs::AHRSUpdateBase) {
        /* Update base IMU class variables */

        ahrs.yaw = ahrs_update.yaw;
        ahrs.pitch = ahrs_update.pitch;
        ahrs.roll = ahrs_update.roll;
        ahrs.compass_heading = ahrs_update.compass_heading;
        // ahrs.yaw_offset_tracker->UpdateHistory(ahrs_update.yaw);

        /* Update AHRS class variables */

        // 9-axis data
        ahrs.fused_heading = ahrs_update.fused_heading;

        // Gravity-corrected linear acceleration (world-frame)
        ahrs.world_linear_accel_x = ahrs_update.linear_accel_x;
        ahrs.world_linear_accel_y = ahrs_update.linear_accel_y;
        ahrs.world_linear_accel_z = ahrs_update.linear_accel_z;

        // Gyro/Accelerometer Die Temperature
        ahrs.mpu_temp_c = ahrs_update.mpu_temp;

        // Barometric Pressure/Altitude
        ahrs.altitude = ahrs_update.altitude;
        ahrs.baro_pressure = ahrs_update.barometric_pressure;

        // Status/Motion Detection
        ahrs.is_moving =
            ahrs_update.sensor_status & self::protocol::registers::NAVX_SENSOR_STATUS_MOVING != 0;
        ahrs.is_rotating = !(ahrs_update.sensor_status
            & self::protocol::registers::NAVX_SENSOR_STATUS_YAW_STABLE
            != 0);
        ahrs.altitude_valid = ahrs_update.sensor_status
            & self::protocol::registers::NAVX_SENSOR_STATUS_ALTITUDE_VALID
            != 0;
        ahrs.is_magnetometer_calibrated = ahrs_update.cal_status
            & self::protocol::registers::NAVX_CAL_STATUS_MAG_CAL_COMPLETE
            != 0;
        ahrs.magnetic_disturbance = ahrs_update.sensor_status
            & self::protocol::registers::NAVX_SENSOR_STATUS_MAG_DISTURBANCE
            != 0;

        ahrs.quaternion_w = ahrs_update.quat_w;
        ahrs.quaternion_x = ahrs_update.quat_x;
        ahrs.quaternion_y = ahrs_update.quat_y;
        ahrs.quaternion_z = ahrs_update.quat_z;
    }

    fn set_ahrs_pos(&self, ahrs_update: &ahrs::AHRSPosUpdate, sensor_timestamp: u64) {
        let mut ahrs = self.0.lock();

        Self::set_ahrs_base(&mut ahrs, &ahrs_update.base);

        ahrs.last_sensor_timestamp = sensor_timestamp;

        ahrs.velocity[0] = ahrs_update.vel_x;
        ahrs.velocity[1] = ahrs_update.vel_y;
        ahrs.velocity[2] = ahrs_update.vel_z;
        ahrs.displacement[0] = ahrs_update.disp_x;
        ahrs.displacement[1] = ahrs_update.disp_y;
        ahrs.displacement[2] = ahrs_update.disp_z;

        ahrs.last_sensor_timestamp = sensor_timestamp;
    }

    fn set_raw_data(&self, raw_data_update: &imu::GyroUpdate, sensor_timestamp: u64) {
        let mut ahrs = self.0.lock();
        ahrs.raw_gyro_x = raw_data_update.gyro_x;
        ahrs.raw_gyro_y = raw_data_update.gyro_y;
        ahrs.raw_gyro_z = raw_data_update.gyro_z;
        ahrs.raw_accel_x = raw_data_update.accel_x;
        ahrs.raw_accel_y = raw_data_update.accel_y;
        ahrs.raw_accel_z = raw_data_update.accel_z;
        ahrs.cal_mag_x = raw_data_update.mag_x;
        ahrs.cal_mag_y = raw_data_update.mag_y;
        ahrs.cal_mag_z = raw_data_update.mag_z;
        ahrs.mpu_temp_c = raw_data_update.temp_c;

        ahrs.last_sensor_timestamp = sensor_timestamp;
    }

    fn set_ahrs_data(&self, ahrs_update: &ahrs::AHRSUpdate, sensor_timestamp: u64) {
        let mut ahrs = self.0.lock();
        Self::set_ahrs_base(&mut ahrs, &ahrs_update.base);
        // Magnetometer Data
        ahrs.cal_mag_x = ahrs_update.cal_mag_x;
        ahrs.cal_mag_y = ahrs_update.cal_mag_y;
        ahrs.cal_mag_z = ahrs_update.cal_mag_z;

        ahrs.last_sensor_timestamp = sensor_timestamp;
    }

    fn set_board_id(&self, board_id: &ahrs::BoardID) {
        let mut ahrs = self.0.lock();
        ahrs.board_type = board_id.type_;
        ahrs.hw_rev = board_id.hw_rev;
        ahrs.fw_ver_major = board_id.fw_ver_major;
        ahrs.fw_ver_minor = board_id.fw_ver_minor;
    }

    fn set_board_state(&self, board_state: &BoardState) {
        let mut ahrs = self.0.lock();
        ahrs.update_rate_hz = board_state.update_rate_hz;
        ahrs.accel_fsr_g = board_state.accel_fsr_g;
        ahrs.gyro_fsr_dps = board_state.gyro_fsr_dps;
        ahrs.capability_flags = board_state.capability_flags;
        ahrs.op_status = board_state.op_status;
        ahrs.sensor_status = board_state.sensor_status;
        ahrs.cal_status = board_state.cal_status;
        ahrs.selftest_status = board_state.selftest_status;
    }

    fn omni_mount_supported(&self) -> bool {
        let ahrs = self.0.lock();
        ahrs.capability_flags & self::protocol::registers::NAVX_CAPABILITY_FLAG_OMNIMOUNT != 0
    }

    fn board_yaw_reset_supported(&self) -> bool {
        let ahrs = self.0.lock();
        ahrs.capability_flags & self::protocol::registers::NAVX_CAPABILITY_FLAG_YAW_RESET != 0
    }

    fn displacement_supported(&self) -> bool {
        let ahrs = self.0.lock();
        ahrs.capability_flags & self::protocol::registers::NAVX_CAPABILITY_FLAG_VEL_AND_DISP != 0
    }

    fn ahrs_pos_timestamp_supported(&self) -> bool {
        let ahrs = self.0.lock();
        ahrs.capability_flags & self::protocol::registers::NAVX_CAPABILITY_FLAG_AHRSPOS_TS != 0
    }
}
