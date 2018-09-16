#![allow(dead_code)]
use wpilib::spi;

mod protocol;

pub struct AHRS<IO: IOProvider> {
    temp: IO,
}

impl<IO: IOProvider> AHRS<IO> {
    pub fn from_spi_minutiae(port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) -> Self {
        unimplemented!()
    }

    pub fn get_yaw(&self) -> f64 {
        unimplemented!()
    }

    pub fn zero_yaw(&self) -> f64 {
        unimplemented!()
    }

    fn spi_init(&mut self, port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) {
        self.common_init(update_rate_hz);
        unimplemented!()
    }

    fn common_init(&mut self, update_rate_hz: u8) {
        unimplemented!()
    }
}

pub trait IOProvider {
    fn is_connected() -> bool;
    fn byte_count() -> f64;
    fn update_count() -> f64;
    fn set_update_rate_hz(update_rate: u8);
    fn zero_yaw();
    fn zero_displacement();
    fn run();
    fn stop();
    fn enable_logging(enable: bool);
}

pub struct RegisterIO<H: RegisterProtocol> {
    temp: H,
}

impl<H: RegisterProtocol> RegisterIO<H> {
    pub fn new(io_provider: H, update_rate_hz: u8) {
        unimplemented!()
    }
}

impl<H: RegisterProtocol> IOProvider for RegisterIO<H> {
    fn is_connected() -> bool {
        unimplemented!()
    }
    fn byte_count() -> f64 {
        unimplemented!()
    }
    fn update_count() -> f64 {
        unimplemented!()
    }
    fn set_update_rate_hz(update_rate: u8) {
        unimplemented!()
    }
    fn zero_yaw() {
        unimplemented!()
    }
    fn zero_displacement() {
        unimplemented!()
    }
    fn run() {
        unimplemented!()
    }
    fn stop() {
        unimplemented!()
    }
    fn enable_logging(enable: bool) {
        unimplemented!()
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
        let lock = SPI_EX.lock();
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
        let lock = SPI_EX.lock();
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
use std::sync::Arc;

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

#[derive(Debug, Clone)]
pub struct StateCoordinator<'a>(&'a Mutex<AhrsState>);

impl<'a> StateCoordinator<'a> {
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

    fn set_ahrs_data(&self, ahrs_update: ahrs::AHRSUpdate, sensor_timestamp: u64) {
        let mut ahrs = self.0.lock();
        Self::set_ahrs_base(&mut ahrs, &ahrs_update.base);
        // Magnetometer Data
        ahrs.cal_mag_x = ahrs_update.cal_mag_x;
        ahrs.cal_mag_y = ahrs_update.cal_mag_y;
        ahrs.cal_mag_z = ahrs_update.cal_mag_z;

        ahrs.last_sensor_timestamp = sensor_timestamp;
    }

    fn set_board_id(&self, board_id: ahrs::BoardID) {
        let mut ahrs = self.0.lock();
        ahrs.board_type = board_id.type_;
        ahrs.hw_rev = board_id.hw_rev;
        ahrs.fw_ver_major = board_id.fw_ver_major;
        ahrs.fw_ver_minor = board_id.fw_ver_minor;
    }

    fn set_board_state(&self, board_state: BoardState) {
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
