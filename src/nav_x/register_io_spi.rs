extern crate libc;
extern crate wpilib;

use std::time::SystemTime;
use std::time::UNIX_EPOCH;
use std::time::Duration;

use std::sync::Arc;
use std::sync::Mutex;
use std::thread;

use wpilib::spi;


use nav_x::AHRS;

struct BoardID {
    board_type: u8,
    hw_rev: u8,
    fw_ver_major: u8,
    fw_ver_minor: u8,
    fw_revision: i16,
    unique_id: [u8; 12],
}

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

struct iregister_io {
    port: spi::Spi,
    bit_rate: u32,
    rx_buffer: [u8; 256],
    trace: bool,
    update_rate_hz: u8,
    last_update_time: SystemTime,
    byte_count: i32,
    update_count: i32,
    last_sensor_timestamp: u32,
    ahrs: Arc<Mutex<AHRS::AHRS>>,
    board_id: BoardID,
    board_state: BoardState,
}

pub fn initDefault(port: spi::Spi, ahrs: Arc<Mutex<AHRS>>) {
    iregister_io::new(port, 8, ahrs).run();
}

impl iregister_io {
    //Stuff
    const MAX_SPI_MSG_LENGTH: usize = 256;

    //Defaults
    const IO_TIMEOUT_SECONDS: f64 = 1.0;
    const DELAY_OVERHEAD_MILLISECONDS: f64 = 4.0;

    //Protocol Constants (No touchy!)
    const NAVX_REG_UPDATE_RATE_HZ: u8 = 0x04;
    const NAVX_REG_INTEGRATION_CTL: u8 = 0x56;
    const NAVX_INTEGRATION_CTL_RESET_YAW: u8 = 0x80;

    const NAVX_REG_SENSOR_STATUS_L: u8 = 0x10;
    const NAVX_REG_SENSOR_STATUS_H: u8 = 0x11;

    const NAVX_INTEGRATION_CTL_RESET_DISP_X: u8 = 0x08;
    const NAVX_INTEGRATION_CTL_RESET_DISP_Y: u8 = 0x10;
    const NAVX_INTEGRATION_CTL_RESET_DISP_Z: u8 = 0x20;

    const NAVX_REG_WHOAMI: u8 = 0x00; /* IMU_MODEL_XXX */
    const NAVX_REG_HW_REV: u8 = 0x01;
    const NAVX_REG_FW_VER_MAJOR: u8 = 0x02;
    const NAVX_REG_FW_VER_MINOR: u8 = 0x03;

    const NAVX_REG_GYRO_FSR_DPS_L: u8 = 0x06; /* Lower 8-bits of Gyro Full-Scale Range */
    const NAVX_REG_GYRO_FSR_DPS_H: u8 = 0x07; /* Upper 8-bits of Gyro Full-Scale Range */
    const NAVX_REG_OP_STATUS: u8 = 0x08; /* NAVX_OP_STATUS_XXX */
    const NAVX_REG_CAL_STATUS: u8 = 0x09; /* NAVX_CAL_STATUS_XXX */
    const NAVX_REG_SELFTEST_STATUS: u8 = 0x0A; /* NAVX_SELFTEST_STATUS_XXX */
    const NAVX_REG_CAPABILITY_FLAGS_L: u8 = 0x0B;
    const NAVX_REG_CAPABILITY_FLAGS_H: u8 = 0x0C;

    pub fn new(port: spi::Spi, bit_rate: u32, ahrs: Arc<Mutex<AHRS::AHRS>>) -> iregister_io {
        iregister_io {
            port,
            bit_rate,
            rx_buffer: [0; iregister_io::MAX_SPI_MSG_LENGTH],
            trace: false,
            update_rate_hz: 60,
            last_update_time: UNIX_EPOCH,
            byte_count: 0,
            update_count: 0,
            last_sensor_timestamp: 0,
            ahrs,
            board_id,
            board_state,
        }
    }

    pub fn init(&mut self) -> bool {
        self.port.set_clock_rate(self.bit_rate as f64);
        self.port.set_msb_first();
        self.port.set_sample_data_on_trailing_edge();
        self.port.set_clock_active_low();
        self.port.set_chip_select_active_low();
        if self.trace {
            println!("navX-MXP:  Initialized SPI communication at bitrate {}", self.bit_rate);
        }
        return true;
    }

    pub fn write(&mut self, address: u8, value: u8) -> bool {
        let mut cmd: [u8; 3] = [0; 3];
        cmd[0] = address | 0x80;
        cmd[1] = value;
        cmd[2] = iregister_io::getCRC(&cmd[..], 3);
        if self.port.write(&cmd) != 3 {
            if self.trace {
                println!("navX-MXP SPI Write error");
            }
            return false;
        }
        return true;
    }


    pub fn read(&mut self, first_address: u8, buffer: &mut [u8; 256], buffer_len: usize) -> bool {
        let mut cmd: [u8; 3] = [0; 3];
        cmd[0] = first_address;
        cmd[1] = buffer_len as u8;
        cmd[2] = iregister_io::getCRC(&cmd[..], 3);

        if self.port.write(&cmd) != 3 {
            return false; // WRITE ERROR
        }

        thread::sleep(Duration::from_millis(1));

        if self.port.read(true, buffer) != buffer_len + 1 {
            if self.trace {
                println!("navX-MXP SPI Read error");
            }
            return false; // READ ERROR
        }
        let crc: u8 = iregister_io::getCRC(&self.rx_buffer[..], buffer_len as u32);
        if crc != self.rx_buffer[buffer_len] {
            if self.trace {
                println!("navX-MXP SPI CRC err.  Length:  {}, Got:  {}; Calculated:  {}", buffer_len, self.rx_buffer[buffer_len], crc);
            }
            return false; // CRC ERROR
        } else {
            for i in 0..buffer_len {
                buffer[i] = self.rx_buffer[i];
            }
        }
        return true;
    }


    /*lol no. This is actually the same implementation as the original library.*/
    pub fn shutdown(&mut self) -> bool {
        return true;
    }

    pub fn enable_logging(&mut self, enable: bool) {
        self.trace = enable;
    }

    fn getCRC(message: &[u8], len: u32) -> u8 {
        let mut crc: u8 = 0;

        for i in 0..len - 1 {
            crc ^= message[i as usize];
            for j in 0..7 {
                if crc & 1 == 1 {
                    crc ^= 0x91;
                }
                crc >>= 1;
            }
        }
        return crc;
    }


    pub fn is_connected(&self) -> bool {
        let time_since_last_update = SystemTime::now().duration_since(self.last_update_time).expect("Whoops, broke time.");
        return time_since_last_update.as_millis() as f64 * 1000.0 <= iregister_io::IO_TIMEOUT_SECONDS;
    }

    pub fn get_byte_count(&self) -> i32 {
        return self.byte_count;
    }

    pub fn get_update_count(&self) -> i32 {
        return self.update_count;
    }

    pub fn set_update_rate_hz(&mut self, update_rate: u8) {
        self.write(iregister_io::NAVX_REG_UPDATE_RATE_HZ, update_rate);
    }

    pub fn ZeroYaw(&mut self) {
        self.write(iregister_io::NAVX_REG_INTEGRATION_CTL, iregister_io::NAVX_INTEGRATION_CTL_RESET_YAW);
    }

    pub fn zero_displacement(&mut self) {
        self.write(iregister_io::NAVX_REG_INTEGRATION_CTL, iregister_io::NAVX_INTEGRATION_CTL_RESET_DISP_X |
            iregister_io::NAVX_INTEGRATION_CTL_RESET_DISP_Y | iregister_io::NAVX_INTEGRATION_CTL_RESET_DISP_Z);
    }


    pub fn run(&mut self) {
        thread::spawn(move || {
            self.init();

            /* Initial Device Configuration */
            self.set_update_rate_hz(self.update_rate_hz);
            self.get_configuration();

            let mut update_rate_ms = 1.0 / self.update_rate_hz;
            if update_rate_ms > iregister_io::DELAY_OVERHEAD_MILLISECONDS {
                update_rate_ms -= iregister_io::DELAY_OVERHEAD_MILLISECONDS;
            }

            /* IO Loop */
            while !self.ahrs.lock().unwrap().should_stop() {
                if board_state.update_rate_hz != self.update_rate_hz {
                    self.set_update_rate_hz(self.update_rate_hz);
                }
                self.get_current_data();
                thread::sleep(Duration::from_millis(1000 / self.update_rate_hz as u64));
            }
        });
    }


    pub fn get_configuration(&mut self) -> bool {
        let mut retry_count: i32 = 0;
        while retry_count < 3 {
            let mut config: [u8; iregister_io::NAVX_REG_SENSOR_STATUS_H + 1 as usize] = [0; iregister_io::NAVX_REG_SENSOR_STATUS_H + 1];
            if self.read(iregister_io::NAVX_REG_WHOAMI, &mut config, config.len()) {
                self.board_id.hw_rev = config[iregister_io::NAVX_REG_HW_REV];
                self.board_id.fw_ver_major = config[iregister_io::NAVX_REG_FW_VER_MAJOR];
                self.board_id.fw_ver_minor = config[iregister_io::NAVX_REG_FW_VER_MINOR];
                self.board_id.board_type = config[iregister_io::NAVX_REG_WHOAMI];
                //notify_sink -> SetBoardID(board_id);

                self.board_state.cal_status = config[iregister_io::NAVX_REG_CAL_STATUS];
                self.board_state.op_status = config[iregister_io::NAVX_REG_OP_STATUS];
                self.board_state.selftest_status = config[iregister_io::NAVX_REG_SELFTEST_STATUS];
                self.board_state.sensor_status = iregister_io::tou16(config[iregister_io::NAVX_REG_SENSOR_STATUS_L..iregister_io::NAVX_REG_SENSOR_STATUS_L + 1]) as i16;
                self.board_state.gyro_fsr_dps = iregister_io::tou16(config[iregister_io::NAVX_REG_GYRO_FSR_DPS_L..iregister_io::NAVX_REG_GYRO_FSR_DPS_L + 1]) as i16;
                self.board_state.accel_fsr_g = config[iregister_io::NAVX_REG_ACCEL_FSR_G] as i16;
                self.board_state.update_rate_hz = config[iregister_io::NAVX_REG_UPDATE_RATE_HZ];
                self.board_state.capability_flags = iregister_io::tou16(config[iregister_io::NAVX_REG_CAPABILITY_FLAGS_L..iregister_io::NAVX_REG_CAPABILITY_FLAGS_L + 1]) as i16;
                //notify_sink -> SetBoardState(board_state);
                return true;
            } else {
                thread::sleep(Duration::from_millis(50));
            }
            retry_count += 1;
        }
        return false;
    }

    fn tou16(arr: &[u8]) -> u16 {
        return (arr[0] as u16) << 8 | arr[1];
    }

    fn tou32(arr: &[u8]) -> u32 {
        return ((arr[0] as u32) << 24) | ((arr[1] as u32) << 16) | ((arr[2] as u32) << 8) | (arr[3] as u32);
    }

    pub fn get_current_data(&mut self) {
        let first_address: u8 = iregister_io::NAVX_REG_UPDATE_RATE_HZ;
        let mut buffer_len: usize;
        let mut curr_data: [u8; iregister_io::NAVX_REG_LAST + 1] = [0; iregister_io::NAVX_REG_LAST + 1];
        /* If firmware supports displacement data, acquire it - otherwise implement */
        /* similar (but potentially less accurate) calculations on this processor.  */
        if displacement_registers {
            buffer_len = iregister_io::NAVX_REG_LAST + 1 - first_address;
        } else {
            buffer_len = iregister_io::NAVX_REG_QUAT_OFFSET_Z_H + 1 - first_address;
        }
        if self.read(first_address, &mut curr_data, buffer_len) {
            let sensor_timestamp: u32 = iregister_io::tou32(curr_data[iregister_io::NAVX_REG_TIMESTAMP_L_L - first_address..iregister_io::NAVX_REG_TIMESTAMP_L_L - first_address + 3]);
            if sensor_timestamp == self.last_sensor_timestamp {
                return;
            }
            self.last_sensor_timestamp = sensor_timestamp;
//            self.ahrspos_update.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
//            self. ahrspos_update.selftest_status = curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
//            self.ahrspos_update.cal_status = curr_data[NAVX_REG_CAL_STATUS];
//            self.ahrspos_update.sensor_status = curr_data[NAVX_REG_SENSOR_STATUS_L - first_address];
//            self.ahrspos_update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_YAW_L - first_address);
//            self.ahrspos_update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_PITCH_L - first_address);
//            self.ahrspos_update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_ROLL_L - first_address);
//            self.ahrspos_update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(curr_data + NAVX_REG_HEADING_L - first_address);
            *self.ahrs.lock().unwrap() = AHRS::AHRS {
                stop: self.ahrs.lock().unwrap(),
                yaw: parse_ascii_float(curr_data[iregister_io::NAVX_REG_YAW_L - first_address..iregister_io::NAVX_REG_YAW_L - first_address + 3]),
                pitch: parse_ascii_float(curr_data[iregister_io::NAVX_REG_PITCH_L - first_address..iregister_io::NAVX_REG_PITCH_L - first_address + 3]),
                roll: parse_ascii_float(curr_data[iregister_io::NAVX_REG_ROLL_L - first_address..iregister_io::NAVX_REG_ROLL_L - first_address + 3]),
                heading: parse_ascii_float(curr_data[iregister_io::NAVX_REG_HEADING_L - first_address..iregister_io::NAVX_REG_HEADING_L - first_address + 3]),
            };
//            self.ahrspos_update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_MPU_TEMP_C_L - first_address);
//            self.ahrspos_update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_X_L - first_address);
//            self.ahrspos_update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_Y_L - first_address);
//            self.ahrspos_update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_Z_L - first_address);
//            self.ahrspos_update.altitude = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_ALTITUDE_D_L - first_address);
//            self.ahrspos_update.barometric_pressure = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_PRESSURE_DL - first_address);
//            self.ahrspos_update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(curr_data + NAVX_REG_FUSED_HEADING_L - first_address);
//            self.ahrspos_update.quat_w = iregister_io::tou16(curr_data[NAVX_REG_QUAT_W_L - first_address..NAVX_REG_QUAT_W_L - first_address + 1]) / 32768.0;
//            self.ahrspos_update.quat_x = iregister_io::tou16(curr_data[NAVX_REG_QUAT_X_L - first_address .. NAVX_REG_QUAT_X_L - first_address + 1]) / 32768.0;
//            self.ahrspos_update.quat_y = iregister_io::tou16(curr_data[NAVX_REG_QUAT_Y_L - first_address .. NAVX_REG_QUAT_Y_L - first_address + 1]) / 32768.0;
//            self.ahrspos_update.quat_z = iregister_io::tou16(curr_data[NAVX_REG_QUAT_Z_L - first_address .. NAVX_REG_QUAT_Z_L - first_address + 1]) / 32768.0;


//            self.board_state.cal_status = curr_data[NAVX_REG_CAL_STATUS - first_address];
//            self.board_state.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
//            self.board_state.selftest_status = curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
//            self.board_state.sensor_status = iregister_io::tou16(curr_data[NAVX_REG_SENSOR_STATUS_L - first_address .. NAVX_REG_SENSOR_STATUS_L - first_address + 1]) as i16;
//            self.board_state.update_rate_hz = curr_data[NAVX_REG_UPDATE_RATE_HZ - first_address];
//            self.board_state.gyro_fsr_dps = iregister_io::tou16(curr_data[NAVX_REG_GYRO_FSR_DPS_L - first_address .. NAVX_REG_GYRO_FSR_DPS_L - first_address + 1]) as i16;
//            self.board_state.accel_fsr_g = curr_data[NAVX_REG_ACCEL_FSR_G - first_address];
//            self.board_state.capability_flags = iregister_io::tou16(curr_data[NAVX_REG_CAPABILITY_FLAGS_L - first_address .. NAVX_REG_CAPABILITY_FLAGS_L - first_address + 1]) as i16;

            self.last_update_time = SystemTime::now();
            self.byte_count += buffer_len;
            self.update_count += 1;
        }
    }
}

fn parse_ascii_float(num: &[u8]) -> f64 {
    let ascii_string: String = match String::from_utf8(num.to_vec()) {
        Ok(v) => v,
        Err(_e) => "-1".to_string(),
    };
    ascii_string.parse::<f64>().unwrap()
}