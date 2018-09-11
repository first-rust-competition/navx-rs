extern crate libc;

use std::time::SystemTime;
use std::time::UNIX_EPOCH;

use std::sync::Arc;
use std::sync::Mutex;
use std::thread;

pub enum spi_port {
    CS0 = 0,
    CS1 = 1,
    CS2 = 2,
    CS3 = 3,
    MXP = 4,
}

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

//Stuff
const MAX_SPI_MSG_LENGTH: usize = 256;

//Defaults
const IO_TIMEOUT_SECONDS: f64 = 1.0;
const DELAY_OVERHEAD_MILLISECONDS: f64 = 4.0;

//Protocol Constants (No touchy!)
const NAVX_REG_UPDATE_RATE_HZ: u8 = 0x04;
const NAVX_REG_INTEGRATION_CTL: u8 = 0x56;
const NAVX_INTEGRATION_CTL_RESET_YAW: u8 = 0x80;
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


struct iregister_io {
    port: spi_port,
    bit_rate: u32,
    rx_buffer: [u8; MAX_SPI_MSG_LENGTH],
    trace: bool,
    update_rate_hz: u8,
    last_update_time: SystemTime,
    byte_count: i32,
    update_count: i32,
    last_sensor_timestamp: i64,
    ahrs: Arc<Mutex<AHRS>>,
    board_id: BoardID,
    board_state: BoardState,
//    IMUProtocol::GyroUpdate raw_data_update;
//    AHRSProtocol::AHRSUpdate ahrs_update;
//    AHRSProtocol::AHRSPosUpdate ahrspos_update;
//    IIOCompleteNotification *notify_sink;
//    IBoardCapabilities *board_capabilities;
}

impl iregister_io {
    pub fn new(port: spi_port, bit_rate: u32, ahrs: Arc<Mutex<AHRS>>) -> iregister_io {
        iregister_io {
            port,
            bit_rate,
            rx_buffer: [0; MAX_SPI_MSG_LENGTH],
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
//        port -> SetClockRate(bitrate);
//        port -> SetMSBFirst();
//        port -> SetSampleDataOnFalling();
//        port-> SetClockActiveLow();
//        port -> SetChipSelectActiveLow();
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
        if /*port -> Write(cmd, sizeof(cmd)) != sizeof(cmd) */ false {
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

        if /*port->Write(cmd, sizeof(cmd)) != sizeof(cmd)*/false {
            return false; // WRITE ERROR
        }

        // TODO: How to wait?

        if /*port->Read(true, self.rx_buffer, buffer_len + 1) != buffer_len + 1 */ false {
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
        return time_since_last_update.as_millis() as f64 * 1000.0 <= IO_TIMEOUT_SECONDS;
    }

    pub fn get_byte_count(&self) -> i32 {
        return self.byte_count;
    }

    pub fn get_update_count(&self) -> i32 {
        return self.update_count;
    }

    pub fn set_update_rate_hz(&mut self, update_rate: u8) {
        self.write(NAVX_REG_UPDATE_RATE_HZ, update_rate);
    }

    pub fn ZeroYaw(&mut self) {
        self.write(NAVX_REG_INTEGRATION_CTL, NAVX_INTEGRATION_CTL_RESET_YAW);
        //notify_sink->YawResetComplete();
    }

    pub fn zero_displacement(&mut self) {
        self.write(NAVX_REG_INTEGRATION_CTL, NAVX_INTEGRATION_CTL_RESET_DISP_X |
            NAVX_INTEGRATION_CTL_RESET_DISP_Y | NAVX_INTEGRATION_CTL_RESET_DISP_Z);
    }


    pub fn run(&mut self) {
        thread::spawn(move || {
            self.init();

            /* Initial Device Configuration */
            self.set_update_rate_hz(self.update_rate_hz);
            self.get_configuration();

            let mut update_rate_ms = 1.0 / self.update_rate_hz;
            if update_rate_ms > DELAY_OVERHEAD_MILLISECONDS {
                update_rate_ms -= DELAY_OVERHEAD_MILLISECONDS;
            }

            /* IO Loop */
            while !stop {
                if board_state.update_rate_hz != self.update_rate_hz {
                    self.set_update_rate_hz(self.update_rate_hz);
                }
                self.get_current_data();
//                delayMillis(update_rate_ms);
            }
        });
    }


    pub fn get_configuration(&mut self) -> bool {
        let mut retry_count: i32 = 0;
        while retry_count < 3 {
            let mut config: [u8; NAVX_REG_SENSOR_STATUS_H + 1 as usize] = [0; NAVX_REG_SENSOR_STATUS_H + 1];
            if self.read(NAVX_REG_WHOAMI, &mut config, config.len()) {
                self.board_id.hw_rev = config[NAVX_REG_HW_REV];
                self.board_id.fw_ver_major = config[NAVX_REG_FW_VER_MAJOR];
                self.board_id.fw_ver_minor = config[NAVX_REG_FW_VER_MINOR];
                self.board_id.board_type = config[NAVX_REG_WHOAMI];
                //notify_sink -> SetBoardID(board_id);

                self.board_state.cal_status = config[NAVX_REG_CAL_STATUS];
                self.board_state.op_status = config[NAVX_REG_OP_STATUS];
                self.board_state.selftest_status = config[NAVX_REG_SELFTEST_STATUS];
                //self.board_state.sensor_status = IMURegisters::decodeProtocolUint16(config + NAVX_REG_SENSOR_STATUS_L);
                //self.board_state.gyro_fsr_dps = IMURegisters::decodeProtocolUint16(config + NAVX_REG_GYRO_FSR_DPS_L);
                self.board_state.accel_fsr_g = config[NAVX_REG_ACCEL_FSR_G] as i16;
                self.board_state.update_rate_hz = config[NAVX_REG_UPDATE_RATE_HZ];
                //self.board_state.capability_flags = IMURegisters::decodeProtocolUint16(config + NAVX_REG_CAPABILITY_FLAGS_L);
                //notify_sink -> SetBoardState(board_state);
                return true;
            } else {
                delayMillis(50);
            }
            retry_count += 1;
        }
        return false;
    }

    pub fn get_current_data(&mut self) {
        let first_address: u8 = NAVX_REG_UPDATE_RATE_HZ;
        //let displacement_registers : bool = board_capabilities -> IsDisplacementSupported();
        let mut buffer_len: usize;
        let mut curr_data: [u8; NAVX_REG_LAST + 1];
        /* If firmware supports displacement data, acquire it - otherwise implement */
        /* similar (but potentially less accurate) calculations on this processor.  */
        if displacement_registers {
            buffer_len = NAVX_REG_LAST + 1 - first_address;
        } else {
            buffer_len = NAVX_REG_QUAT_OFFSET_Z_H + 1 - first_address;
        }
        if self.read(first_address, /*(uint8_t *)curr_data*/b0, buffer_len) {
            //let sensor_timestamp: i64 = IMURegisters::decodeProtocolUint32(curr_data + NAVX_REG_TIMESTAMP_L_L - first_address);
            if sensor_timestamp == self.last_sensor_timestamp {
                return;
            }
            self.last_sensor_timestamp = sensor_timestamp;
            ahrspos_update.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
            ahrspos_update.selftest_status = curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
            ahrspos_update.cal_status = curr_data[NAVX_REG_CAL_STATUS];
            ahrspos_update.sensor_status = curr_data[NAVX_REG_SENSOR_STATUS_L - first_address];
            ahrspos_update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_YAW_L - first_address);
            ahrspos_update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_PITCH_L - first_address);
            ahrspos_update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_ROLL_L - first_address);
            ahrspos_update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(curr_data + NAVX_REG_HEADING_L - first_address);
            ahrspos_update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(curr_data + NAVX_REG_MPU_TEMP_C_L - first_address);
            ahrspos_update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_X_L - first_address);
            ahrspos_update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_Y_L - first_address);
            ahrspos_update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(curr_data + NAVX_REG_LINEAR_ACC_Z_L - first_address);
            ahrspos_update.altitude = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_ALTITUDE_D_L - first_address);
            ahrspos_update.barometric_pressure = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_PRESSURE_DL - first_address);
            ahrspos_update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(curr_data + NAVX_REG_FUSED_HEADING_L - first_address);
            ahrspos_update.quat_w = ((float)IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_QUAT_W_L - first_address)) / 32768.0f;
            ahrspos_update.quat_x = ((float)IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_QUAT_X_L - first_address)) / 32768.0f;
            ahrspos_update.quat_y = ((float)IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_QUAT_Y_L - first_address)) / 32768.0f;
            ahrspos_update.quat_z = ((float)IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_QUAT_Z_L - first_address)) / 32768.0f;
            if (displacement_registers) {
                ahrspos_update.vel_x = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_VEL_X_I_L - first_address);
                ahrspos_update.vel_y = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_VEL_Y_I_L - first_address);
                ahrspos_update.vel_z = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_VEL_Z_I_L - first_address);
                ahrspos_update.disp_x = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_DISP_X_I_L - first_address);
                ahrspos_update.disp_y = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_DISP_Y_I_L - first_address);
                ahrspos_update.disp_z = IMURegisters::decodeProtocol1616Float(curr_data + NAVX_REG_DISP_Z_I_L - first_address);
                notify_sink -> SetAHRSPosData(ahrspos_update, sensor_timestamp);
            } else {
                ahrs_update.op_status = ahrspos_update.op_status;
                ahrs_update.selftest_status = ahrspos_update.selftest_status;
                ahrs_update.cal_status = ahrspos_update.cal_status;
                ahrs_update.sensor_status = ahrspos_update.sensor_status;
                ahrs_update.yaw = ahrspos_update.yaw;
                ahrs_update.pitch = ahrspos_update.pitch;
                ahrs_update.roll = ahrspos_update.roll;
                ahrs_update.compass_heading = ahrspos_update.compass_heading;
                ahrs_update.mpu_temp = ahrspos_update.mpu_temp;
                ahrs_update.linear_accel_x = ahrspos_update.linear_accel_x;
                ahrs_update.linear_accel_y = ahrspos_update.linear_accel_y;
                ahrs_update.linear_accel_z = ahrspos_update.linear_accel_z;
                ahrs_update.altitude = ahrspos_update.altitude;
                ahrs_update.barometric_pressure = ahrspos_update.barometric_pressure;
                ahrs_update.fused_heading = ahrspos_update.fused_heading;
                notify_sink -> SetAHRSData(ahrs_update, sensor_timestamp);
            }

            board_state.cal_status = curr_data[NAVX_REG_CAL_STATUS - first_address];
            board_state.op_status = curr_data[NAVX_REG_OP_STATUS - first_address];
            board_state.selftest_status = curr_data[NAVX_REG_SELFTEST_STATUS - first_address];
            board_state.sensor_status = IMURegisters::decodeProtocolUint16(curr_data + NAVX_REG_SENSOR_STATUS_L - first_address);
            board_state.update_rate_hz = curr_data[NAVX_REG_UPDATE_RATE_HZ - first_address];
            board_state.gyro_fsr_dps = IMURegisters::decodeProtocolUint16(curr_data + NAVX_REG_GYRO_FSR_DPS_L - first_address);
            board_state.accel_fsr_g = (int16_t)curr_data[NAVX_REG_ACCEL_FSR_G - first_address];
            board_state.capability_flags = IMURegisters::decodeProtocolUint16(curr_data + NAVX_REG_CAPABILITY_FLAGS_L - first_address);
            notify_sink -> SetBoardState(board_state);

            raw_data_update.gyro_x = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_GYRO_X_L - first_address);
            raw_data_update.gyro_y = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_GYRO_Y_L - first_address);
            raw_data_update.gyro_z = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_GYRO_Z_L - first_address);
            raw_data_update.accel_x = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_ACC_X_L - first_address);
            raw_data_update.accel_y = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_ACC_Y_L - first_address);
            raw_data_update.accel_z = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_ACC_Z_L - first_address);
            raw_data_update.mag_x = IMURegisters::decodeProtocolInt16(curr_data + NAVX_REG_MAG_X_L - first_address);
            raw_data_update.temp_c = ahrspos_update.mpu_temp;
            notify_sink -> SetRawData(raw_data_update, sensor_timestamp);

            this -> last_update_time = Timer::GetFPGATimestamp();
            byte_count += buffer_len;
            update_count + +;
        }
    }
}