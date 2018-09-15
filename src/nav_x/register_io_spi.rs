#![allow(dead_code)]

extern crate libc;
extern crate wpilib;

use std::time::Duration;
use std::time::SystemTime;
use std::time::UNIX_EPOCH;

use std::sync::Arc;
use std::sync::Mutex;
use std::thread;

use wpilib::spi;

use nav_x::ahrs;

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

struct IRegisterIO {
    port: spi::Spi,
    bit_rate: u32,
    rx_buffer: [u8; 256],
    trace: bool,
    update_rate_hz: u8,
    last_update_time: SystemTime,
    last_sensor_timestamp: u32,
    ahrs: Arc<Mutex<ahrs::AHRS>>,
    board_state: BoardState,
}

pub fn init_default(port: spi::Spi, ahrs: Arc<Mutex<ahrs::AHRS>>) {
    IRegisterIO::new(port, 8, ahrs);
}

impl IRegisterIO {
    //All constants verified!
    const MAX_SPI_MSG_LENGTH: usize = 256;

    //Defaults
    const IO_TIMEOUT_SECONDS: f64 = 1.0;
    const DELAY_OVERHEAD_MILLISECONDS: f32 = 4.0;

    //Protocol Constants (No touchy!)
    const NAVX_REG_UPDATE_RATE_HZ: u8 = 0x04;
    const NAVX_REG_INTEGRATION_CTL: u8 = 0x56;
    const NAVX_INTEGRATION_CTL_RESET_YAW: u8 = 0x80;
    const NAVX_REG_ACCEL_FSR_G: u8 = 0x05;

    const NAVX_REG_SENSOR_STATUS_L: u8 = 0x10;
    const NAVX_REG_SENSOR_STATUS_H: u8 = 0x11;

    const NAVX_INTEGRATION_CTL_RESET_DISP_X: u8 = 0x08;
    const NAVX_INTEGRATION_CTL_RESET_DISP_Y: u8 = 0x10;
    const NAVX_INTEGRATION_CTL_RESET_DISP_Z: u8 = 0x20;

    const NAVX_REG_WHOAMI: u8 = 0x00; /* IMU_MODEL_XXX */

    const NAVX_REG_GYRO_FSR_DPS_L: u8 = 0x06; /* Lower 8-bits of Gyro Full-Scale Range */
    const NAVX_REG_GYRO_FSR_DPS_H: u8 = 0x07; /* Upper 8-bits of Gyro Full-Scale Range */
    const NAVX_REG_OP_STATUS: u8 = 0x08; /* NAVX_OP_STATUS_XXX */
    const NAVX_REG_CAL_STATUS: u8 = 0x09; /* NAVX_CAL_STATUS_XXX */
    const NAVX_REG_SELFTEST_STATUS: u8 = 0x0A; /* NAVX_SELFTEST_STATUS_XXX */
    const NAVX_REG_CAPABILITY_FLAGS_L: u8 = 0x0B;

    const NAVX_REG_TIMESTAMP_L_L: u8 = 0x12;

    const NAVX_REG_YAW_L: u8 = 0x16; /* Lower 8 bits of Yaw     */
    const NAVX_REG_ROLL_L: u8 = 0x18; /* Lower 8 bits of Roll    */
    const NAVX_REG_PITCH_L: u8 = 0x1A; /* Lower 8 bits of Pitch   */
    const NAVX_REG_HEADING_L: u8 = 0x1C; /* Lower 8 bits of Heading */
    const NAVX_REG_HEADING_H: u8 = 0x1D; /* Upper 8 bits of Heading */
    const NAVX_REG_FUSED_HEADING_L: u8 = 0x1E; /* Upper 8 bits of Fused Heading */
    const NAVX_REG_FUSED_HEADING_H: u8 = 0x1F; /* Upper 8 bits of Fused Heading */

    const NAVX_REG_LAST: u8 = 0xF6;

    pub fn new(port: spi::Spi, bit_rate: u32, ahrs: Arc<Mutex<ahrs::AHRS>>) {
        let mut register_io = IRegisterIO {
            port,
            bit_rate,
            rx_buffer: [0; Self::MAX_SPI_MSG_LENGTH],
            trace: true,
            update_rate_hz: 60,
            last_update_time: UNIX_EPOCH,
            last_sensor_timestamp: 0,
            ahrs,
            board_state: BoardState {
                op_status: 0,
                sensor_status: 0,
                cal_status: 0,
                selftest_status: 0,
                capability_flags: 0,
                update_rate_hz: 0,
                accel_fsr_g: 0,
                gyro_fsr_dps: 0,
            },
        };

        thread::spawn(move || {
            register_io.init();

            register_io.set_update_rate_hz(60);
            register_io.get_configuration();

            let mut update_rate_ms = 1000. / (register_io.update_rate_hz as f32);
            if update_rate_ms > Self::DELAY_OVERHEAD_MILLISECONDS {
                update_rate_ms -= Self::DELAY_OVERHEAD_MILLISECONDS;
            }

            loop {
                {
                    if register_io.ahrs.lock().unwrap().should_stop() {
                        break;
                    }
                }
                if register_io.board_state.update_rate_hz != register_io.update_rate_hz {
                    let update_rate = register_io.update_rate_hz.clone();
                    register_io.set_update_rate_hz(update_rate);
                }
                register_io.get_current_data();
                //TODO: do we need to worry about the time it takes to actually do the work?
                thread::sleep(Duration::from_millis(update_rate_ms as u64));
            }
        });
    }

    pub fn init(&mut self) -> bool {
        self.port.set_clock_rate(self.bit_rate as f64);
        self.port.set_msb_first();
        self.port.set_sample_data_on_trailing_edge();
        self.port.set_clock_active_low();
        self.port.set_chip_select_active_low().unwrap();
        if self.trace {
            println!(
                "navX-MXP:  Initialized SPI communication at bitrate {}",
                self.bit_rate
            );
        }
        return true;
    }

    //Verified based on c++ source.
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/src/RegisterIOSPI.cpp#L28
    pub fn write(&mut self, address: u8, value: u8) -> bool {
        let mut cmd: Vec<u8> = vec![0; 3];
        cmd[0] = address | 0x80;
        cmd[1] = value;
        cmd[2] = Self::get_crc(&cmd[..], 2);
        if self.port.write(&cmd) != 3 {
            if self.trace {
                println!("navX-MXP SPI Write error");
            }
            return false;
        }
        return true;
    }

    //Verified based on c++ source.
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/src/RegisterIOSPI.cpp#L41
    pub fn read(&mut self, first_address: u8, buffer: &mut Vec<u8>, buffer_len: u8) -> bool {
        let mut cmd: [u8; 3] = [0; 3];
        cmd[0] = first_address;
        cmd[1] = buffer_len as u8;
        cmd[2] = Self::get_crc(&cmd[..], 2);

        if self.port.write(&cmd) != 3 {
            return false; // WRITE ERROR
        }

        thread::sleep(Duration::from_nanos(200));

        //Now that this part is fixed, it should always pass this if statement when connected.
        if self
            .port
            .read(true, &mut self.rx_buffer[0..(buffer_len + 1) as usize])
            != (buffer_len + 1) as i32
        {
            if self.trace {
                println!("NavX-MXP SPI Read error!");
            }
            return false; // READ ERROR
        }

        let crc: u8 = Self::get_crc(&self.rx_buffer[0..(buffer_len + 1) as usize], buffer_len);

        if crc != self.rx_buffer[buffer_len as usize] {
            if self.trace {
                //This is probably what is causing all the issues!
                println!(
                    "navX-MXP SPI CRC err.  Length:  {}, Got:  {}; Calculated:  {}",
                    buffer_len, self.rx_buffer[buffer_len as usize], crc
                );
            }
            return false; // CRC ERROR
        } else {
            //I wrote this part in place of memcpy.
            //You could try libc::memcpy instead.
            let mut pass: bool = false;
            for i in 0..buffer_len + 1 {
                buffer[i as usize] = self.rx_buffer[i as usize];
                if buffer[i as usize] != 0 {
                    pass = true;
                }
            }
            if pass {
                println!("Passed checksum and contained data!");
            }
        }
        return true;
    }

    //On by default for testing!
    pub fn enable_logging(&mut self, enable: bool) {
        self.trace = enable;
    }

    //Verified based on c++ source:
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/include/IMURegisters.h#L445
    fn get_crc(message: &[u8], len: u8) -> u8 {
        let mut crc: u8 = 0;

        for i in 0..len {
            crc ^= message[i as usize];
            for _ in 0..8 {
                if crc & 1 == 1 {
                    crc ^= 0x91;
                }
                crc >>= 1;
            }
        }
        return crc;
    }

    pub fn set_update_rate_hz(&mut self, update_rate: u8) {
        self.write(Self::NAVX_REG_UPDATE_RATE_HZ, update_rate);
    }

    //TODO: Add callable version (from AHRS)!
    pub fn zero_yaw(&mut self) {
        self.write(
            Self::NAVX_REG_INTEGRATION_CTL,
            Self::NAVX_INTEGRATION_CTL_RESET_YAW,
        );
    }

    //Not sure if I still need this function, might need to remove.
    pub fn get_configuration(&mut self) -> bool {
        let mut retry_count: i32 = 0;
        while retry_count < 3 {
            let mut config: [u8; 256] = [0; 256];
            if self.read(
                Self::NAVX_REG_WHOAMI,
                &mut config.to_vec(),
                Self::NAVX_REG_SENSOR_STATUS_H + 1,
            ) {
                self.board_state.cal_status = config[Self::NAVX_REG_CAL_STATUS as usize];
                self.board_state.op_status = config[Self::NAVX_REG_OP_STATUS as usize];
                self.board_state.selftest_status = config[Self::NAVX_REG_SELFTEST_STATUS as usize];
                self.board_state.sensor_status = Self::tou16(
                    &config[Self::NAVX_REG_SENSOR_STATUS_L as usize
                                ..(Self::NAVX_REG_SENSOR_STATUS_L + 2) as usize],
                ) as i16;
                self.board_state.gyro_fsr_dps = Self::tou16(
                    &config[Self::NAVX_REG_GYRO_FSR_DPS_L as usize
                                ..(Self::NAVX_REG_GYRO_FSR_DPS_L + 2) as usize],
                ) as i16;
                self.board_state.accel_fsr_g = config[Self::NAVX_REG_ACCEL_FSR_G as usize] as i16;
                self.board_state.update_rate_hz = config[Self::NAVX_REG_UPDATE_RATE_HZ as usize];
                self.board_state.capability_flags = Self::tou16(
                    &config[Self::NAVX_REG_CAPABILITY_FLAGS_L as usize
                                ..(Self::NAVX_REG_CAPABILITY_FLAGS_L + 2) as usize],
                ) as i16;
                return true;
            } else {
                thread::sleep(Duration::from_millis(50));
            }
            retry_count += 1;
        }
        return false;
    }

    //unverified (c++ original used pointer black-magic)
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/include/IMURegisters.h#L318
    fn tou16(arr: &[u8]) -> u16 {
        return (arr[0] as u16) << 8 | (arr[1] as u16);
    }

    //unverified (c++ original used pointer black-magic)
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/include/IMURegisters.h#L332
    fn tou32(arr: &[u8]) -> u32 {
        return ((arr[0] as u32) << 24)
            | ((arr[1] as u32) << 16)
            | ((arr[2] as u32) << 8)
            | (arr[3] as u32);
    }

    //Verified for the portions I kept.
    //https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/src/RegisterIO.cpp#L130
    pub fn get_current_data(&mut self) {
        let first_address: u8 = Self::NAVX_REG_UPDATE_RATE_HZ;

        //Hard coded to match known board capabilities!, see source link above
        let buffer_len: u8 = Self::NAVX_REG_LAST + 1 - first_address;
        let mut curr_data: Vec<u8> = vec![0; (Self::NAVX_REG_LAST + 1) as usize];

        if self.read(first_address, &mut curr_data, buffer_len) {
            //            let sensor_timestamp: u32 = Self::tou32(&curr_data[(Self::NAVX_REG_TIMESTAMP_L_L - first_address) as usize..(Self::NAVX_REG_TIMESTAMP_L_L - first_address + 4) as usize]);
            //            self.last_sensor_timestamp = sensor_timestamp;

            //This could be the incorrect way to do this, I don't know.
            *self.ahrs.lock().unwrap() = ahrs::get_instance(
                self.ahrs.lock().unwrap().should_stop(),
                parse_ascii_float(
                    &curr_data[(Self::NAVX_REG_YAW_L - first_address) as usize
                                   ..(Self::NAVX_REG_YAW_L - first_address + 4) as usize],
                ),
                parse_ascii_float(
                    &curr_data[(Self::NAVX_REG_PITCH_L - first_address) as usize
                                   ..(Self::NAVX_REG_PITCH_L - first_address + 4) as usize],
                ),
                parse_ascii_float(
                    &curr_data[(Self::NAVX_REG_ROLL_L - first_address) as usize
                                   ..(Self::NAVX_REG_ROLL_L - first_address + 4) as usize],
                ),
                parse_ascii_float(
                    &curr_data[(Self::NAVX_REG_HEADING_L - first_address) as usize
                                   ..(Self::NAVX_REG_HEADING_L - first_address + 4) as usize],
                ),
            );

            //If it gets to this point, print the output for yaw.
            println!("Yaw: {}", self.ahrs.lock().unwrap().get_yaw());
            //TODO: Replace with RobotBase::FPGA stamp
            self.last_update_time = SystemTime::now();
        }
    }
}

//Unverified!
//https://github.com/kauailabs/navxmxp/blob/732ff85c1535ed61e572c0d7e1035320b3a1dc02/roborio/c%2B%2B/navx_frc_cpp/include/IMURegisters.h#L344
fn parse_ascii_float(num: &[u8]) -> f64 {
    let ascii_string: String = match String::from_utf8(num.to_vec()) {
        Ok(v) => v,
        Err(_e) => "-1".to_string(),
    };
    match ascii_string.parse::<f64>() {
        Ok(v) => v,
        Err(_) => 0.0,
    }
}
