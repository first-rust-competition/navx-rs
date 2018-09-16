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

pub trait RegisterProtocol {}

pub struct RegisterIOSPI {}

impl RegisterProtocol for RegisterIOSPI {}

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

use self::protocol::imu;

use self::protocol::ahrs;

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

    fn set_ahrs_pos(&self, ahrs_update: &ahrs::AHRSPosUpdate, sensor_timestamp: u64) {
        unimplemented!()
    }

    fn set_raw_data(&self, raw_data_update: &imu::GyroUpdate, sensor_timestamp: u64) {
        unimplemented!()
    }

    fn set_ahrs_data(&self, ahrs_update: ahrs::AHRSUpdate, sensor_timestamp: u64) {
        unimplemented!()
    }

    fn set_board_id(&self, board_id: ahrs::BoardID) {
        unimplemented!()
    }

    fn set_board_state(&self, board_state: BoardState) {
        unimplemented!()
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
