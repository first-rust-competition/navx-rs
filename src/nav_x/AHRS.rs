extern crate wpilib;

use wpilib::spi;

use std::sync::Arc;
use std::sync::Mutex;

use nav_x::register_io_spi;


pub struct AHRS {
    stop: bool,
    yaw: f64,
    pitch: f64,
    roll: f64,
    heading: f64,
}

//pub struct AHRSUpdateBase {
//    yaw: f32,
//    pitch: f32,
//    roll: f32,
//    compass_heading: f32,
//    altitude: f32,
//    fused_heading: f32,
//    linear_accel_x: f32,
//    linear_accel_y: f32,
//    linear_accel_z: f32,
//    mpu_temp: f32,
//    quat_w: f32,
//    quat_x: f32,
//    quat_y: f32,
//    quat_z: f32,
//    barometric_pressure: f32,
//    baro_temp: f32,
//    op_status: u8,
//    sensor_status: u8,
//    cal_status: u8,
//    selftest_status: u8,
//}
//
//pub struct AHRSUpdate {
//    base_update: AHRSUpdateBase,
//    cal_mag_x: i16,
//    cal_mag_y: i16,
//    cal_mag_z: i16,
//    mag_field_norm_ratio: f32,
//    mag_field_norm_scalar: f32,
//    raw_mag_x: i16,
//    raw_mag_y: i16,
//    raw_mag_z: i16,
//}
//
//
//pub struct AHRSPosUpdate {
//    base_update: AHRSUpdateBase,
//    vel_x: f32,
//    vel_y: f32,
//    vel_z: f32,
//    disp_x: f32,
//    disp_y: f32,
//    disp_z: f32,
//}

impl AHRS for AHRS {
    fn new() -> Arc<Mutex<AHRS>> {
        let ahrs = AHRS {
            stop: false,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            heading: 0.0,
        };
        let mut wrapped = Arc::new(Mutex::new(ahrs));
        register_io_spi::initDefault(spi::Spi::new(spi::Port::MXP).expect("Unable to get SPI MXP port!"), wrapped);
        wrapped
    }

    fn get_yaw(&self) -> f64 {
        self.yaw
    }

    fn get_pitch(&self) -> f64 {
        self.pitch
    }

    fn get_roll(&self) -> f64 {
        self.roll
    }

    fn get_heading(&self) -> f64 {
        self.heading
    }

    fn stop(&mut self) {
        self.stop = true;
    }

    fn should_stop(&self) -> bool {
        return self.stop;
    }
}