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

impl AHRS {
    pub fn new() -> Arc<Mutex<AHRS>> {
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

    pub fn get_yaw(&self) -> f64 {
        self.yaw
    }

    pub fn get_pitch(&self) -> f64 {
        self.pitch
    }

    pub fn get_roll(&self) -> f64 {
        self.roll
    }

    pub fn get_heading(&self) -> f64 {
        self.heading
    }

    pub fn stop(&mut self) {
        self.stop = true;
    }

    pub fn should_stop(&self) -> bool {
        return self.stop;
    }
}