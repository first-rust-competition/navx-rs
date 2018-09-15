#![feature(duration_as_u128)]

extern crate wpilib;
use std::{thread, time};
use wpilib::ds::*;
use wpilib::*;
mod nav_x;

fn main() {
    let _robot = RobotBase::new().expect("HAL FAILED");
    let mut driver_station = DriverStation::new();
    RobotBase::start_competition();
    let navx = nav_x::ahrs::AHRS::new();

    loop {
        //driver_station.report_error(navx.lock().unwrap().get_yaw().to_string().as_ref());
        thread::sleep(time::Duration::from_millis(500));
    }
}
