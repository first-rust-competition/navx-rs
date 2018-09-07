extern crate wpilib;
use std::{thread, time};
use wpilib::*;
use wpilib::serial::*;
use wpilib::ds::*;

mod nav_x;



fn main() {
    let _robot = RobotBase::new().expect("HAL FAILED");
    let mut driver_station = DriverStation::new();
    RobotBase::start_competition();
    let navx =  nav_x::NavX::new( Port::MXP);
    loop {
        //driver_station.report_error(&navx.get_yaw().to_string());
        thread::sleep(time::Duration::from_millis(500));
    }
}
