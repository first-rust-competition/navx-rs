extern crate wpilib;
use std::{thread, time};
use wpilib::*;
use wpilib::serial::*;

mod nav_x;



fn main() {
    let _robot = RobotBase::new().expect("HAL FAILED");
    RobotBase::start_competition();
    let navx =  nav_x::NavX::new( Port::MXP);

    loop {
        println!("{}", navx.get_yaw());
        thread::sleep(time::Duration::from_millis(500));
    }
}
