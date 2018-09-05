extern crate wpilib;
use std::{thread, time};
use wpilib::*;
use wpilib::serial::*;

mod NavX;



fn main() {
    let robot = RobotBase::new().expect("HAL FAILED");
    RobotBase::start_competition();
    let ds = robot.get_ds_instance();

    let mut navx =  NavX::NavX::new( Port::MXP);

    loop {
        println!("{}", navx.get_yaw());
        thread::sleep(time::Duration::from_millis(500));
    }
}
