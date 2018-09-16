//Re-add when deleting main
//#![feature(duration_as_u128)]

extern crate byteorder;
#[macro_use]
extern crate derive_more;
#[macro_use]
extern crate shrinkwraprs;
#[cfg(not(test))]
extern crate wpilib;

extern crate parking_lot;

pub mod ahrs;
mod register_io_spi;

pub mod topdown;
