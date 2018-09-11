extern crate wpilib;

use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use wpilib::ds::*;
use wpilib::serial::*;


pub struct NavX {
    port_id: Port,
    serial_port: Arc<Mutex<SerialPort>>,
    stop: Arc<Mutex<bool>>,
    yaw: Arc<Mutex<f64>>,
    pitch: Arc<Mutex<f64>>,
    roll: Arc<Mutex<f64>>,
    heading: Arc<Mutex<f64>>,
}

struct BufferParseResponse {
    bytes_parsed: usize,
    parse_start: usize,
    packets_found: Vec<Packet>
}

struct Packet {
    packet_type: u8,
    length: usize,
    contents: [u8; 128]
}

#[allow(dead_code)]
impl NavX {
    pub fn new(port_id: Port, bit_rate: u16) -> NavX {
        let mut navx = NavX {
            port_id,
            serial_port: Arc::new(Mutex::new(NavX::get_serial_port(port_id))),
            stop: Arc::new(Mutex::new(false)),
            yaw: Arc::new(Mutex::new(0.0)),
            pitch: Arc::new(Mutex::new(0.0)),
            roll: Arc::new(Mutex::new(0.0)),
            heading: Arc::new(Mutex::new(0.0)),
        };
        navx.run();
        navx
    }

    pub fn get_serial_port(port_id: Port) -> SerialPort {
        let mut serial_port = SerialPort::new(57600, port_id, 8, Parity::None, StopBits::One)
            .expect("NavX serial port did not create correctly!");
        NavX::configure_serial_port(&mut serial_port);
        serial_port
    }


    pub fn run(&mut self) {
        let yaw: Arc<Mutex<f64>> = self.yaw.clone();
        let roll: Arc<Mutex<f64>> = self.roll.clone();
        let pitch: Arc<Mutex<f64>> = self.pitch.clone();
        let heading: Arc<Mutex<f64>> = self.heading.clone();
        let serial_port: Arc<Mutex<SerialPort>> = self.serial_port.clone();
        let stop: Arc<Mutex<bool>> = self.stop.clone();

        thread::spawn(move || {
            let mut buffer: [u8; 256] = [0; 256];
            let mut partial_buffer: [u8; 256] = [0; 256];
            let mut bytes_read: usize;
            let mut read_progress: usize = 0;

            NavX::configure_serial_port(&mut serial_port.lock().unwrap());

            serial_port.lock().unwrap().write(&[0xA1, 0xA3, 0x08, 0xF9, 0x08, 0xB4, 0xC4, 0x10, 0x13]);

            while !*stop.lock().unwrap() {
                DriverStation::new().report_error("Attempting to read from serial port...");
                //initial parse of buffer
                bytes_read = match serial_port.lock().unwrap().read(&mut buffer[..]) {
                    Ok(v) => v,
                    Err(_e) => {
                        NavX::configure_serial_port(&mut serial_port.lock().unwrap());
                        0
                    }
                } as usize;
                DriverStation::new().report_error( "Finished reading from serial port.");

                //Skip the logic if it didn't find a packet
                if bytes_read == 0 {
                    DriverStation::new().report_error( "0 bytes read in last read.");

                    for i in 0..255 {
                        if buffer[i] != 0 {
                            DriverStation::new().report_error("Found non-zero entry in buffer!");
                        }
                    }
                    continue;
                }
                DriverStation::new().report_error("Got Data! Wooh!");

                //Parse what came through initially
                let first_read_response =
                    NavX::read_buffer(bytes_read, &mut buffer, &yaw, &pitch, &roll, &heading);

                //Add on the initial bytes
                if first_read_response.parse_start > 0 {
                    for i in 0..first_read_response.parse_start {
                        partial_buffer[read_progress + i + 1] = buffer[i];
                    }
                    read_progress += first_read_response.parse_start;
                }

                //Remove dangling bytes from mangled packets
                if first_read_response.bytes_parsed != first_read_response.parse_start {
                    let partial_buffer_read = NavX::read_buffer(
                        read_progress as usize,
                        &mut partial_buffer,
                        &yaw,
                        &pitch,
                        &roll,
                        &heading,
                    );

                    for i in partial_buffer_read.bytes_parsed..read_progress {
                        partial_buffer[i - partial_buffer_read.bytes_parsed] = partial_buffer[i];
                    }
                    read_progress -= partial_buffer_read.bytes_parsed;
                }

                //Flush partial_buffer and remove all the broken packets. A few dropped packets is not an issue.
                if read_progress + bytes_read - first_read_response.bytes_parsed >= 256 {
                    read_progress = 0;

                    //If a full packet has not yet been received, store it then rerun the loop.
                } else if first_read_response.bytes_parsed < bytes_read {
                    for i in first_read_response.bytes_parsed..bytes_read {
                        partial_buffer[read_progress + i + 1 - first_read_response.bytes_parsed] =
                            buffer[i];
                    }
                    read_progress += bytes_read - first_read_response.bytes_parsed;
                }
            }
        });
    }

    pub fn get_yaw(&self) -> f64 {
        *self.yaw.lock().unwrap()
    }

    pub fn get_pitch(&self) -> f64 {
        *self.pitch.lock().unwrap()
    }

    pub fn get_roll(&self) -> f64 {
        *self.roll.lock().unwrap()
    }

    pub fn get_heading(&self) -> f64 {
        *self.heading.lock().unwrap()
    }

    pub fn stop(&self) {
        *self.stop.lock().unwrap() = true;
    }
}
