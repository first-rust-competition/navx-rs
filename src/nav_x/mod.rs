extern crate wpilib;

use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use wpilib::ds::*;
use wpilib::serial::*;

mod register_io_spi;

const MESSAGE_START: u8 = b'!';
const BINARY_MESSAGE: u8 = b'#';

const COMPASS_MESSAGE: u8 = b'y';
const RAW_DATA_MESSAGE: u8 = b'g';

#[allow(dead_code)]
const STREAM_CONFIG_COMMAND: u8 = b'S';

#[allow(dead_code)]
const STREAM_CONFIG_RESPONSE: u8 = b's';

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
    pub fn new(port_id: Port) -> NavX {
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

    fn configure_serial_port(port: &mut SerialPort) {
        let mut error = None;
        match port.set_read_buf_size(256) {
            Ok(_v) => (),
            Err(e) => {
                error = Some(e);
                ()
            }
        };
        match port.set_timeout(1.0) {
            Ok(_v) => (),
            Err(e) => {
                error = Some(e);
                ()
            }
        };
        match port.enable_termination(b'\n') {
            Ok(_v) => (),
            Err(e) => {
                error = Some(e);
                ()
            }
        };
        match port.flush() {
            Ok(_v) => (),
            Err(e) => {
                error = Some(e);
                ()
            }
        };
        match port.reset() {
            Ok(_v) => (),
            Err(e) => {
                error = Some(e);
                ()
            }
        };
        match error {
            None => (),
            Some(v) => {
                let mut driver_station = DriverStation::new();
                driver_station.report_error("Serial port closed during reset!");
                driver_station.report_error(&v.message().to_uppercase());
            }
        }
        DriverStation::new().report_error("Configured Serial port.");
    }

    pub fn reset_serial_port(&mut self) {
        self.serial_port = Arc::new(Mutex::new(NavX::get_serial_port(self.port_id)));
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

    fn read_buffer(
        bytes_read: usize,
        buffer: &mut [u8; 256],
        yaw: &Arc<Mutex<f64>>,
        pitch: &Arc<Mutex<f64>>,
        roll: &Arc<Mutex<f64>>,
        heading: &Arc<Mutex<f64>>,
    ) -> BufferParseResponse {
        let mut direct_buffer_read_progress = 0;
        let mut awaiting_bytes = false;
        let mut parse_start = 512;
        let mut found_msg: bool = false;
        let mut packets_found = Vec::new();

        for i in 0..bytes_read {
            if buffer[i as usize] == MESSAGE_START {
                direct_buffer_read_progress = i;
                found_msg = true;
                if parse_start == 512 {
                    parse_start = i;
                }
            } else if buffer[i as usize] == BINARY_MESSAGE && found_msg {
                found_msg = false;
                //There are no binary packets we care about at the moment, so they can just be scrapped.
            } else {
                found_msg = false;
                if buffer[i as usize] == COMPASS_MESSAGE {
                    if i + 32 < bytes_read {
                        awaiting_bytes = true;
                    } else {
                        match NavX::verifyPacket(&mut buffer[i - 1 .. i + 32]) {
                            Some(v) => packets_found.push(v),
                            None => ()
                        };
                    }
                } else if buffer[i as usize] == RAW_DATA_MESSAGE {
                }
            }

            //Push bytes to other array if ending on a partial packet.
            if awaiting_bytes {
                break;
            }
        }

        BufferParseResponse {
            bytes_parsed: {
                if awaiting_bytes {
                    direct_buffer_read_progress
                } else {
                    bytes_read
                }
            },
            parse_start: {
                if parse_start == 512 {
                    0
                } else {
                    parse_start
                }
            },
            packets_found
        }
    }

    /*
       Takes in a slice which represents a single packet, then returns a packet struct if the packet
       passes the checksum and contains the correct packet start and end.
    */
    fn verifyPacket(packet: &mut [u8]) -> Option<Packet> {
        //Verify packet start and end conditions.
        if packet[0] != 0xA1 || packet[packet.len() - 2] != 0x13 || packet[packet.len() - 1] != 0x10 {
            return None;
        }
        let mut checksum : u8 = 0;
        for i in 0..packet.len() - 5 {
            checksum += packet[i];
        }

        let str_checksum = &match String::from_utf8(packet[packet.len() - 5..packet.len() - 3].to_vec()) {
            Ok(v) => v,
            Err(_e) => "00".to_string()
        };

        let original_checksum_val = u32::from_str_radix(str_checksum, 16).unwrap() as u8;
        if original_checksum_val != checksum {
            return None;
        }
        if packet[1] == 0xA3 {
            return Option::Some(Packet {
                packet_type: packet[3],
                length: packet.len() - 9 as usize,
                contents:  {
                    let mut arr : [u8; 128] = [0; 128];
                    arr.copy_from_slice(&packet[4 .. packet.len() - 5]);
                    arr
                }
            });
        }
        return Option::Some(Packet {
            packet_type: packet[1],
            length: packet.len() - 7 as usize,
            contents: {
                let mut arr : [u8; 128] = [0; 128];
                arr.copy_from_slice(&packet[2 .. packet.len() - 5]);
                arr
            }
    });
}

//deprecated
fn parse_compass_message_packet(
    body: &[u8],
    yaw: &Arc<Mutex<f64>>,
    pitch: &Arc<Mutex<f64>>,
    roll: &Arc<Mutex<f64>>,
    heading: &Arc<Mutex<f64>>,
) {
    DriverStation::new().report_error("Parsed packet for compass message.");
    *yaw.lock().unwrap() = NavX::parse_ascii_float(&body[0..6]);
    *pitch.lock().unwrap() = NavX::parse_ascii_float(&body[7..13]);
    *roll.lock().unwrap() = NavX::parse_ascii_float(&body[14..20]);
    *heading.lock().unwrap() = NavX::parse_ascii_float(&body[21..27]);
}

fn parse_ascii_float(num: &[u8]) -> f64 {
    let ascii_string: String = match String::from_utf8(num.to_vec()) {
        Ok(v) => v,
        Err(_e) => "-1".to_string(),
    };
    ascii_string.parse::<f64>().unwrap()
}

fn parse_binary_packet(_message_id: u8, _body: &[u8]) {
    //Maybe switch based on id, and distribute the body to some more specific parser?
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
}
