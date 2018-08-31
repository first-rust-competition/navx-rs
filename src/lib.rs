extern crate wpilib;

use wpilib::serial::*;

const MESSAGE_START: u8 = '!' as u8;
const BINARY_MESSAGE: u8 = '#' as u8;

const COMPASS_MESSAGE: u8 = 'y' as u8;
const RAW_DATA_MESSAGE: u8 = 'g' as u8;
const STREAM_CONFIG_COMMAND: u8 = 'S' as u8;
const STREAM_CONFIG_RESPONSE: u8 = 's' as u8;

struct NavX {
    port_id: Port,
    serial_port: SerialPort,
    stop: bool,
    yaw: f64,
    pitch: f64,
    roll: f64,
    heading: f64,
}

struct BufferParseResponse {
    bytes_parsed: usize,
    parse_start: usize,
}

impl NavX {
    pub fn new(port_id: Port) -> NavX {
        NavX { port_id, serial_port: NavX::get_serial_port(port_id), stop: false, yaw: 0.0, pitch: 0.0, roll: 0.0, heading: 0.0 }
    }

    pub fn get_serial_port(port_id: Port) -> SerialPort {
        let mut serial_port = SerialPort::new(57600, port_id, 8, Parity::None, StopBits::One)
            .expect("NavX serial port did not create correctly!");
        NavX::configure_serial_port(&mut serial_port);
        serial_port
    }

    fn configure_serial_port(mut port: &mut SerialPort) {
        port.set_read_buf_size(256);
        port.set_timeout(1.0);
        port.enable_termination('\n' as u8);
        port.flush();
        port.reset();
    }

    pub fn reset_serial_port(&mut self) {
        self.serial_port = NavX::get_serial_port(self.port_id);
    }

    fn run(&mut self) {
        self.stop = false;

        let mut buffer: [u8; 256] = [0; 256];
        let mut partial_buffer: [u8; 256] = [0; 256];
        let mut bytes_read: usize;
        let mut read_progress: usize = 0;

        NavX::configure_serial_port(&mut self.serial_port);

        while !self.stop {

            //initial parse of buffer
            bytes_read = match self.serial_port.read(&mut buffer[..]){
                Ok(v) => v,
                Err(_e) => {
                    NavX::configure_serial_port(&mut self.serial_port);
                    0
                }
            } as usize;

            //Skip the logic if it didn't find a packet
            if bytes_read == 0 {
                continue;
            }

            //Parse what came through initially
            let first_read_response = self.read_buffer(bytes_read, buffer);

            //Add on the initial bytes
            if first_read_response.parse_start > 0 {
                for i in 0..first_read_response.parse_start {
                    partial_buffer[read_progress + i + 1] = buffer[i];
                }
                read_progress += first_read_response.parse_start;
            }

            //Remove dangling bytes from mangled packets
            if first_read_response.bytes_parsed != first_read_response.parse_start {
                let partial_buffer_read =  self.read_buffer(read_progress as usize, partial_buffer);

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
                    partial_buffer[read_progress + i + 1 - first_read_response.bytes_parsed] = buffer[i];
                }
                read_progress += bytes_read - first_read_response.bytes_parsed;
            }
        }
    }


    fn read_buffer(&mut self, bytes_read: usize, buffer: [u8; 256]) -> BufferParseResponse {
        let mut direct_buffer_read_progress = 0;
        let mut awaiting_bytes = false;
        let mut parse_start = 512;
        let mut found_msg : bool = false;

        for i in 0..bytes_read {
            if buffer[i as usize] == MESSAGE_START {
                direct_buffer_read_progress = i;
                found_msg = true;
                if parse_start == 512 {
                    parse_start = i;
                }
            } else if buffer[i as usize] == BINARY_MESSAGE && found_msg {
                found_msg = false;
                if buffer[i + 1 as usize] as usize + i  > bytes_read {
                    awaiting_bytes = true;
                } else {
                    self.parse_binary_packet(buffer[(i + 2) as usize], &buffer[(i + 3) as usize..(i + 3 + buffer[(i + 1) as usize] as usize) as usize]);
                }
            } else {
                found_msg = false;
                if buffer[i as usize] == COMPASS_MESSAGE {
                    if i + 32 < bytes_read {
                        awaiting_bytes = true;
                    } else {
                        self.parse_compass_message_packet(&buffer[(i + 1)..(i + 28)]);
                    }
                } else if buffer[i as usize] == RAW_DATA_MESSAGE {}
            }

            //Push bytes to other array if ending on a partial packet.
            if awaiting_bytes {
                break;
            }
        }

        return BufferParseResponse {
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
                }else {
                    parse_start
                }
            },
        };
    }

    fn parse_compass_message_packet(&mut self, body: &[u8]) {
        self.yaw = NavX::parse_ascii_float(&body[0..6]);
        self.pitch = NavX::parse_ascii_float(&body[7..13]);
        self.roll = NavX::parse_ascii_float(&body[14..20]);
        self.heading = NavX::parse_ascii_float(&body[21..27]);
    }

    fn parse_ascii_float(num: &[u8]) -> f64 {
        let ascii_string: String = match String::from_utf8(num.to_vec()) {
            Ok(v) => v,
            Err(_e) => "-1".to_string()
        };
        return ascii_string.parse::<f64>().unwrap();
    }

    fn parse_binary_packet(&self, message_id: u8, body: &[u8]) {
        //Maybe switch based on id, and distribute the body to some more specific parser?
    }

    pub fn get_yaw(&self) -> f64 {
        return self.yaw;
    }

    pub fn get_pitch(&self) -> f64 {
        return self.pitch;
    }

    pub fn get_roll(&self) -> f64 {
        return self.roll;
    }

    pub fn get_heading(&self) -> f64 {
        return self.heading;
    }
}