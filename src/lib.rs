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
    bytes_parsed: i32,
    parse_start: i32,
}

impl NavX {
    pub fn new(port_id: Port) -> NavX {
        NavX { port_id, serial_port: getSerialPort(port_id), stop: false, yaw: 0.0, pitch: 0.0, roll: 0.0, heading: 0.0 }
    }

    pub fn get_serial_port(port_id: Port) -> SerialPort {
        let mut serial_port = SerialPort::new(57600, port_id, 8, Parity::None, StopBits::One)
            .expect("NavX serial port did not create correctly!");
        configureSerialPort(serial_port)
    }

    fn configure_serial_port(&mut serialPort: SerialPort) {
        serialPort.set_read_buf_size(256);
        serialPort.set_timeout(1.0);
        serialPort.enable_termination('\n' as u8);
        serialPort.flush();
        serialPort.reset();
    }

    pub fn reset_serial_port(&mut self) {
        self.serial_port = getSerialPort(self.port_id);
    }

    fn run(&mut self) {
        self.stop = false;


        let mut read_progress: u32 = 0;
        let mut direct_buffer_read_progress: i32 = 0;
        let mut buffer: [u8; 256];
        let mut partial_buffer: [u8; 256];
        let mut bytes_read: usize;

        let mut found_message: bool = false;
        let mut awaiting_bytes: bool = false;

        configureSerialPort(self.serial_port);

        while !self.stop {

            //initial parse of buffer
            bytes_read = self.serial_port.read(&buffer)? as usize;

            //Skip the logic if it didn't find a packet
            if bytes_read == 0 {
                continue;
            }

            //Parse what came through initially
            let bytes_parsed = self.read_buffer(bytes_read, buffer);

            //Add on the initial bytes
            if bytes_parsed.parse_start > 0 {
                for i in 0..bytes_parsed.parse_start {
                    partial_buffer[read_progress + i + 1] = bytes_read[i];
                }
                read_progress += bytes_parsed.parse_start;
            }

            //If a full packet has not yet been received, store it then rerun the loop.
            if bytes_parsed < bytes_read {
                for i in bytes_parsed..bytes_read {
                    partial_buffer[read_progress + i - bytes_parsed + 1] = bytes_read[i];
                }
                read_progress += bytes_read - bytes_parsed;
            }
        }
    }


    fn read_buffer(&mut self, bytes_read: usize, buffer: [u8; 256]) -> BufferParseResponse {
        let mut direct_buffer_read_progress = 0;
        let mut awaiting_bytes = false;
        let mut parse_start = -1;

        for i : usize in 0..bytes_read {
            if buffer[i as usize] == MESSAGE_START {
                direct_buffer_read_progress = i;
                foundMessage = true;
                if parse_start == -1 {
                    parse_start = i;
                }
            } else if buffer[i as usize] == BINARY_MESSAGE {
                foundMessage = false;

                if buffer[i + 1 as usize] + i > bytes_read {
                    awaiting_bytes = true;
                } else {
                    self.parse_binary_packet(buffer[(i + 2) as usize], &buffer[(i + 3) as usize..(i + 3 + buffer[(i + 1) as usize]) as usize]);
                }
            } else {
                foundMessage = false;
                if buffer[i as usize] == COMPASS_MESSAGE {
                    if i + 32 < bytes_read {
                        awaiting_bytes = true;
                    } else {
                        self.parse_compass_message_packet(buffer[(i + 1) as usize..(i + 28) as usize]);
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
            parse_start,
        };
    }

    fn parse_compass_message_packet(&mut self, body: slice<u8>) {
        self.yaw = parseASCIIFloat(body[0..6]);
        self.pitch = parseASCIIFloat(body[7..13]);
        self.roll = parseASCIIFloat(body[14..20]);
        self.heading = parseASCIIFloat(body[21..27]);
    }

    fn parse_asciifloat(num: slice<u8>) -> f64 {
        let mut err: bool = false;
        let ascii_string: String = match String::from_utf8(num.to_vec()) {
            Ok(v) => v,
            Err(e) => "-1".to_string()
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