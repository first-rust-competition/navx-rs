extern crate libc;

pub enum spi_port {
    CS0 = 0,
    CS1 = 1,
    CS2 = 2,
    CS3 = 3,
    MXP = 4,
}

const MAX_SPI_MSG_LENGTH: usize = 256;

struct iregister_io {
    port: spi_port,
    bit_rate: u32,
    rx_buffer: [u8; MAX_SPI_MSG_LENGTH],
    trace: bool,
}

impl iregister_io {
    pub fn new(port: spi_port, bit_rate: u32) -> iregister_io {
        iregister_io {
            port,
            bit_rate,
            rx_buffer: [0; MAX_SPI_MSG_LENGTH],
            trace: false,
        }
    }

    pub fn init(&mut self) -> bool {
//        port -> SetClockRate(bitrate);
//        port -> SetMSBFirst();
//        port -> SetSampleDataOnFalling();
//        port-> SetClockActiveLow();
//        port -> SetChipSelectActiveLow();
        if self.trace {
            println!("navX-MXP:  Initialized SPI communication at bitrate {}", self.bit_rate);
        }
        return true;
    }

    pub fn write(&mut self, address: u8, value: u8) -> bool {
        let mut cmd: [u8; 3] = [0; 3];
        cmd[0] = address | 0x80;
        cmd[1] = value;
        cmd[2] = iregister_io::getCRC(&cmd[..], 3);
        if /*port -> Write(cmd, sizeof(cmd)) != sizeof(cmd) */ false {
            if self.trace {
                println!("navX-MXP SPI Write error");
            }
            return false;
        }
        return true;
    }


    pub fn read(&mut self, first_address: u8, buffer: &mut [u8; 256], buffer_len: usize) -> bool {
        let mut cmd: [u8; 3] = [0; 3];
        cmd[0] = first_address;
        cmd[1] = buffer_len as u8;
        cmd[2] = iregister_io::getCRC(&cmd[..], 3);

        if /*port->Write(cmd, sizeof(cmd)) != sizeof(cmd)*/false {
            return false; // WRITE ERROR
        }

        // TODO: How to wait?

        if /*port->Read(true, self.rx_buffer, buffer_len + 1) != buffer_len + 1 */ false {
            if self.trace {
                println!("navX-MXP SPI Read error");
            }
            return false; // READ ERROR
        }
        let crc: u8 = iregister_io::getCRC(&self.rx_buffer[..], buffer_len as u32);
        if crc != self.rx_buffer[buffer_len] {
            if self.trace {
                println!("navX-MXP SPI CRC err.  Length:  {}, Got:  {}; Calculated:  {}", buffer_len, self.rx_buffer[buffer_len], crc);
            }
            return false; // CRC ERROR
        } else {
            for i in 0..buffer_len {
                buffer[i] = self.rx_buffer[i];
            }
        }
        return true;
    }


    pub fn shutdown(&mut self) -> bool {
        return true;
    }

    pub fn enable_logging(&mut self, enable: bool) {
        self.trace = enable;
    }

    fn getCRC(message: &[u8], len: u32) -> u8 {
        let mut crc: u8 = 0;

        for i in 0..len - 1 {
            crc ^= message[i as usize];
            for j in 0..7 {
                if crc & 1 == 1 {
                    crc ^= 0x91;
                }
                crc >>= 1;
            }
        }
        return crc;
    }
}