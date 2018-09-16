use wpilib::spi;

mod protocol;

pub struct AHRS<IO: IOProvider> {
    temp: IO,
}

impl<IO: IOProvider> AHRS<IO> {
    pub fn from_spi_minutiae(port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) -> Self {
        unimplemented!()
    }

    pub fn get_yaw(&self) -> f64 {
        unimplemented!()
    }

    pub fn zero_yaw(&self) -> f64 {
        unimplemented!()
    }

    fn spi_init(&mut self, port: spi::Port, spi_bitrate: u32, update_rate_hz: u8) {
        self.common_init(update_rate_hz);
        unimplemented!()
    }

    fn common_init(&mut self, update_rate_hz: u8) {
        unimplemented!()
    }
}

pub trait IOProvider {
    fn is_connected() -> bool;
    fn byte_count() -> f64;
    fn update_count() -> f64;
    fn set_update_rate_hz(update_rate: u8);
    fn zero_yaw();
    fn zero_displacement();
    fn run();
    fn stop();
    fn enable_logging(enable: bool);
}

pub struct RegisterIO<H: RegisterProtocol> {
    temp: H,
}

impl<H: RegisterProtocol> RegisterIO<H> {
    fn new(io_provider: H, update_rate_hz: u8) {
        unimplemented!()
    }
}

impl<H: RegisterProtocol> IOProvider for RegisterIO<H> {
    fn is_connected() -> bool {
        unimplemented!()
    }
    fn byte_count() -> f64 {
        unimplemented!()
    }
    fn update_count() -> f64 {
        unimplemented!()
    }
    fn set_update_rate_hz(update_rate: u8) {
        unimplemented!()
    }
    fn zero_yaw() {
        unimplemented!()
    }
    fn zero_displacement() {
        unimplemented!()
    }
    fn run() {
        unimplemented!()
    }
    fn stop() {
        unimplemented!()
    }
    fn enable_logging(enable: bool) {
        unimplemented!()
    }
}

pub trait RegisterProtocol {}

pub struct RegisterIOSPI {}

impl RegisterProtocol for RegisterIOSPI {}
