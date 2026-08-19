//! RS422 UART link to the payload board: DE pin, driver setup and raw byte I/O.

use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, Gpio37, Gpio38, Gpio39, Output, PinDriver},
    uart::{self, UartDriver, UART1},
    units::Hertz,
};

pub const BAUD_RATE: u32 = 115_200;

pub struct Uart {
    driver: UartDriver<'static>,
    /// RS422 full-duplex: the TX driver-enable pin is held high for the lifetime
    /// of the link. Kept as a field so it is not dropped (which would reset the
    /// pin and disable the transmitter).
    _de: PinDriver<'static, Output>,
}

impl Uart {
    pub fn new(
        uart: UART1<'static>,
        tx: Gpio38<'static>,
        rx: Gpio37<'static>,
        de: Gpio39<'static>,
    ) -> Self {
        let mut de = PinDriver::output(de).unwrap();
        de.set_high().unwrap();

        let driver = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &uart::config::Config::new()
                .baudrate(Hertz(BAUD_RATE))
                .rx_fifo_size(8192),
        )
        .unwrap();

        Self { driver, _de: de }
    }

    /// Write all of `data` and block until the TX FIFO has drained.
    pub fn send(&self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            sent += self.driver.write(&data[sent..]).unwrap();
        }
        self.driver.wait_tx_done(delay::BLOCK).unwrap();
    }

    pub fn read(&self, buf: &mut [u8], timeout: u32) -> Result<usize, esp_idf_hal::sys::EspError> {
        self.driver.read(buf, timeout)
    }
}
