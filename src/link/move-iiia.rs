//! MOVE-IIIa carrier payload-link bring-up: CSP over KISS over RS422 (UART1).

use esp_idf_hal::{
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver, UartRxDriver},
    units::Hertz,
};

use super::csp::{CspLink, CspLinkConfig};
use super::{NODE, PayloadLink};
use crate::error::{Error, Result};

const LINK_BAUD_RATE: u32 = 115_200;

/// The mission's payload link over the ESP32-P4 UART RX transport.
pub type Rs422Link = PayloadLink<UartRxDriver<'static>>;

/// Bring up the RS422 UART (TX=GPIO38, RX=GPIO37, driver-enable=GPIO39) and
/// the CSP node, and wrap them in the mission's [`PayloadLink`].
pub fn initialize_payload_link(peripherals: Peripherals) -> Result<Rs422Link> {
    let mut de = PinDriver::output(peripherals.pins.gpio39).map_err(|_| Error::Peripheral)?;
    de.set_high().map_err(|_| Error::Peripheral)?;
    core::mem::forget(de);

    let driver = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart::config::Config::new()
            .baudrate(Hertz(LINK_BAUD_RATE))
            .rx_fifo_size(8192),
    )
    .map_err(|_| Error::UartAllocation)?;
    let (uart_tx, uart_rx) = driver.into_split();

    let csp = CspLink::try_new(
        CspLinkConfig {
            address: NODE,
            hostname: "beacon",
            model: "esp32p4",
        },
        uart_tx,
        uart_rx,
    )
    .map_err(|_| Error::CspInit)?;

    PayloadLink::try_new(csp)
}
