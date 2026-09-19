//! MOVE-IIIa carrier payload link: CSP over KISS over RS422 (UART1), and the
//! mission's [`PayloadLink`] built on top of the CSP node.

use esp_idf_hal::{
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};

use super::csp::{CspLink, CspLinkConfig};
use super::payload::PayloadLink;
use super::{CommandLink, NODE};
use crate::error::{Error, Result};

const LINK_BAUD_RATE: u32 = 115_200;

/// Bring up the RS422 UART (TX=GPIO38, RX=GPIO37, driver-enable=GPIO39) and the
/// CSP node, and wrap them in the mission's payload link over the ESP32-P4 UART.
pub fn initialize_payload_link(peripherals: Peripherals) -> Result<impl CommandLink> {
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
