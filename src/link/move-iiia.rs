//! MOVE-IIIa carrier payload link: CSP over KISS over RS422 (UART1), and the
//! mission's [`PayloadLink`] built on top of the CSP node.

use std::collections::VecDeque;

use esp_idf_hal::{
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};

use super::command::parse_update_packet;
use super::csp::{CspLink, CspLinkConfig, SerialRead};
use super::kiss;
use super::{Command, CommandLink, Message, NODE};
use crate::error::{Error, Result};

const LINK_BAUD_RATE: u32 = 115_200;
const UPDATE_PORT: u8 = 10;
const CMD_PORT: u8 = 11;

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

/// The mission's payload link: the update / command service sockets over a CSP
/// node, implementing [`CommandLink`]. A private carrier detail — callers only
/// ever see it as `impl CommandLink`.
struct PayloadLink<R> {
    csp: CspLink<R>,
    update_sock: libcsp::Socket,
    cmd_sock: libcsp::Socket,
    /// Commands already received but not yet handed out by [`Self::receive`].
    commands: VecDeque<Command>,
}

impl<R: SerialRead> PayloadLink<R> {
    /// Bind the update / command service sockets on the brought-up CSP node. The
    /// node owns the serial transport and is pumped via its `poll`.
    fn try_new(csp: CspLink<R>) -> Result<Self> {
        let update_sock = csp.bind(UPDATE_PORT).map_err(|_| Error::CspInit)?;
        let cmd_sock = csp.bind(CMD_PORT).map_err(|_| Error::CspInit)?;

        Ok(Self {
            csp,
            update_sock,
            cmd_sock,
            commands: VecDeque::new(),
        })
    }

    /// Drain the update and command sockets into the internal command queue.
    fn collect_commands(&mut self) {
        while let Some(conn) = self.update_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if let Some(cmd) = parse_update_packet(pkt.data()) {
                    self.commands.push_back(cmd);
                }
            }
        }
        while let Some(conn) = self.cmd_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if pkt.data().starts_with(b"SSTV") {
                    log::info!("CMD: SSTV requested");
                    self.commands.push_back(Command::Sstv);
                } else {
                    log::warn!("CMD: unknown payload: {}", kiss::fmt_payload(pkt.data()));
                }
            }
        }
    }
}

impl<R: SerialRead> CommandLink for PayloadLink<R> {
    /// KISS-encodes the message and writes it to the serial link before returning.
    fn send(&self, message: Message) {
        log::info!(
            "Sending '{}' to {}:{}",
            String::from_utf8_lossy(&message.payload()),
            message.node(),
            message.port()
        );
        self.csp.send(
            message.node(),
            message.port(),
            libcsp::Priority::Norm,
            &message.payload(),
        );
    }

    /// Pumps the CSP node once, then drains the service sockets into the
    /// command queue.
    fn receive(&mut self) -> Result<Option<Command>> {
        if let Some(cmd) = self.commands.pop_front() {
            return Ok(Some(cmd));
        }
        self.csp.poll().map_err(|_| Error::UartReceive)?;
        self.collect_commands();
        Ok(self.commands.pop_front())
    }
}
