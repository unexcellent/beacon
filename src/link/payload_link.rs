//! The [`PayloadLink`] device: the mission's OTA / command service sockets over
//! a CSP node, implementing [`CommandLink`].

use std::collections::VecDeque;

use super::command::parse_ota_packet;
use super::csp::{CspLink, SerialRead};
use super::kiss;
use super::{Command, CommandLink, Message};
use crate::{Error, Result};

const OTA_PORT: u8 = 10;
const CMD_PORT: u8 = 11;

pub struct PayloadLink<R> {
    csp: CspLink<R>,
    ota_sock: libcsp::Socket,
    cmd_sock: libcsp::Socket,
    /// Commands already received but not yet handed out by [`Self::receive`].
    commands: VecDeque<Command>,
}

impl<R: SerialRead> PayloadLink<R> {
    /// Bind the OTA / command service sockets on the brought-up CSP node. The
    /// node owns the serial transport and is pumped via its `poll`.
    pub fn try_new(csp: CspLink<R>) -> Result<Self> {
        let ota_sock = csp.bind(OTA_PORT).map_err(|_| Error::CspInit)?;
        let cmd_sock = csp.bind(CMD_PORT).map_err(|_| Error::CspInit)?;

        Ok(Self {
            csp,
            ota_sock,
            cmd_sock,
            commands: VecDeque::new(),
        })
    }

    /// Drain the OTA and command sockets into the internal command queue.
    fn collect_commands(&mut self) {
        while let Some(conn) = self.ota_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if let Some(cmd) = parse_ota_packet(pkt.data()) {
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
