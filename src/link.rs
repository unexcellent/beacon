//! The mission protocol spoken over the CSP payload link: the network map
//! (which node/port carries what) and the message and command formats. The
//! generic CSP-over-KISS engine — including the RX pump and its error policy —
//! lives in [`beacon::csp`](beacon::csp); this is the mission application on top.

use std::borrow::Cow;
use std::collections::VecDeque;

use beacon::csp::{CspLink, SerialRead};
use beacon::kiss;

use crate::{Error, Result};

pub const NODE: u16 = 7;
const OTA_PORT: u8 = 10;
const CMD_PORT: u8 = 11;
const PAYLOAD_NODE: u16 = 14;
const PAYLOAD_PORT: u8 = 1;
const OBC_NODE: u16 = 1;
const OBC_PORT: u8 = 1;
const UHF_GROUND_NODE: u16 = 2;
const UHF_GROUND_PORT: u8 = 1;

// OTA wire protocol: first payload byte selects the command, the rest is
// little-endian command data.
const OTA_CMD_ANNOUNCE: u8 = 0x00;
const OTA_CMD_BEGIN: u8 = 0x01;
const OTA_CMD_DATA: u8 = 0x02;
const OTA_CMD_END: u8 = 0x03;

/// A command received from the payload board, returned by
/// [`PayloadLink::receive`] for the application to execute.
pub enum Command {
    Sstv,
    /// A firmware update was announced with the given chunk size.
    UpdateAnnounced(u16),
    /// A firmware update session starts, expecting this many bytes in total.
    UpdateBegin(u32),
    /// One block of firmware bytes starting at `offset`.
    UpdateData {
        offset: u32,
        data: Vec<u8>,
    },
    /// The sender considers the firmware transfer complete.
    UpdateEnd,
}

/// Messages that can be transmitted via the payload link
pub enum Message {
    /// Message for payload board announcing idle state.
    Available,
    /// Boot announcement carrying the firmware identity for ground validation.
    Booted(String),
    /// Message for payload board announcing exit of idle state.
    Busy,
    /// Error report for ground station.
    Error(Error),
}

impl Message {
    /// Return the destination CSP node.
    pub fn node(&self) -> u16 {
        match self {
            Self::Available | Self::Busy => PAYLOAD_NODE,
            Self::Booted(_) => OBC_NODE,
            Self::Error(_) => UHF_GROUND_NODE,
        }
    }

    /// Return the destination CSP port.
    pub fn port(&self) -> u8 {
        match self {
            Self::Available | Self::Busy => PAYLOAD_PORT,
            Self::Booted(_) => OBC_PORT,
            Self::Error(_) => UHF_GROUND_PORT,
        }
    }

    /// Return the raw packet payload bytes.
    pub fn payload(&self) -> Cow<'_, [u8]> {
        match self {
            Self::Available => Cow::Borrowed(b"AVAILABLE".as_slice()),
            Self::Busy => Cow::Borrowed(b"BUSY".as_slice()),
            Self::Booted(fw) => Cow::Owned(format!("STATUS: BOOTED {fw}").into_bytes()),
            Self::Error(e) => Cow::Owned(e.to_string().into_bytes()),
        }
    }
}

/// The mission's view of the payload link: transmit [`Message`]s and poll for
/// inbound [`Command`]s. [`PayloadLink`] is the hardware-backed implementation;
/// depending on the trait lets the OTA and error-reporting logic run against a
/// stand-in link off-target.
pub trait CommandLink {
    /// Transmit a message via the link.
    fn send(&self, message: Message);

    /// Return the next pending command, or None if none is currently available.
    /// When nothing is buffered this pumps the link once, waiting up to the read
    /// timeout for traffic, so it is meant to be called in a `while let
    /// Some(cmd)` loop rather than a `for` loop.
    fn receive(&mut self) -> Result<Option<Command>>;
}

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

/// Parse one packet from the OTA socket into a [`Command`], or None (with a log)
/// if it is malformed.
fn parse_ota_packet(payload: &[u8]) -> Option<Command> {
    match *payload.first()? {
        OTA_CMD_ANNOUNCE => {
            let chunk_size = match payload.get(1..3) {
                Some(b) => u16::from_le_bytes([b[0], b[1]]),
                None => 0,
            };
            Some(Command::UpdateAnnounced(chunk_size))
        }
        OTA_CMD_BEGIN => match payload.get(1..5) {
            Some(b) => Some(Command::UpdateBegin(u32::from_le_bytes([
                b[0], b[1], b[2], b[3],
            ]))),
            None => {
                log::error!("OTA BEGIN: payload too short ({} bytes)", payload.len() - 1);
                None
            }
        },
        OTA_CMD_DATA => match payload.get(1..5) {
            Some(b) if payload.len() > 5 => Some(Command::UpdateData {
                offset: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
                data: payload[5..].to_vec(),
            }),
            _ => {
                log::error!("OTA DATA: payload too short ({} bytes)", payload.len() - 1);
                None
            }
        },
        OTA_CMD_END => Some(Command::UpdateEnd),
        cmd => {
            log::warn!("OTA: unknown command 0x{cmd:02x}");
            None
        }
    }
}
