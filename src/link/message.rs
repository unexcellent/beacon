//! Outbound messages and their destinations on the network map.

use std::borrow::Cow;

use crate::Error;

const PAYLOAD_NODE: u16 = 14;
const PAYLOAD_PORT: u8 = 1;
const OBC_NODE: u16 = 1;
const OBC_PORT: u8 = 1;
const UHF_GROUND_NODE: u16 = 2;
const UHF_GROUND_PORT: u8 = 1;

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
