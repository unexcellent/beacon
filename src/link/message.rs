//! Outbound messages and their destinations on the network map.

use std::borrow::Cow;

use crate::error::Error;

/// A destination on the CSP network: which node, and which port on it.
#[derive(Clone, Copy)]
pub struct Dest {
    pub node: u16,
    pub port: u8,
}

/// Where each class of outbound [`Message`] is routed. Supplied by the firmware
/// at link bring-up so a different mission can re-target the payload without
/// editing this crate.
#[derive(Clone, Copy)]
pub struct Routes {
    /// `Available` / `Busy` status.
    pub payload: Dest,
    /// `Booted` firmware-identity announcement.
    pub obc: Dest,
    /// `Error` reports.
    pub uhf_ground: Dest,
}

impl Routes {
    /// The MOVE-IIIa network map.
    pub const MOVE_IIIA: Routes = Routes {
        payload: Dest { node: 14, port: 1 },
        obc: Dest { node: 1, port: 1 },
        uhf_ground: Dest { node: 2, port: 1 },
    };
}

/// Messages that can be transmitted via the payload link
pub enum Message {
    /// Message for payload board announcing idle state.
    Available,
    /// Boot announcement carrying the firmware identity (version, ELF SHA256,
    /// and running OTA partition) for ground validation.
    Booted(String),
    /// Message for payload board announcing exit of idle state.
    Busy,
    /// Error report for ground station.
    Error(Error),
}

impl Message {
    /// Resolve this message's destination against the firmware's network map.
    pub fn dest(&self, routes: &Routes) -> Dest {
        match self {
            Self::Available | Self::Busy => routes.payload,
            Self::Booted(_) => routes.obc,
            Self::Error(_) => routes.uhf_ground,
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
