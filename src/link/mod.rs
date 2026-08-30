//! The mission protocol spoken over the CSP payload link: the network map
//! (which node/port carries what) and the message and command formats. The
//! generic CSP-over-KISS engine — including the RX pump and its error policy —
//! lives in the [`csp`] submodule; this is the mission application on top.
//!
//! - [`Command`] — inbound commands from the bus, parsed in [`command`].
//! - [`Message`] — outbound messages and their destinations, in [`message`].
//! - [`PayloadLink`] — the CSP-node-backed [`CommandLink`], in [`payload_link`].
//!
//! Its lower layers live alongside it: the generic CSP node in [`csp`] and the
//! KISS/CSP wire codec in [`kiss`].

pub mod csp;
mod kiss;

mod command;
mod message;
mod payload_link;

pub use command::Command;
pub use message::Message;
pub use payload_link::PayloadLink;

use crate::Result;

/// This node's CSP address on the payload bus.
pub const NODE: u16 = 7;

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
