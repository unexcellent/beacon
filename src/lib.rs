//! Reusable beacon firmware: device drivers and the mission application.
//!
//! The device modules ([`camera`], [`audio`]) are project-agnostic — generic
//! sensor/DAC traits, chip drivers over `embedded-hal`, and ESP32-P4 transports.
//! On top of them sit the mission pieces: the payload [`link`] (CSP over KISS),
//! the crate-wide [`error`] type, firmware [`update`], SSTV [`transmit_sstv`],
//! and the [`idle`] command loop. Only board bring-up and `main` live in the
//! binary.

pub mod audio;
pub mod camera;
pub mod error;
pub mod idle;
pub mod link;
pub mod transmit_sstv;
pub mod update;
