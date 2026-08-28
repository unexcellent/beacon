//! Reusable device abstractions for the beacon firmware.
//!
//! The [`camera`] module is written to be project-agnostic: generic sensor and
//! frame-transport traits, chip drivers over `embedded-hal`, and ESP32-P4
//! transport implementations. Application-specific policy (output resolution,
//! watermarking, SSTV encoding) lives in the binary.

pub mod audio;
pub mod camera;
pub mod csp;
pub mod kiss;
