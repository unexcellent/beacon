//! Hardware-facing modules: the audio channel, the USB-C debug channel, and
//! the CSP-over-KISS payload link with its libcsp glue. (The cameras live in
//! the library's `beacon::camera`, wired up by the binary's `cameras` module.)

pub mod audio;
pub mod csp_arch;
pub mod kiss;
pub mod link;
