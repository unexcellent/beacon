//! The CSP-over-KISS payload link with its libcsp glue. (Cameras and audio
//! live in the `beacon` library, wired up by the binary's `board` module.)

pub mod csp_arch;
pub mod kiss;
pub mod link;
