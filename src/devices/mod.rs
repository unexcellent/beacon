//! Hardware-facing modules: the cameras, the audio channel, the USB-C debug
//! channel, and the CSP-over-KISS payload link with its libcsp glue.

pub mod audio;
pub mod camera;
pub mod csp_arch;
pub mod debug;
pub mod kiss;
pub mod link;
