//! Data-plane trait: how raw frames physically arrive.

use std::time::Duration;

/// A frame transport, configured with the sensor's
/// [`FrameFormat`](super::FrameFormat) at construction. It moves raw bytes and
/// knows nothing about the chip that produces them.
pub trait CameraInterface {
    type Error: core::fmt::Debug;

    /// Obtain the next complete raw frame.
    ///
    /// Push-based transports (DMA-driven, like CSI) block until the next frame
    /// has been delivered or `timeout` elapses. Pull-based transports (like a
    /// clocked-out SPI readout) read the frame immediately; the caller is
    /// responsible for any readiness handshake first (see
    /// [`FrameTrigger`](super::sensor::FrameTrigger)).
    ///
    /// The returned slice lives in the transport's capture buffer and is only
    /// valid until the next call; for continuously-streaming transports the
    /// hardware may already be overwriting it with the following frame.
    fn wait_frame(&mut self, timeout: Duration) -> Result<&[u8], Self::Error>;
}
