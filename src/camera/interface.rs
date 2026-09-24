//! The platform's communication channel to a camera: the I2C control bus for
//! register access plus the frame-data transport. Implemented by ESP32-P4
//! transports in [`esp`](super::esp); the sensor drivers are generic over it and
//! carry no platform code of their own.

use std::time::Duration;

use embedded_hal::i2c::I2c;

use super::CameraError;

pub trait CameraInterface {
    /// The I2C bus type this transport exposes for register access.
    type Bus: I2c;

    /// The control bus to the sensor's registers. Sensors frame their own
    /// register protocol (address width differs per chip) over this.
    fn bus(&mut self) -> &mut Self::Bus;

    /// Obtain the next complete raw frame.
    ///
    /// Push-based transports (DMA-driven, like CSI) block until the next frame
    /// arrives or `timeout` elapses. Pull-based transports (a clocked-out SPI
    /// readout) read immediately; the caller must complete any readiness
    /// handshake first. The returned slice lives in the transport's capture
    /// buffer and is only valid until the next call.
    fn wait_frame(&mut self, timeout: Duration) -> Result<&[u8], CameraError>;
}
