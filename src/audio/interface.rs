//! The platform transport a DAC driver writes its packed bytes to.

use super::AudioError;

/// The platform transport a DAC driver writes its packed bytes to. Implemented
/// by [`I2sInterface`](super::I2sInterface) on the ESP32-P4; the DAC driver
/// carries no platform code.
pub trait AudioInterface {
    /// Write packed audio bytes to the hardware, blocking until all are accepted.
    fn write(&mut self, bytes: &[u8]) -> Result<(), AudioError>;
}
