//! Audio output: a mono `i16` sample stream packed into a DAC's serial format
//! and pushed to the hardware.
//!
//! [`AudioChannel`] is the application-facing role — what an SSTV synthesiser
//! needs: a sample rate and a place to send samples. A concrete DAC driver
//! implements it, generic over an [`AudioInterface`] that moves the packed bytes
//! to hardware. [`I2sInterface`] is the ESP32-P4 I2S implementation; all platform
//! code lives there, so a DAC driver stays platform-agnostic. The concrete DAC
//! drivers live in the per-carrier firmware crates.

mod encoder;
#[cfg(target_os = "espidf")]
mod i2s;
mod interface;

pub use encoder::AudioEncoder;
#[cfg(target_os = "espidf")]
pub use i2s::{I2sConfig, I2sInterface};
pub use interface::AudioInterface;

/// Error raised by the audio channel.
#[derive(Clone, Copy, Debug)]
pub enum AudioError {
    /// The I2S channel could not be created or configured.
    Init,
    /// Writing samples to the I2S channel failed.
    Transmission,
}

/// A place to send a mono `i16` sample stream. Implemented by a concrete DAC
/// driver; the SSTV synthesiser drives whatever satisfies this.
pub trait AudioChannel {
    /// The sample rate the DAC is clocked at, in Hz. The synthesiser must
    /// generate samples at this rate.
    fn sample_rate(&self) -> u32;

    /// Queue one sample for output. May flush buffered samples to hardware.
    fn transmit(&mut self, sample: i16) -> Result<(), AudioError>;

    /// Push any buffered samples to hardware.
    fn flush(&mut self) -> Result<(), AudioError>;
}
