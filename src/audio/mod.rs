//! Audio output: a mono `i16` sample stream packed into a DAC's serial format
//! and pushed to the hardware.
//!
//! [`AudioChannel`] is the application-facing role — what an SSTV synthesiser
//! needs: a sample rate and a place to send samples. [`Pcm5102a`] implements it
//! for the PCM5102A DAC, generic over an [`AudioInterface`] that moves the packed
//! bytes to hardware. [`I2sInterface`] is the ESP32-P4 I2S implementation; all
//! platform code lives there, so the DAC driver itself is platform-agnostic.

mod encoder;
mod i2s;
mod interface;
mod pcm5102a;

/// MOVE-IIIa carrier bring-up.
#[path = "move-iiia.rs"]
pub mod move_iiia;

pub use encoder::AudioEncoder;
pub use i2s::{I2sConfig, I2sInterface};
pub use interface::AudioInterface;
pub use pcm5102a::Pcm5102a;

/// Error raised by the audio channel.
#[derive(Clone, Copy, Debug)]
pub enum AudioError {
    /// The I2S channel could not be created or configured.
    Init,
    /// Writing samples to the I2S channel failed.
    Transmission,
}

/// A place to send a mono `i16` sample stream. Implemented by a concrete DAC
/// driver ([`Pcm5102a`]); the SSTV synthesiser drives whatever satisfies this.
pub trait AudioChannel {
    /// The sample rate the DAC is clocked at, in Hz. The synthesiser must
    /// generate samples at this rate.
    fn sample_rate(&self) -> u32;

    /// Queue one sample for output. May flush buffered samples to hardware.
    fn transmit(&mut self, sample: i16) -> Result<(), AudioError>;

    /// Push any buffered samples to hardware.
    fn flush(&mut self) -> Result<(), AudioError>;
}
