//! MOVE-IIIa carrier audio bring-up: the PCM5102A DAC over I2S.

use super::{I2sConfig, I2sInterface, Pcm5102a};
use crate::error::Result;

/// Interface configuration for the Philips I2S standard at 16 kHz on the ESP32-P4.
///
/// MCLK = 256 × 16 000 Hz = 4.096 MHz. BCLK = MCLK / 8 = 512 kHz,
/// which exactly satisfies the 16 kHz × 2 channels × 16 bits = 512 kHz requirement.
const PHILLIPS_I2S: I2sConfig = I2sConfig {
    sample_rate: 16_000,
    clock_divider: 8,
    mclk_pin: 20,
    bclk_pin: 21,
    dout_pin: 22,
    ws_pin: 23,
    chunk_size: 512,
};

/// Bring up the PCM5102A DAC on the MOVE-IIIa I2S pins.
pub fn initialize_audio_channel() -> Result<Pcm5102a<I2sInterface>> {
    let interface = I2sInterface::new(&Pcm5102a::<I2sInterface>::ENCODER, &PHILLIPS_I2S)?;
    Ok(Pcm5102a::new(
        interface,
        PHILLIPS_I2S.sample_rate,
        PHILLIPS_I2S.chunk_size,
    ))
}
