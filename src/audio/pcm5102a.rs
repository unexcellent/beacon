//! The PCM5102A DAC driver: packs the mono sample stream and drives it over an
//! [`AudioInterface`].

use super::{AudioChannel, AudioEncoder, AudioError, AudioInterface};

/// The PCM5102A 16-bit stereo DAC, driven over an [`AudioInterface`].
///
/// Packs the mono `i16` stream into the DAC's stereo I2S frame (audio on the
/// configured channel, silence on the other) and batches frames into DMA-sized
/// writes.
pub struct Pcm5102a<I: AudioInterface> {
    interface: I,
    sample_rate: u32,
    left_channel: bool,
    buf: Vec<u8>,
    buf_pos: usize,
}

impl<I: AudioInterface> Pcm5102a<I> {
    /// Serial format of the PCM5102A: Philips I2S standard, 16-bit, stereo,
    /// MSB-first, audio on the right channel. Build the [`I2sInterface`](super::I2sInterface)
    /// from this so its slot configuration matches the packing below.
    pub const ENCODER: AudioEncoder = AudioEncoder {
        width_16bit: true,
        stereo: true,
        bit_shift: true,
        left_channel: false,
        big_endian: false,
        least_significant_bit_first: false,
        left_align_data: false,
    };

    /// Wrap a transport as a PCM5102A output. `sample_rate` must match the rate
    /// the interface is clocked at; `chunk_size` is the number of samples
    /// batched per write.
    pub fn new(interface: I, sample_rate: u32, chunk_size: usize) -> Self {
        Self {
            interface,
            sample_rate,
            left_channel: Self::ENCODER.left_channel,
            buf: vec![0u8; chunk_size * 4],
            buf_pos: 0,
        }
    }
}

impl<I: AudioInterface> AudioChannel for Pcm5102a<I> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Packs a mono 16-bit sample into the stereo I2S frame and writes to the
    /// interface when the internal buffer is full.
    fn transmit(&mut self, sample: i16) -> Result<(), AudioError> {
        let [lo, hi] = sample.to_le_bytes();
        let (audio_off, silent_off) = if self.left_channel { (0, 2) } else { (2, 0) };
        self.buf[self.buf_pos + audio_off] = lo;
        self.buf[self.buf_pos + audio_off + 1] = hi;
        self.buf[self.buf_pos + silent_off] = 0;
        self.buf[self.buf_pos + silent_off + 1] = 0;
        self.buf_pos += 4;

        if self.buf_pos == self.buf.len() {
            self.flush()?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), AudioError> {
        self.interface.write(&self.buf[..self.buf_pos])?;
        self.buf_pos = 0;
        Ok(())
    }
}

impl<I: AudioInterface> Drop for Pcm5102a<I> {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}
