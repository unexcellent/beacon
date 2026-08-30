//! Audio output: a mono `i16` sample stream packed into a DAC's serial format
//! and pushed to the hardware.
//!
//! [`AudioChannel`] is the application-facing role — what an SSTV synthesiser
//! needs: a sample rate and a place to send samples. [`Pcm5102a`] implements it
//! for the PCM5102A DAC, generic over an [`AudioInterface`] that moves the packed
//! bytes to hardware. [`I2sInterface`] is the ESP32-P4 I2S implementation; all
//! platform code lives there, so the DAC driver itself is platform-agnostic.

use core::ffi::c_void;
use std::ptr;

use esp_idf_sys::{
    esp, gpio_num_t_GPIO_NUM_NC as GPIO_NC, i2s_chan_config_t, i2s_chan_handle_t,
    i2s_channel_disable, i2s_channel_enable, i2s_channel_init_std_mode, i2s_channel_write,
    i2s_data_bit_width_t_I2S_DATA_BIT_WIDTH_16BIT as BW_16,
    i2s_data_bit_width_t_I2S_DATA_BIT_WIDTH_24BIT as BW_24, i2s_del_channel,
    i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256 as MCLK_256, i2s_new_channel,
    i2s_port_t_I2S_NUM_0 as I2S0, i2s_role_t_I2S_ROLE_MASTER as MASTER,
    i2s_slot_bit_width_t_I2S_SLOT_BIT_WIDTH_AUTO as SLOT_AUTO,
    i2s_slot_mode_t_I2S_SLOT_MODE_MONO as MONO, i2s_slot_mode_t_I2S_SLOT_MODE_STEREO as STEREO,
    i2s_std_clk_config_t, i2s_std_config_t, i2s_std_gpio_config_t, i2s_std_slot_config_t,
    i2s_std_slot_mask_t_I2S_STD_SLOT_BOTH as SLOT_BOTH,
    i2s_std_slot_mask_t_I2S_STD_SLOT_LEFT as SLOT_LEFT,
    i2s_std_slot_mask_t_I2S_STD_SLOT_RIGHT as SLOT_RIGHT,
    soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT as CLK_DEFAULT,
};

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

/// The platform transport a DAC driver writes its packed bytes to. Implemented
/// by [`I2sInterface`] on the ESP32-P4; the DAC driver carries no platform code.
pub trait AudioInterface {
    /// Write packed audio bytes to the hardware, blocking until all are accepted.
    fn write(&mut self, bytes: &[u8]) -> Result<(), AudioError>;
}

/// Describes the serial data format expected by a DAC or audio codec.
///
/// These settings map directly to the I2S slot configuration and determine
/// how samples are packed on the wire.
pub struct AudioEncoder {
    /// Use 16-bit samples (`true`) or 24-bit samples (`false`).
    pub width_16bit: bool,
    /// Transmit both left and right channels (`true`) or a single mono channel (`false`).
    pub stereo: bool,
    /// Delay data by one BCLK cycle relative to WS (Philips I2S standard). Set `false` for left-justified format.
    pub bit_shift: bool,
    /// In stereo mode: which channel carries the audio signal (`true` = left, `false` = right).
    /// In mono mode: which I2S slot to use (`true` = left/WS-low, `false` = right/WS-high).
    pub left_channel: bool,
    /// Transmit the most significant byte first (`true`) or least significant byte first (`false`).
    pub big_endian: bool,
    /// Transmit the least significant bit first within each byte (`true`) or most significant bit first (`false`).
    pub least_significant_bit_first: bool,
    /// Align sample data to the left (MSB) edge of the slot (`true`) or right (LSB) edge (`false`).
    pub left_align_data: bool,
}

/// Physical I2S bus configuration: timing, pins, and DMA chunk size.
pub struct I2sConfig {
    /// Audio sample rate in Hz.
    pub sample_rate: u32,
    /// BCLK divider applied to MCLK (BCLK = MCLK / clock_divider).
    pub clock_divider: u32,
    /// GPIO pin number for MCLK (master clock).
    pub mclk_pin: u8,
    /// GPIO pin number for BCLK (bit clock).
    pub bclk_pin: u8,
    /// GPIO pin number for DOUT (serial data out).
    pub dout_pin: u8,
    /// GPIO pin number for WS (word select / left-right clock).
    pub ws_pin: u8,
    /// Number of samples per DMA write. Larger values increase latency but reduce CPU overhead.
    pub chunk_size: usize,
}

/// An active ESP32-P4 I2S transmit channel, configured for a DAC's serial format.
///
/// Enabled on construction; the underlying hardware channel is disabled and
/// released on drop.
pub struct I2sInterface {
    inner: i2s_chan_handle_t,
}

impl I2sInterface {
    /// Allocate and enable an I2S TX channel in standard mode, configured from
    /// the DAC's `encoder` format and the bus `config`.
    pub fn new(encoder: &AudioEncoder, config: &I2sConfig) -> Result<Self, AudioError> {
        let chan_cfg = i2s_chan_config_t {
            id: I2S0,
            role: MASTER,
            dma_desc_num: 6,
            dma_frame_num: 240,
            ..Default::default()
        };

        let mut tx: i2s_chan_handle_t = ptr::null_mut();
        unsafe {
            esp!(i2s_new_channel(&chan_cfg, &mut tx, ptr::null_mut()))
                .map_err(|_| AudioError::Init)?
        };

        let data_bit_width = if encoder.width_16bit { BW_16 } else { BW_24 };
        let ws_width = if encoder.width_16bit { 16 } else { 24 };
        let (slot_mode, slot_mask) = if encoder.stereo {
            (STEREO, SLOT_BOTH)
        } else if encoder.left_channel {
            (MONO, SLOT_LEFT)
        } else {
            (MONO, SLOT_RIGHT)
        };

        let std_cfg = i2s_std_config_t {
            clk_cfg: i2s_std_clk_config_t {
                sample_rate_hz: config.sample_rate,
                clk_src: CLK_DEFAULT,
                ext_clk_freq_hz: 0,
                mclk_multiple: MCLK_256,
                bclk_div: config.clock_divider,
            },
            slot_cfg: i2s_std_slot_config_t {
                data_bit_width,
                slot_bit_width: SLOT_AUTO,
                slot_mode,
                slot_mask,
                ws_width,
                ws_pol: false,
                bit_shift: encoder.bit_shift,
                left_align: encoder.left_align_data,
                big_endian: encoder.big_endian,
                bit_order_lsb: encoder.least_significant_bit_first,
            },
            gpio_cfg: i2s_std_gpio_config_t {
                mclk: config.mclk_pin as i32,
                bclk: config.bclk_pin as i32,
                ws: config.ws_pin as i32,
                dout: config.dout_pin as i32,
                din: GPIO_NC,
                invert_flags: Default::default(),
            },
        };

        unsafe { esp!(i2s_channel_init_std_mode(tx, &std_cfg)).map_err(|_| AudioError::Init)? };
        unsafe { esp!(i2s_channel_enable(tx)).map_err(|_| AudioError::Init)? };

        Ok(Self { inner: tx })
    }
}

impl AudioInterface for I2sInterface {
    fn write(&mut self, bytes: &[u8]) -> Result<(), AudioError> {
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let mut written = 0usize;
            unsafe {
                esp!(i2s_channel_write(
                    self.inner,
                    remaining.as_ptr() as *const c_void,
                    remaining.len(),
                    &mut written,
                    u32::MAX,
                ))
                .map_err(|_| AudioError::Transmission)?;
            }
            remaining = &remaining[written..];
        }
        Ok(())
    }
}

impl Drop for I2sInterface {
    fn drop(&mut self) {
        unsafe {
            let _ = i2s_channel_disable(self.inner);
            let _ = i2s_del_channel(self.inner);
        }
    }
}

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
    /// MSB-first, audio on the right channel. Build the [`I2sInterface`] from
    /// this so its slot configuration matches the packing below.
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
