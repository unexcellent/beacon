use core::ffi::c_void;
use std::ptr;

use esp_idf_sys::{
    EspError, esp, gpio_num_t_GPIO_NUM_NC as GPIO_NC, i2s_chan_config_t, i2s_chan_handle_t,
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

/// Encoder configuration for the PCM5102A 16-bit stereo DAC.
///
/// Uses the Philips I2S standard: 16-bit samples, stereo, MSB-first, audio on the right channel.
pub const PCM5102A: AudioEncoder = AudioEncoder {
    width_16bit: true,
    stereo: true,
    bit_shift: true,
    left_channel: false,
    big_endian: false,
    least_significant_bit_first: false,
    left_align_data: false,
};

/// Describes the physical I2S bus configuration: timing, pins, and DMA chunk size.
pub struct AudioInterface {
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

/// Interface configuration for the Philips I2S standard at 16 kHz on the ESP32-P4.
///
/// MCLK = 256 × 16 000 Hz = 4.096 MHz. BCLK = MCLK / 8 = 512 kHz,
/// which exactly satisfies the 16 kHz × 2 channels × 16 bits = 512 kHz requirement.
pub const PHILLIPS_I2S: AudioInterface = AudioInterface {
    sample_rate: 16_000,
    clock_divider: 8,
    mclk_pin: 20,
    bclk_pin: 21,
    dout_pin: 22,
    ws_pin: 23,
    chunk_size: 512,
};

/// An active I2S transmit channel configured for a specific encoder and interface.
///
/// The channel is created in a disabled state. Call [`enable`](AudioChannel::enable)
/// before transmitting and [`disable`](AudioChannel::disable) when done.
/// The underlying hardware channel is released automatically on drop.
pub struct AudioChannel {
    inner: i2s_chan_handle_t,
    left_channel: bool,
    buf: Vec<u8>,
    buf_pos: usize,
}

impl AudioChannel {
    /// Creates and configures a new I2S TX channel.
    ///
    /// Allocates the ESP-IDF I2S channel and applies the standard-mode configuration
    /// derived from `encoder` and `interface`. The channel is left disabled after creation.
    pub fn try_new(encoder: AudioEncoder, interface: AudioInterface) -> crate::Result<Self> {
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
                .map_err(|_| crate::Error::AudioInit)?
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
                sample_rate_hz: interface.sample_rate,
                clk_src: CLK_DEFAULT,
                ext_clk_freq_hz: 0,
                mclk_multiple: MCLK_256,
                bclk_div: interface.clock_divider,
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
                mclk: interface.mclk_pin as i32,
                bclk: interface.bclk_pin as i32,
                ws: interface.ws_pin as i32,
                dout: interface.dout_pin as i32,
                din: GPIO_NC,
                invert_flags: Default::default(),
            },
        };

        unsafe {
            esp!(i2s_channel_init_std_mode(tx, &std_cfg)).map_err(|_| crate::Error::AudioInit)?
        };

        let channel = Self {
            inner: tx,
            left_channel: encoder.left_channel,
            buf: vec![0u8; interface.chunk_size * 4],
            buf_pos: 0,
        };
        channel.enable().map_err(|_| crate::Error::AudioInit)?;
        Ok(channel)
    }

    /// Packs a mono 16-bit sample into the stereo I2S frame and writes to DMA when the
    /// internal buffer is full.
    pub fn transmit(&mut self, sample: i16) -> Result<(), EspError> {
        let [lo, hi] = sample.to_le_bytes();
        let (audio_off, silent_off) = if self.left_channel { (0, 2) } else { (2, 0) };
        self.buf[self.buf_pos + audio_off]     = lo;
        self.buf[self.buf_pos + audio_off + 1] = hi;
        self.buf[self.buf_pos + silent_off]     = 0;
        self.buf[self.buf_pos + silent_off + 1] = 0;
        self.buf_pos += 4;

        if self.buf_pos == self.buf.len() {
            self.flush()?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        let mut remaining = &self.buf[..self.buf_pos];
        while !remaining.is_empty() {
            let mut written = 0usize;
            unsafe {
                esp!(i2s_channel_write(
                    self.inner,
                    remaining.as_ptr() as *const c_void,
                    remaining.len(),
                    &mut written,
                    u32::MAX,
                ))?;
            }
            remaining = &remaining[written..];
        }
        self.buf_pos = 0;
        Ok(())
    }

    fn disable(&mut self) {
        let _ = self.flush();
        unsafe {
            let _ = i2s_channel_disable(self.inner);
        }
    }

    fn enable(&self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.inner)) }
    }
}

impl Drop for AudioChannel {
    fn drop(&mut self) {
        self.disable();
        unsafe {
            let _ = i2s_del_channel(self.inner);
        }
    }
}
