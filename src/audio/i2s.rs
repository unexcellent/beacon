//! ESP32-P4 I2S transmit transport: the platform-specific [`AudioInterface`].

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

use super::{AudioEncoder, AudioError, AudioInterface};

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
