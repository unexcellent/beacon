use core::ffi::c_void;
use std::ptr;

use esp_idf_sys::{
    EspError, esp, gpio_num_t_GPIO_NUM_NC as GPIO_NC, i2s_chan_config_t, i2s_chan_handle_t,
    i2s_channel_disable, i2s_channel_enable, i2s_channel_init_std_mode, i2s_channel_write,
    i2s_data_bit_width_t_I2S_DATA_BIT_WIDTH_16BIT as BW_16, i2s_del_channel,
    i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256 as MCLK_256, i2s_new_channel,
    i2s_port_t_I2S_NUM_0 as I2S0, i2s_role_t_I2S_ROLE_MASTER as MASTER,
    i2s_slot_bit_width_t_I2S_SLOT_BIT_WIDTH_AUTO as SLOT_AUTO,
    i2s_slot_mode_t_I2S_SLOT_MODE_STEREO as STEREO, i2s_std_clk_config_t, i2s_std_config_t,
    i2s_std_gpio_config_t, i2s_std_slot_config_t,
    i2s_std_slot_mask_t_I2S_STD_SLOT_BOTH as SLOT_BOTH,
    soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT as CLK_DEFAULT,
};

pub const SAMPLE_RATE: u32 = 16_000;

const PIN_MCLK: i32 = 20; // SCK
const PIN_BCLK: i32 = 21; // BCK
const PIN_DOUT: i32 = 22; // DIN on DAC side
const PIN_WS: i32 = 23; // LRCK

pub const CHUNK_SAMPLES: usize = 512;

pub struct I2s(i2s_chan_handle_t);

impl I2s {
    pub fn new() -> Result<Self, EspError> {
        let chan_cfg = i2s_chan_config_t {
            id: I2S0,
            role: MASTER,
            dma_desc_num: 6,
            dma_frame_num: 240,
            ..Default::default()
        };

        let mut tx: i2s_chan_handle_t = ptr::null_mut();
        unsafe { esp!(i2s_new_channel(&chan_cfg, &mut tx, ptr::null_mut()))? };

        let std_cfg = i2s_std_config_t {
            clk_cfg: i2s_std_clk_config_t {
                sample_rate_hz: SAMPLE_RATE,
                clk_src: CLK_DEFAULT,
                ext_clk_freq_hz: 0,
                mclk_multiple: MCLK_256,
                bclk_div: 8,
            },
            slot_cfg: i2s_std_slot_config_t {
                data_bit_width: BW_16,
                slot_bit_width: SLOT_AUTO,
                slot_mode: STEREO,
                slot_mask: SLOT_BOTH,
                ws_width: 16,
                ws_pol: false,
                bit_shift: true,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
            },
            gpio_cfg: i2s_std_gpio_config_t {
                mclk: PIN_MCLK,
                bclk: PIN_BCLK,
                ws: PIN_WS,
                dout: PIN_DOUT,
                din: GPIO_NC,
                invert_flags: Default::default(),
            },
        };

        unsafe { esp!(i2s_channel_init_std_mode(tx, &std_cfg))? };

        Ok(Self(tx))
    }

    pub fn write_all(&mut self, data: &[u8]) -> Result<(), EspError> {
        let mut written = 0usize;
        unsafe {
            esp!(i2s_channel_write(
                self.0,
                data.as_ptr() as *const c_void,
                data.len(),
                &mut written,
                u32::MAX,
            ))
        }
    }

    pub fn disable(&mut self) {
        unsafe {
            let _ = i2s_channel_disable(self.0);
        }
    }

    pub fn enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.0)) }
    }
}

impl Drop for I2s {
    fn drop(&mut self) {
        unsafe {
            let _ = i2s_channel_disable(self.0);
            let _ = i2s_del_channel(self.0);
        }
    }
}
