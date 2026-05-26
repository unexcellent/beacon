use core::ffi::c_void;
use std::f32::consts::PI;
use std::ptr;

use esp_idf_sys::{
    EspError,
    esp,
    // GPIO
    gpio_num_t_GPIO_NUM_NC as GPIO_NC,
    i2s_chan_config_t,
    i2s_chan_handle_t,
    i2s_channel_disable,
    i2s_channel_enable,
    i2s_channel_init_std_mode,
    i2s_channel_write,
    // Slot / bit-width
    i2s_data_bit_width_t_I2S_DATA_BIT_WIDTH_16BIT as BW_16,
    i2s_del_channel,
    i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256 as MCLK_256,
    i2s_new_channel,
    // Channel / role
    i2s_port_t_I2S_NUM_0 as I2S0,
    i2s_role_t_I2S_ROLE_MASTER as MASTER,
    i2s_slot_bit_width_t_I2S_SLOT_BIT_WIDTH_AUTO as SLOT_AUTO,
    i2s_slot_mode_t_I2S_SLOT_MODE_STEREO as STEREO,
    i2s_std_clk_config_t,
    i2s_std_config_t,
    i2s_std_gpio_config_t,
    i2s_std_slot_config_t,
    i2s_std_slot_mask_t_I2S_STD_SLOT_BOTH as SLOT_BOTH,
    // Clock source — I2S_CLK_SRC_DEFAULT (= 0) maps to XTAL on ESP32-P4 rev < 3.0.
    // PLL_F160M (the hal default) is only valid on rev ≥ 3.0 and asserts otherwise.
    soc_periph_i2s_clk_src_t_I2S_CLK_SRC_DEFAULT as CLK_DEFAULT,
};
use sstv::{Encoder, Mode, RgbPixel};

// Decoded at compile time by build.rs: 320 × 240 pixels, 3 bytes each.
static PATCH_RGB: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/patch_rgb.bin"));

const SAMPLE_RATE: u32 = 16_000;

// PCM5102A GPIO pins
const PIN_MCLK: i32 = 20; // SCK
const PIN_BCLK: i32 = 21; // BCK
const PIN_DOUT: i32 = 22; // DIN on DAC side
const PIN_WS: i32 = 23; // LRCK

// 512-sample chunks × 4 bytes/frame (stereo 16-bit) = 2 KB stack buffer per flush.
const CHUNK_SAMPLES: usize = 512;

/// Thin RAII wrapper around a raw I2S TX channel handle.
struct I2sTx(i2s_chan_handle_t);

impl I2sTx {
    fn new() -> Result<Self, EspError> {
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
            // Philips I2S: bit_shift=true, ws_width=data_bit_width, both slots active.
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

    fn write_all(&mut self, data: &[u8]) -> Result<(), EspError> {
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
}

impl I2sTx {
    fn disable(&mut self) {
        unsafe {
            let _ = i2s_channel_disable(self.0);
        }
    }

    fn enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.0)) }
    }
}

impl Drop for I2sTx {
    fn drop(&mut self) {
        unsafe {
            let _ = i2s_channel_disable(self.0);
            let _ = i2s_del_channel(self.0);
        }
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("SSTV beacon starting — Robot36 / 16 kHz / PCM5102A");

    let mut i2s = I2sTx::new().unwrap();

    loop {
        i2s.enable().unwrap();
        transmit(&mut i2s);
        i2s.disable();
        log::info!("Waiting 30 s before next transmission");
        std::thread::sleep(std::time::Duration::from_secs(30));
    }
}

fn transmit(i2s: &mut I2sTx) {
    log::info!("Transmission start");

    let pixels = PATCH_RGB
        .chunks_exact(3)
        .map(|p| RgbPixel::new(p[0], p[1], p[2]));

    let encoder = Encoder::new(Mode::Robot36, pixels).unwrap();

    // Stereo interleaved 16-bit: [L_lo, L_hi, R_lo, R_hi] per frame.
    // Left channel is silent; audio goes on right.
    let mut buf = [0u8; CHUNK_SAMPLES * 4];
    let mut buf_pos = 0usize;

    let mut phase = 0.0f32;
    let mut sample_adjust = 0.0f32;

    for tone in encoder {
        let freq = tone.frequency.hz() as f32;
        let duration_sec = tone.duration.ns() as f32 / 1_000_000_000.0;

        let exact_samples = duration_sec * SAMPLE_RATE as f32 + sample_adjust;
        let num_samples = exact_samples.round() as usize;
        sample_adjust = exact_samples - num_samples as f32;

        let phase_step = 2.0 * PI * freq / SAMPLE_RATE as f32;

        for _ in 0..num_samples {
            let sample = (phase.sin() * i16::MAX as f32) as i16;
            let [lo, hi] = sample.to_le_bytes();

            buf[buf_pos] = 0; // left lo  (silent)
            buf[buf_pos + 1] = 0; // left hi  (silent)
            buf[buf_pos + 2] = lo; // right lo (audio)
            buf[buf_pos + 3] = hi; // right hi (audio)
            buf_pos += 4;

            phase = (phase + phase_step) % (2.0 * PI);

            if buf_pos == buf.len() {
                i2s.write_all(&buf).unwrap();
                buf_pos = 0;
            }
        }
    }

    if buf_pos > 0 {
        i2s.write_all(&buf[..buf_pos]).unwrap();
    }

    log::info!("Transmission complete (~36 s Robot36 frame)");
}
