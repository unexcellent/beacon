//! Held-CS chunked SPI frame readout, as required by SenXor-style thermal
//! processors (MI48Dx): the host clocks the frame out of the slave's output
//! buffer, keeping CS asserted across the entire multi-chunk frame.

use core::ffi::c_void;
use std::time::Duration;

use esp_idf_sys::*;

use super::EspI2c;
use crate::camera::{CameraError, CameraInterface};

/// SPI wiring and readout parameters. The frame size comes from the sensor's
/// [`FrameFormat`](crate::camera::FrameFormat) at construction.
pub struct SpiFrameConfig {
    pub host: spi_host_device_t,
    pub cs_pin: i32,
    pub clk_pin: i32,
    pub mosi_pin: i32,
    pub miso_pin: i32,
    pub clock_hz: i32,
    /// A full frame exceeds the per-transaction size limit, so it is read in
    /// chunks of this many bytes.
    pub chunk_bytes: usize,
    /// Settle time on each manual CS edge before/after clocking.
    pub cs_settle_us: u32,
}

/// Pull-based transport: [`wait_frame`](Self::wait_frame) clocks a frame out
/// immediately, so callers must complete the readiness handshake (e.g. polling
/// the sensor's DATA_READY status) first.
///
/// CS is driven manually and stays asserted across the ENTIRE frame: the slave
/// pauses on the SCLK gaps between chunks and resumes clocking out the same
/// frame, so releasing CS mid-frame would desync it.
pub struct SpiFrameInterface {
    i2c: EspI2c,
    spi: spi_device_handle_t,
    cs_pin: i32,
    chunk_bytes: usize,
    cs_settle_us: u32,
    /// DMA-capable receive buffer for one full frame, reused across captures.
    rx_buf: *mut u8,
    frame_bytes: usize,
}

impl SpiFrameInterface {
    /// `i2c` is the control bus to the sensor; the SPI readout is configured
    /// from `config` for frames of `frame_bytes`.
    pub fn new(
        i2c: EspI2c,
        config: SpiFrameConfig,
        frame_bytes: usize,
    ) -> Result<Self, CameraError> {
        unsafe {
            let mut bus: spi_bus_config_t = core::mem::zeroed();
            bus.__bindgen_anon_1.mosi_io_num = config.mosi_pin;
            bus.__bindgen_anon_2.miso_io_num = config.miso_pin;
            bus.sclk_io_num = config.clk_pin;
            bus.__bindgen_anon_3.quadwp_io_num = -1;
            bus.__bindgen_anon_4.quadhd_io_num = -1;
            bus.max_transfer_sz = config.chunk_bytes as i32;
            if spi_bus_initialize(config.host, &bus, spi_common_dma_t_SPI_DMA_CH_AUTO as _)
                != ESP_OK
            {
                return Err(CameraError::Transport);
            }

            let mut dev: spi_device_interface_config_t = core::mem::zeroed();
            dev.clock_speed_hz = config.clock_hz;
            dev.mode = 0;
            dev.spics_io_num = -1;
            dev.queue_size = 1;
            let mut handle: spi_device_handle_t = core::ptr::null_mut();
            if spi_bus_add_device(config.host, &dev, &mut handle) != ESP_OK {
                return Err(CameraError::Transport);
            }

            let cs_cfg = gpio_config_t {
                pin_bit_mask: 1u64 << config.cs_pin,
                mode: gpio_mode_t_GPIO_MODE_OUTPUT,
                pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
                pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
                intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
                hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
            };
            if gpio_config(&cs_cfg) != ESP_OK {
                return Err(CameraError::Transport);
            }
            gpio_set_level(config.cs_pin as gpio_num_t, 1);

            let rx_buf = heap_caps_malloc(frame_bytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT) as *mut u8;
            assert!(!rx_buf.is_null(), "SPI frame buffer allocation failed");

            Ok(Self {
                i2c,
                spi: handle,
                cs_pin: config.cs_pin,
                chunk_bytes: config.chunk_bytes,
                cs_settle_us: config.cs_settle_us,
                rx_buf,
                frame_bytes,
            })
        }
    }
}

impl CameraInterface for SpiFrameInterface {
    type Bus = EspI2c;

    fn bus(&mut self) -> &mut EspI2c {
        &mut self.i2c
    }

    /// Clock one full frame out of the slave. The frame must already be ready;
    /// `timeout` is unused (the readout itself is bounded by the SPI clock).
    /// The returned slice lives in the readout buffer, valid until the next call.
    fn wait_frame(&mut self, _timeout: Duration) -> Result<&[u8], CameraError> {
        unsafe {
            gpio_set_level(self.cs_pin as gpio_num_t, 0);
            esp_rom_delay_us(self.cs_settle_us);

            let mut result = Ok(());
            let mut offset = 0usize;
            while offset < self.frame_bytes {
                let chunk = (self.frame_bytes - offset).min(self.chunk_bytes);
                let mut trans: spi_transaction_t = core::mem::zeroed();
                trans.length = chunk * 8; // bits to clock this chunk
                trans.__bindgen_anon_2.rx_buffer = self.rx_buf.add(offset) as *mut c_void;
                if spi_device_polling_transmit(self.spi, &mut trans) != ESP_OK {
                    log::error!("SPI frame read failed at byte {offset}");
                    result = Err(CameraError::Transport);
                    break;
                }
                offset += chunk;
            }

            esp_rom_delay_us(self.cs_settle_us);
            gpio_set_level(self.cs_pin as gpio_num_t, 1);

            result.map(|()| core::slice::from_raw_parts(self.rx_buf as *const u8, self.frame_bytes))
        }
    }
}

impl Drop for SpiFrameInterface {
    fn drop(&mut self) {
        unsafe { heap_caps_free(self.rx_buf as *mut c_void) };
    }
}
