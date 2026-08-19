#![allow(unsafe_op_in_unsafe_fn)]
//! Driver for the MI1602 long-wave-infrared thermal camera module.
//!
//! The MI1602 is a raw 160×120 SenXor™ LWIR sensor and cannot be read directly.
//! It is paired with a companion **MI48Dx** thermal-image processor that performs
//! per-pixel calibration, bad-pixel correction and raw→temperature conversion. The
//! host (this ESP32-P4) therefore talks to the *MI48Dx*, not the MI1602:
//!
//! ```text
//! MI1602 ──SenXor bus──► MI48Dx ──► ESP32-P4
//!                          I²C  (register control)
//!                          SPI  (thermal frame readout)
//!                          DATA_READY (frame-ready signal)
//! ```
//!
//! The MI48Dx `MODE` pin is hard-wired to ground, selecting the SPI + I²C host
//! interface. Register access is I²C-only; the SPI slave is used purely to clock
//! out the temperature frame from the Output Frame Buffer when `DATA_READY` is high.
//!
//! Each pixel read back is a 16-bit unsigned temperature in units of 0.1 K.
//! We average several frames to suppress random per-pixel noise, normalise to a
//! robust (percentile-clipped) range, map to greyscale and nearest-neighbour
//! upscale the 160×120 array to the 320×240 grid expected by Robot36 SSTV.

use core::ffi::c_void;
use std::time::Duration;

use esp_idf_sys::*;
use sstv::RgbPixel;

use super::{Camera, Image, OUTPUT_HEIGHT, OUTPUT_WIDTH, watermark};

// ── Pin mapping: ESP32-P4 ↔ MI48Dx ──────────────────────────────────────────────
// SPI2 data bus (MI48Dx is the SPI slave; ESP32 is master, Mode 0, MSB-first).
const PIN_CS: i32 = 31; // G31_RMII_MDC  → SPI2 CS  (MI48 SSN, active low)
const PIN_CLK: i32 = 28; // G28_RMII_RXDV → SPI2 CLK
const PIN_MOSI: i32 = 30; // G30_RMII_RXD1 → SPI2 MOSI (host clocks dummy 0x0000)
const PIN_MISO: i32 = 29; // G29_RMII_RXD0 → SPI2 MISO (thermal data)
// I²C control bus (MI48Dx register access), driven by the HP I²C peripheral via the
// GPIO matrix. On the ESP32-P4 GPIO0-15 are the LP-capable pads, so these equal the
// schematic's LP_GPIO12/15.
const PIN_I2C_SDA: i32 = 12; // GPIO12 (LP_GPIO12)
const PIN_I2C_SCL: i32 = 15; // GPIO15 (LP_GPIO15)
// Frame-ready signal from the MI48Dx (input, active high).
// The reference driver runs with this line unused (-1) and polls STATUS.DATA_READY over
// I²C instead — the GPIO10 routing proved unreliable on the carrier, so we mirror that.
const PIN_DATA_READY: i32 = -1;
// MI48Dx RSTN (active-low reset). It must NOT float. Either pull it up to 3V3 on the
// board, or wire it to a spare ESP32 GPIO and set this constant to that GPIO number so
// the firmware issues a clean reset at startup. -1 = not host-driven.
const PIN_RESET: i32 = -1;

const SPI_HOST: spi_host_device_t = spi_host_device_t_SPI2_HOST;
/// 7.8 MHz: the reference driver's proven value (higher rates corrupted the frame CRC).
const SPI_CLOCK_HZ: i32 = 7_800_000;
/// A full frame exceeds the per-transaction size limit, so it is read in chunks. Unlike an
/// ordinary SPI slave, the MI48Dx requires CS to stay asserted across the ENTIRE frame: it
/// pauses on the SCLK gaps between chunks and resumes clocking out the same frame, so CS is
/// driven manually (see `read_frame`) rather than by the hardware per-transaction.
const SPI_CHUNK_BYTES: usize = 16_384;
/// Settle time on each manual CS edge before/after clocking, per the reference driver.
const SPI_CS_SETTLE_US: u32 = 100;

// ── MI48Dx I²C register map ──────────────────────────────────────────────────────
const I2C_ADDR: u16 = 0x41; // ADDR strapped high → 0x41 (the reference's working strap).
const I2C_TIMEOUT_MS: i32 = 100;
/// 100 kHz, matching the reference driver. Clock-stretch tolerance (`scl_wait_us`), not a
/// slow clock, is what makes the link reliable here.
const I2C_CLOCK_HZ: u32 = 100_000;

const REG_FRAME_MODE: u8 = 0xB1;
const REG_FW_VERSION_1: u8 = 0xB2; // [7:4] major, [3:0] minor
const REG_FW_VERSION_2: u8 = 0xB3; // build
const REG_STATUS: u8 = 0xB6;
const REG_MODULE_TYPE: u8 = 0xBB;
// Filter registers, per the reference driver's map (pysenxor mi48.py). The temporal filter
// is controlled at 0xD0 (0x00 off / 0x03 on) with its strength in 0xD1/0xD2; the median
// filter is a separate register at 0x30 (0x00 off / 0x01 on) — NOT a bit inside 0xD0.
const REG_FILTER_TEMPORAL: u8 = 0xD0;
const REG_FILTER_TEMPORAL_LSB: u8 = 0xD1;
const REG_FILTER_TEMPORAL_MSB: u8 = 0xD2;
const REG_FILTER_MEDIAN: u8 = 0x30;

// FRAME_MODE (0xB1) bits.
const FRAME_MODE_SINGLE_FRAME: u8 = 1 << 0; // capture exactly one frame, then idle
const FRAME_MODE_NO_HEADER: u8 = 1 << 5; // drop the 1-row frame header → pure pixel data

// STATUS (0xB6) bits.
const STATUS_DATA_READY: u8 = 1 << 4;
const STATUS_BOOTING_UP: u8 = 1 << 5;

// Filter enable/disable values (whole-register writes, not bitfields).
const FILTER_TEMPORAL_ON: u8 = 0x03;
const FILTER_MEDIAN_ON: u8 = 0x01;

// ── Sensor / output geometry ──────────────────────────────────────────────────────
const THERMAL_WIDTH: usize = 160;
const THERMAL_HEIGHT: usize = 120;
/// Words per frame in NO_HEADER mode (one 16-bit temperature per pixel).
const FRAME_WORDS: usize = THERMAL_WIDTH * THERMAL_HEIGHT;
const FRAME_BYTES: usize = FRAME_WORDS * 2;

// SSTV Robot36 output grid (`super::OUTPUT_WIDTH`/`OUTPUT_HEIGHT`) is exactly 2× the
// sensor, so upscaling is loss-free.

/// Settle delay after the I²C bus is reset and before the first probe, letting the rails
/// and the MI48Dx come up. Matches the reference driver's 2 s post-reset wait.
const BOOT_SETTLE_MS: u64 = 2_000;
/// Milliseconds to poll STATUS.BOOTING_UP before giving up, matching the reference's
/// MI1602_BOOT_TIMEOUT_MS. pysenxor polls with no timeout at all; 3 s is a safe bound.
const BOOT_TIMEOUT_MS: u32 = 3_000;
/// Frames to capture and average after warm-up. Random per-pixel sensor noise is
/// uncorrelated between frames, so averaging N frames cuts its amplitude by ~√N —
/// the main lever against the salt-and-pepper speckle in a single raw frame.
const AVERAGE_FRAMES: u32 = 8;
/// Per-frame timeout waiting for DATA_READY, matching the reference's
/// MI1602_DATA_READY_TIMEOUT_MS. Real frames arrive in tens of ms.
const FRAME_TIMEOUT_MS: u32 = 2_000;

pub struct ThermalCamera {
    spi: spi_device_handle_t,
    i2c_bus: i2c_master_bus_handle_t,
    i2c: i2c_master_dev_handle_t,
    /// DMA-capable receive buffer for one full frame, reused across captures.
    rx_buf: *mut u8,
}

impl ThermalCamera {
    pub fn try_new() -> crate::Result<Self> {
        log::info!("Thermal camera: initializing (MI1602 via MI48Dx)...");
        unsafe {
            Self::reset_mi48();
            let spi = Self::setup_spi().map_err(|_| crate::Error::ThermalInit)?;
            let (i2c_bus, i2c) = Self::setup_i2c().map_err(|_| crate::Error::ThermalInit)?;
            Self::setup_data_ready_gpio().map_err(|_| crate::Error::ThermalInit)?;

            let rx_buf = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT) as *mut u8;
            assert!(!rx_buf.is_null(), "MI48: frame buffer allocation failed");

            let cam = Self {
                spi,
                i2c_bus,
                i2c,
                rx_buf,
            };
            // Let the rails and the MI48Dx settle after the bus reset before the first
            // probe, as the reference driver does. (See BOOT_SETTLE_MS.)
            std::thread::sleep(Duration::from_millis(BOOT_SETTLE_MS));
            // Ground-truth diagnostics: who is actually on the I²C bus, and does the
            // configured device address respond?
            cam.scan_bus();
            match cam.read_reg(REG_STATUS) {
                Ok(s) => log::info!("MI48: I²C link OK at 0x{I2C_ADDR:02x} (STATUS=0x{s:02x})"),
                Err(e) => log::error!(
                    "MI48: no response at 0x{I2C_ADDR:02x} ({}). Check ADDR pin (0x40/0x41), \
                     SDA=LP_GPIO{PIN_I2C_SDA}/SCL=LP_GPIO{PIN_I2C_SCL} wiring, and external pull-ups.",
                    err_name(e)
                ),
            }
            cam.wait_for_boot();
            cam.log_identity();
            cam.configure_filters();
            Ok(cam)
        }
    }

    /// Trigger and read exactly one single-shot frame into `rx_buf`. Header stripped
    /// (NO_HEADER) → pure pixel data. Returns false (and leaves the chip idle) on timeout.
    unsafe fn capture_one(&self) -> bool {
        // Surface a failed trigger write: if this I²C write does not land, the MI48Dx is
        // never told to capture, so DATA_READY can never assert. Logging it distinguishes
        // "trigger didn't reach the chip" from "chip triggered but produced no frame".
        if let Err(e) = self.write_reg(REG_FRAME_MODE, FRAME_MODE_SINGLE_FRAME | FRAME_MODE_NO_HEADER)
        {
            log::warn!("MI48: FRAME_MODE trigger write failed: {}", err_name(e));
        }
        if self.wait_for_data_ready(FRAME_TIMEOUT_MS) {
            self.read_frame();
            true
        } else {
            let status = match self.read_reg(REG_STATUS) {
                Ok(s) => format!("0x{s:02x}"),
                Err(e) => err_name(e).to_string(),
            };
            // Leave the chip idle so the next trigger starts clean.
            let _ = self.write_reg(REG_FRAME_MODE, 0x00);
            log::warn!("MI48: frame timed out waiting for DATA_READY (STATUS={status})");
            false
        }
    }

    /// Read pixel `i` from `rx_buf`. Words are 16-bit, MSB byte first on the wire.
    unsafe fn word(&self, i: usize) -> u16 {
        ((*self.rx_buf.add(2 * i) as u16) << 8) | (*self.rx_buf.add(2 * i + 1) as u16)
    }

    // ── Setup ──────────────────────────────────────────────────────────────────────

    /// Issue a clean active-low reset to the MI48Dx if RSTN is wired to a GPIO.
    /// Timing follows the reference driver: assert low ≥50 µs, then ~50 ms to settle.
    unsafe fn reset_mi48() {
        if PIN_RESET < 0 {
            log::warn!(
                "MI48: RSTN not host-driven (PIN_RESET=-1). If the reset line floats the chip \
                 will not boot reliably — pull RSTN up to 3V3, or wire it to a GPIO and set PIN_RESET."
            );
            return;
        }
        let cfg = gpio_config_t {
            pin_bit_mask: 1u64 << PIN_RESET,
            mode: gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        let _ = esp!(gpio_config(&cfg));
        gpio_set_level(PIN_RESET as gpio_num_t, 0);
        esp_rom_delay_us(100);
        gpio_set_level(PIN_RESET as gpio_num_t, 1);
        std::thread::sleep(Duration::from_millis(50));
        log::info!("MI48: issued hardware reset via GPIO{PIN_RESET}");
    }

    unsafe fn setup_spi() -> Result<spi_device_handle_t, EspError> {
        let mut bus: spi_bus_config_t = core::mem::zeroed();
        bus.__bindgen_anon_1.mosi_io_num = PIN_MOSI;
        bus.__bindgen_anon_2.miso_io_num = PIN_MISO;
        bus.sclk_io_num = PIN_CLK;
        bus.__bindgen_anon_3.quadwp_io_num = -1;
        bus.__bindgen_anon_4.quadhd_io_num = -1;
        bus.max_transfer_sz = SPI_CHUNK_BYTES as i32;
        esp!(spi_bus_initialize(
            SPI_HOST,
            &bus,
            spi_common_dma_t_SPI_DMA_CH_AUTO as _
        ))?;

        let mut dev: spi_device_interface_config_t = core::mem::zeroed();
        dev.clock_speed_hz = SPI_CLOCK_HZ;
        dev.mode = 0; // MI48Dx requires SPI Mode 0.
        // CS is driven by hand (see `read_frame`) so it can stay asserted across the whole
        // multi-chunk frame; -1 tells the driver not to toggle it per transaction.
        dev.spics_io_num = -1;
        dev.queue_size = 1;
        let mut handle: spi_device_handle_t = core::ptr::null_mut();
        esp!(spi_bus_add_device(SPI_HOST, &dev, &mut handle))?;

        // Configure CS as a plain GPIO output, idle high (de-asserted).
        let cs_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << PIN_CS,
            mode: gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        esp!(gpio_config(&cs_cfg))?;
        gpio_set_level(PIN_CS as gpio_num_t, 1);
        Ok(handle)
    }

    unsafe fn setup_i2c() -> Result<(i2c_master_bus_handle_t, i2c_master_dev_handle_t), EspError> {
        let mut bus_cfg: i2c_master_bus_config_t = core::mem::zeroed();
        bus_cfg.i2c_port = i2c_port_t_I2C_NUM_0 as i2c_port_num_t;
        bus_cfg.sda_io_num = PIN_I2C_SDA as gpio_num_t;
        bus_cfg.scl_io_num = PIN_I2C_SCL as gpio_num_t;
        bus_cfg.__bindgen_anon_1.clk_source = soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT as _;
        bus_cfg.glitch_ignore_cnt = 7;
        // Weak internal pull-ups as a safety net; the board should still have proper
        // external 4.7 kΩ pull-ups on SDA/SCL for reliable comms.
        bus_cfg.flags.set_enable_internal_pullup(1);
        let mut bus: i2c_master_bus_handle_t = core::ptr::null_mut();
        esp!(i2c_new_master_bus(&bus_cfg, &mut bus))?;
        // Free the bus in case the MI48Dx (or a prior half-finished transfer) left SDA held
        // low after the P4 reset — without this the master can find the whole bus wedged and
        // no device ACKs. The reference driver does this immediately after creating the bus.
        esp!(i2c_master_bus_reset(bus))?;

        let mut dev_cfg: i2c_device_config_t = core::mem::zeroed();
        dev_cfg.dev_addr_length = i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = I2C_ADDR;
        dev_cfg.scl_speed_hz = I2C_CLOCK_HZ;
        // Tolerate the MI48Dx clock-stretching. It holds SCL low while busy — notably
        // during boot and between register accesses — for tens of ms. With too short a
        // wait the master aborts the transfer with ESP_ERR_INVALID_STATE: longer writes
        // (reg+data) fail outright while shorter reads occasionally squeak through, which
        // is exactly the "writes NACK, STATUS unreadable, stuck BOOTING_UP" symptom.
        // 50 ms is the value the reference MI1602 driver settled on.
        dev_cfg.scl_wait_us = 50_000;
        let mut dev: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(i2c_master_bus_add_device(bus, &dev_cfg, &mut dev))?;
        Ok((bus, dev))
    }

    /// Probe the whole 7-bit address space and log which devices ACK. If nothing
    /// answers, report what probing our own address returns so a broken bus
    /// (INVALID_STATE) is distinguishable from an empty one (NOT_FOUND).
    unsafe fn scan_bus(&self) {
        let mut found = Vec::new();
        for addr in 0x08u16..=0x77 {
            // 100 ms per probe: absent addresses NACK immediately, but a present MI48Dx
            // that is clock-stretching during boot needs a wide enough window to ACK.
            if i2c_master_probe(self.i2c_bus, addr, 100) == ESP_OK {
                found.push(addr);
            }
        }
        match found.len() {
            0 => {
                let e = i2c_master_probe(self.i2c_bus, I2C_ADDR, 100);
                log::error!(
                    "MI48: I²C scan found no devices (probe 0x{I2C_ADDR:02x} → {}). \
                     Check wiring, address, and pull-ups.",
                    err_name(e)
                );
            }
            1..=8 => {
                let list: Vec<String> = found.iter().map(|a| format!("0x{a:02x}")).collect();
                log::info!("MI48: I²C devices responding: {}", list.join(", "));
            }
            n => log::error!(
                "MI48: {n} addresses ACKed — this is a bus-integrity fault (noisy/stuck SDA), \
                 not {n} real devices. Suspect slow rise time (pull-ups too weak, wiring too \
                 long, or clock too fast) or a wiring problem on SDA/SCL."
            ),
        }
    }

    unsafe fn setup_data_ready_gpio() -> Result<(), EspError> {
        // -1 → the line is unused; frame-readiness is polled via STATUS over I²C instead.
        if PIN_DATA_READY < 0 {
            return Ok(());
        }
        let cfg = gpio_config_t {
            pin_bit_mask: 1u64 << PIN_DATA_READY,
            mode: gpio_mode_t_GPIO_MODE_INPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            // Pull down so a disconnected/idle line reads low instead of floating high
            // (the MI48Dx drives DATA_READY push-pull active-high, so this is safe).
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_ENABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        esp!(gpio_config(&cfg))
    }

    /// Block until the MI48Dx finishes booting (STATUS.BOOTING_UP clears); register
    /// writes and frame capture are only valid afterwards. Mirrors the reference's
    /// `mi1602_bootup`: clear any leftover streaming state, then poll STATUS.
    unsafe fn wait_for_boot(&self) {
        // Clear any leftover FRAME_MODE state before polling (may NACK while still booting).
        let _ = self.write_reg(REG_FRAME_MODE, 0x00);
        for _ in 0..(BOOT_TIMEOUT_MS / 10) {
            match self.read_reg(REG_STATUS) {
                Ok(s) if s & STATUS_BOOTING_UP == 0 => return,
                _ => {} // still booting, or a transient I²C error — keep polling
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        log::warn!(
            "MI48: boot not confirmed after {} s (STATUS unreadable or still booting)",
            BOOT_TIMEOUT_MS / 1000
        );
    }

    unsafe fn log_identity(&self) {
        match (
            self.read_reg(REG_FW_VERSION_1),
            self.read_reg(REG_FW_VERSION_2),
            self.read_reg(REG_MODULE_TYPE),
        ) {
            (Ok(v1), Ok(build), Ok(module)) => log::info!(
                "MI48: firmware {}.{}.{}, module type 0x{:02x}",
                v1 >> 4,
                v1 & 0x0F,
                build,
                module
            ),
            _ => log::warn!("MI48: could not read identity registers over I²C"),
        }
    }

    unsafe fn configure_filters(&self) {
        // Temporal filter (strength 0x0080) + median filter, via the reference's register
        // semantics: strength in 0xD1/0xD2, temporal enable at 0xD0, median at its own 0x30.
        let writes = [
            (REG_FILTER_TEMPORAL_LSB, 0x80),
            (REG_FILTER_TEMPORAL_MSB, 0x00),
            (REG_FILTER_TEMPORAL, FILTER_TEMPORAL_ON),
            (REG_FILTER_MEDIAN, FILTER_MEDIAN_ON),
        ];
        for (reg, val) in writes {
            if let Err(e) = self.write_reg(reg, val) {
                log::warn!(
                    "MI48: filter config write 0x{reg:02x} failed: {}",
                    err_name(e)
                );
            }
        }
        std::thread::sleep(Duration::from_millis(60));
    }

    // ── I²C register access ──────────────────────────────────────────────────────

    unsafe fn write_reg(&self, reg: u8, val: u8) -> Result<(), esp_err_t> {
        let buf = [reg, val];
        let err = i2c_master_transmit(self.i2c, buf.as_ptr(), 2, I2C_TIMEOUT_MS);
        if err == ESP_OK { Ok(()) } else { Err(err) }
    }

    unsafe fn read_reg(&self, reg: u8) -> Result<u8, esp_err_t> {
        let mut val = 0u8;
        let err = i2c_master_transmit_receive(self.i2c, &reg, 1, &mut val, 1, I2C_TIMEOUT_MS);
        if err == ESP_OK { Ok(val) } else { Err(err) }
    }

    // ── Frame readout ──────────────────────────────────────────────────────────────

    /// Wait for a frame to be ready. Primary signal is the DATA_READY GPIO; we also
    /// poll STATUS.DATA_READY over I²C as a fallback so capture still works during
    /// bring-up if the GPIO mapping is off.
    unsafe fn wait_for_data_ready(&self, timeout_ms: u32) -> bool {
        let mut waited = 0u32;
        loop {
            // When the DATA_READY GPIO is wired, trust it; otherwise (PIN_DATA_READY < 0,
            // our default) poll STATUS.DATA_READY over I²C, as the reference driver does.
            if PIN_DATA_READY >= 0 {
                if gpio_get_level(PIN_DATA_READY as gpio_num_t) != 0 {
                    return true;
                }
            } else if matches!(self.read_reg(REG_STATUS), Ok(s) if s & STATUS_DATA_READY != 0) {
                return true;
            }
            if waited >= timeout_ms {
                return false;
            }
            std::thread::sleep(Duration::from_millis(5));
            waited += 5;
        }
    }

    /// Clock one full frame out of the MI48Dx into `rx_buf`, in chunks that stay under the
    /// per-transaction size limit. CS is asserted low for the ENTIRE frame: the MI48Dx pauses
    /// on the SCLK gaps between chunks and resumes clocking out the same frame, so releasing
    /// CS mid-frame would desync it. MOSI is left idle (tx_buffer = NULL); the master's SCLK
    /// alone clocks the data out of the slave.
    unsafe fn read_frame(&self) {
        gpio_set_level(PIN_CS as gpio_num_t, 0);
        esp_rom_delay_us(SPI_CS_SETTLE_US);

        let mut offset = 0usize;
        while offset < FRAME_BYTES {
            let chunk = (FRAME_BYTES - offset).min(SPI_CHUNK_BYTES);
            let mut trans: spi_transaction_t = core::mem::zeroed();
            trans.length = chunk * 8; // bits to clock this chunk
            trans.__bindgen_anon_2.rx_buffer = self.rx_buf.add(offset) as *mut c_void;
            if spi_device_polling_transmit(self.spi, &mut trans) != ESP_OK {
                log::error!("MI48: SPI frame read failed at byte {offset}");
                break;
            }
            offset += chunk;
        }

        esp_rom_delay_us(SPI_CS_SETTLE_US);
        gpio_set_level(PIN_CS as gpio_num_t, 1);
    }

}

impl Camera for ThermalCamera {
    /// The MI48Dx has no host-controlled standby: it is triggered per frame and manages
    /// its own calibration, so there is nothing to power on. (RSTN is not host-driven —
    /// see `PIN_RESET`.)
    fn power_on(&mut self) {}

    /// See [`power_on`](Self::power_on): the MI48Dx has no host-controlled standby.
    fn power_off(&mut self) {}

    /// Capture and discard `frames` single-shot frames so the on-chip temporal/median
    /// filters — which keep state across captures — converge before the frames we keep.
    fn calibrate(&mut self, frames: u32) {
        for _ in 0..frames {
            unsafe { self.capture_one() };
        }
    }

    /// Average `AVERAGE_FRAMES` frames per pixel, then map the result to a greyscale,
    /// upscaled [`Image`]. Random sensor noise is uncorrelated between frames, so the mean
    /// converges to the true temperature while the noise shrinks by ~√N — this is what
    /// kills the salt-and-pepper speckle. Call [`calibrate`](Self::calibrate) first so the
    /// on-chip filters have settled.
    fn capture(&mut self) -> Image {
        unsafe {
            let mut acc = vec![0u32; FRAME_WORDS];
            let mut n = 0u32;
            for _ in 0..AVERAGE_FRAMES {
                if self.capture_one() {
                    for (i, slot) in acc.iter_mut().enumerate() {
                        *slot += self.word(i) as u32;
                    }
                    n += 1;
                }
            }

            if n == 0 {
                log::error!("MI48: no frame captured — transmitting a blank image");
                return build_image(&vec![0u16; FRAME_WORDS]);
            }
            let averaged: Vec<u16> = acc.iter().map(|&s| (s / n) as u16).collect();
            build_image(&averaged)
        }
    }
}

impl Drop for ThermalCamera {
    fn drop(&mut self) {
        unsafe { heap_caps_free(self.rx_buf as *mut c_void) };
    }
}

/// Human-readable name for an ESP error code (e.g. "ESP_ERR_TIMEOUT").
fn err_name(code: esp_err_t) -> &'static str {
    unsafe {
        core::ffi::CStr::from_ptr(esp_err_to_name(code))
            .to_str()
            .unwrap_or("unknown")
    }
}

/// Turn an averaged 160×120 temperature frame into a greyscale, 2×-upscaled [`Image`],
/// flipped horizontally to match the RGB camera's orientation.
fn build_image(words: &[u16]) -> Image {
    // Robust auto-scale: clip the coldest and hottest ~1% of pixels before choosing the
    // black/white points. A few outlier/noisy pixels would otherwise stretch the whole
    // range and wash the low-contrast scene out into amplified noise.
    let mut sorted = words.to_vec();
    sorted.sort_unstable();
    let trim = sorted.len() / 100;
    let min = sorted[trim];
    let max = sorted[sorted.len() - 1 - trim];
    let range = (max.saturating_sub(min)).max(1) as f32;

    let mut pixels = Vec::with_capacity(OUTPUT_WIDTH * OUTPUT_HEIGHT);
    for oy in 0..OUTPUT_HEIGHT {
        let sy = oy * THERMAL_HEIGHT / OUTPUT_HEIGHT;
        for ox in 0..OUTPUT_WIDTH {
            // Overlay the shared MOVE-IIIa watermark (same position as the RGB image).
            let pixel = if watermark::is_white_at(ox, oy) {
                RgbPixel::new(255, 255, 255)
            } else {
                // Mirror left↔right (flip horizontally) so the scene matches the RGB image.
                let sx = (THERMAL_WIDTH - 1) - (ox * THERMAL_WIDTH / OUTPUT_WIDTH);
                let v = words[sy * THERMAL_WIDTH + sx];
                let norm = v.saturating_sub(min) as f32 / range;
                grayscale(norm)
            };
            pixels.push(pixel);
        }
    }
    Image::from_pixels(OUTPUT_WIDTH, OUTPUT_HEIGHT, pixels)
}

/// Map a normalised temperature in `[0, 1]` to a greyscale pixel
/// (black = coldest, white = hottest).
fn grayscale(t: f32) -> RgbPixel {
    let g = (t.clamp(0.0, 1.0) * 255.0) as u8;
    RgbPixel::new(g, g, g)
}
