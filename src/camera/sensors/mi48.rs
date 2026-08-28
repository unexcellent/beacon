//! Driver for the MI48Dx thermal-image processor paired with a SenXor™ LWIR
//! sensor (MI1602: raw 160×120).
//!
//! The raw sensor cannot be read directly; the MI48Dx performs per-pixel
//! calibration, bad-pixel correction and raw→temperature conversion, so the
//! host talks to the MI48Dx:
//!
//! ```text
//! MI1602 ──SenXor bus──► MI48Dx ──► host
//!                          I²C  (register control — this driver)
//!                          SPI  (thermal frame readout — SpiFrameInterface)
//!                          DATA_READY (mirrored in STATUS, polled over I²C)
//! ```
//!
//! Each pixel read back is a 16-bit unsigned temperature in units of 0.1 K,
//! MSB first on the wire ([`PixelFormat::Gray16`]).
//!
//! The chip clock-stretches SCL for tens of ms while busy (notably during boot
//! and between register accesses); the I2C bus it is handed must tolerate that
//! (e.g. `scl_wait_us` ≈ 50 ms on esp-idf) or writes fail outright while short
//! reads occasionally squeak through.

use std::time::Duration;

use embedded_hal::i2c::I2c;

use crate::camera::sensor::{CameraSensor, FrameTrigger};
use crate::camera::{FrameFormat, PixelFormat};

// ── MI48Dx I²C register map ──────────────────────────────────────────────────────
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

/// Milliseconds to poll STATUS.BOOTING_UP before giving up, matching the reference's
/// MI1602_BOOT_TIMEOUT_MS. pysenxor polls with no timeout at all; 3 s is a safe bound.
const BOOT_TIMEOUT_MS: u32 = 3_000;

pub struct Mi48<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Mi48<I2C> {
    /// 7-bit I2C address with the ADDR pin strapped high (the reference's
    /// working strap); strapped low it is 0x40.
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x41;

    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }
}

impl<I2C: I2c> Mi48<I2C> {
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg, val])
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut val = [0u8];
        self.i2c.write_read(self.address, &[reg], &mut val)?;
        Ok(val[0])
    }

    /// Block until the chip finishes booting (STATUS.BOOTING_UP clears); register
    /// writes and frame capture are only valid afterwards. Mirrors the reference's
    /// `mi1602_bootup`: clear any leftover streaming state, then poll STATUS.
    fn wait_for_boot(&mut self) {
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

    fn log_identity(&mut self) {
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

    fn configure_filters(&mut self) {
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
                log::warn!("MI48: filter config write 0x{reg:02x} failed: {e:?}");
            }
        }
        std::thread::sleep(Duration::from_millis(60));
    }
}

impl<I2C: I2c> CameraSensor for Mi48<I2C> {
    type Error = I2C::Error;

    fn format(&self) -> FrameFormat {
        FrameFormat {
            width: 160,
            height: 120,
            pixel: PixelFormat::Gray16,
        }
    }

    /// Bring-up diagnostics and configuration. Deliberately infallible in the
    /// same way as the reference driver: an unresponsive chip is logged (every
    /// later capture then times out cleanly) rather than failing bring-up.
    fn init(&mut self) -> Result<(), Self::Error> {
        match self.read_reg(REG_STATUS) {
            Ok(s) => log::info!(
                "MI48: I²C link OK at 0x{:02x} (STATUS=0x{s:02x})",
                self.address
            ),
            Err(e) => log::error!(
                "MI48: no response at 0x{:02x} ({e:?}). Check the ADDR strap (0x40/0x41), \
                 SDA/SCL wiring, and external pull-ups.",
                self.address
            ),
        }
        self.wait_for_boot();
        self.log_identity();
        self.configure_filters();
        Ok(())
    }

    /// Single-shot chip: nothing to start — each frame is armed via
    /// [`trigger`](FrameTrigger::trigger).
    fn start(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Leave the chip idle (clears FRAME_MODE) so the next trigger starts clean.
    fn stop(&mut self) -> Result<(), Self::Error> {
        self.write_reg(REG_FRAME_MODE, 0x00)
    }
}

impl<I2C: I2c> FrameTrigger for Mi48<I2C> {
    /// Trigger exactly one single-shot frame, header stripped (NO_HEADER) →
    /// pure pixel data in the output frame buffer once ready.
    fn trigger(&mut self) -> Result<(), Self::Error> {
        self.write_reg(REG_FRAME_MODE, FRAME_MODE_SINGLE_FRAME | FRAME_MODE_NO_HEADER)
    }

    fn frame_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.read_reg(REG_STATUS)? & STATUS_DATA_READY != 0)
    }
}
