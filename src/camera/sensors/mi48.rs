//! The MI48Dx thermal-image processor paired with a SenXor™ LWIR sensor
//! (MI1602: raw 160×120), as a thermal camera: register control plus the
//! frame-averaging / normalisation pipeline.
//!
//! The raw sensor cannot be read directly; the MI48Dx performs per-pixel
//! calibration, bad-pixel correction and raw→temperature conversion, so the
//! host talks to the MI48Dx:
//!
//! ```text
//! MI1602 ──SenXor bus──► MI48Dx ──► host
//!                          I²C  (register control, via the interface bus)
//!                          SPI  (thermal frame readout, via the interface)
//!                          DATA_READY (mirrored in STATUS, polled over I²C)
//! ```
//!
//! Each pixel read back is a 16-bit unsigned temperature in units of 0.1 K,
//! MSB first on the wire ([`PixelFormat::Gray16`]).
//!
//! The chip clock-stretches SCL for tens of ms while busy (notably during boot
//! and between register accesses); the I2C bus behind the interface must tolerate
//! that (e.g. `scl_wait_us` ≈ 50 ms on esp-idf) or writes fail outright while
//! short reads occasionally squeak through. All ESP32 logic lives behind the
//! [`CameraInterface`]; this driver is platform-agnostic.

use std::time::Duration;

use embedded_hal::i2c::{ErrorType, I2c};
use sstv::RgbPixel;

use crate::camera::{Camera, CameraInterface, FrameFormat, Image, PixelFormat};

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

/// Frames to capture and average after warm-up. Random per-pixel sensor noise is
/// uncorrelated between frames, so averaging N frames cuts its amplitude by ~√N —
/// the main lever against the salt-and-pepper speckle in a single raw frame.
const AVERAGE_FRAMES: u32 = 8;
/// Frames to capture and discard on activation so the on-chip temporal/median
/// filters converge before the kept capture.
const WARMUP_FRAMES: u32 = 5;
/// Per-frame timeout waiting for the sensor to report frame-ready, matching the
/// reference's MI1602_DATA_READY_TIMEOUT_MS. Real frames arrive in tens of ms.
const FRAME_TIMEOUT_MS: u32 = 2_000;

pub struct Mi48<I> {
    interface: I,
    address: u8,
    /// Sensor grid in pixels (from the sensor's frame format).
    size: (usize, usize),
    output: (usize, usize),
    /// Mirror left↔right so the scene matches a co-mounted camera's orientation.
    mirror: bool,
}

impl<I> Mi48<I> {
    /// 7-bit I2C address with the ADDR pin strapped high (the reference's
    /// working strap); strapped low it is 0x40.
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x41;

    /// The raw frame format this sensor produces on its data interface. Exposed
    /// as a constant so the transport can be sized before the camera exists.
    pub const FORMAT: FrameFormat = FrameFormat {
        width: 160,
        height: 120,
        pixel: PixelFormat::Gray16,
    };

    /// Wrap a frame transport (which also carries the I2C control bus) as a
    /// thermal camera. `address` is the board-strapped 7-bit I2C address.
    pub fn new(interface: I, address: u8, output: (usize, usize), mirror: bool) -> Self {
        Self {
            interface,
            address,
            size: (Self::FORMAT.width, Self::FORMAT.height),
            output,
            mirror,
        }
    }
}

impl<I: CameraInterface> Mi48<I> {
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), <I::Bus as ErrorType>::Error> {
        self.interface.bus().write(self.address, &[reg, val])
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, <I::Bus as ErrorType>::Error> {
        let mut val = [0u8];
        self.interface.bus().write_read(self.address, &[reg], &mut val)?;
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

    /// Bring-up diagnostics and configuration. Deliberately infallible in the
    /// same way as the reference driver: an unresponsive chip is logged (every
    /// later capture then times out cleanly) rather than failing bring-up.
    pub fn init(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
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

    /// Single-shot chip: nothing to start — each frame is armed via `trigger`.
    fn start(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        Ok(())
    }

    /// Leave the chip idle (clears FRAME_MODE) so the next trigger starts clean.
    fn stop(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        self.write_reg(REG_FRAME_MODE, 0x00)
    }

    /// Trigger exactly one single-shot frame, header stripped (NO_HEADER) →
    /// pure pixel data in the output frame buffer once ready.
    fn trigger(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        self.write_reg(REG_FRAME_MODE, FRAME_MODE_SINGLE_FRAME | FRAME_MODE_NO_HEADER)
    }

    /// Whether a triggered frame has finished and is ready to read out.
    fn frame_ready(&mut self) -> Result<bool, <I::Bus as ErrorType>::Error> {
        Ok(self.read_reg(REG_STATUS)? & STATUS_DATA_READY != 0)
    }

    /// Trigger and read exactly one single-shot frame. Returns None (and leaves
    /// the chip idle) on timeout or a failed readout.
    fn capture_one(&mut self) -> Option<&[u8]> {
        if let Err(e) = self.trigger() {
            log::warn!("thermal: frame trigger write failed: {e:?}");
        }

        let mut waited = 0u32;
        let ready = loop {
            if matches!(self.frame_ready(), Ok(true)) {
                break true;
            }
            if waited >= FRAME_TIMEOUT_MS {
                break false;
            }
            std::thread::sleep(Duration::from_millis(5));
            waited += 5;
        };

        if !ready {
            // Leave the chip idle so the next trigger starts clean.
            let _ = self.stop();
            log::warn!("thermal: frame timed out waiting for frame-ready");
            return None;
        }
        self.interface.wait_frame(Duration::ZERO).ok()
    }
}

impl<I: CameraInterface> Camera for Mi48<I> {
    /// Single-shot sensors are triggered per frame and manage their own
    /// calibration, so there is nothing to power on.
    fn power_on(&mut self) {
        let _ = self.start();
    }

    /// Leaves the chip idle.
    fn power_off(&mut self) {
        let _ = self.stop();
    }

    /// Capture and discard warm-up frames so on-chip temporal/median filters —
    /// which keep state across captures — converge before the frames we keep.
    fn calibrate(&mut self) {
        for _ in 0..WARMUP_FRAMES {
            self.capture_one();
        }
    }

    /// Average [`AVERAGE_FRAMES`] frames per pixel, then map the result to a
    /// greyscale, upscaled [`Image`]. Random sensor noise is uncorrelated between
    /// frames, so the mean converges to the true temperature while the noise
    /// shrinks by ~√N — this is what kills the salt-and-pepper speckle. Call
    /// [`calibrate`](Self::calibrate) first so the on-chip filters have settled.
    fn receive_frame(&mut self) -> Image {
        let words_per_frame = self.size.0 * self.size.1;
        let mut acc = vec![0u32; words_per_frame];
        let mut n = 0u32;
        for _ in 0..AVERAGE_FRAMES {
            if let Some(frame) = self.capture_one()
                && frame.len() >= words_per_frame * 2
            {
                for (i, slot) in acc.iter_mut().enumerate() {
                    *slot += u16::from_be_bytes([frame[2 * i], frame[2 * i + 1]]) as u32;
                }
                n += 1;
            }
        }

        if n == 0 {
            log::error!("thermal: no frame captured — producing a blank image");
            return build_image(
                &vec![0u16; words_per_frame],
                self.size,
                self.output,
                self.mirror,
            );
        }
        let averaged: Vec<u16> = acc.iter().map(|&s| (s / n) as u16).collect();
        build_image(&averaged, self.size, self.output, self.mirror)
    }
}

/// Turn an averaged temperature frame into a greyscale, nearest-neighbour
/// scaled [`Image`], optionally flipped horizontally.
fn build_image(words: &[u16], size: (usize, usize), output: (usize, usize), mirror: bool) -> Image {
    let (src_w, src_h) = size;
    let (out_w, out_h) = output;

    // Robust auto-scale: clip the coldest and hottest ~1% of pixels before choosing the
    // black/white points. A few outlier/noisy pixels would otherwise stretch the whole
    // range and wash the low-contrast scene out into amplified noise.
    let mut sorted = words.to_vec();
    sorted.sort_unstable();
    let trim = sorted.len() / 100;
    let min = sorted[trim];
    let max = sorted[sorted.len() - 1 - trim];
    let range = (max.saturating_sub(min)).max(1) as f32;

    let mut pixels = Vec::with_capacity(out_w * out_h);
    for oy in 0..out_h {
        let sy = oy * src_h / out_h;
        for ox in 0..out_w {
            let mut sx = ox * src_w / out_w;
            if mirror {
                sx = (src_w - 1) - sx;
            }
            let v = words[sy * src_w + sx];
            let norm = v.saturating_sub(min) as f32 / range;
            pixels.push(grayscale(norm));
        }
    }
    Image::from_pixels(out_w, out_h, pixels)
}

/// Map a normalised temperature in `[0, 1]` to a greyscale pixel
/// (black = coldest, white = hottest).
fn grayscale(t: f32) -> RgbPixel {
    let g = (t.clamp(0.0, 1.0) * 255.0) as u8;
    RgbPixel::new(g, g, g)
}
