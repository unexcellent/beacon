//! A thermal camera: any single-shot [`FrameTrigger`] sensor emitting
//! [`Gray16`](PixelFormat::Gray16) temperature frames, paired with any
//! [`CameraInterface`], averaged and normalised into a greyscale [`Image`].
//!
//! We average several frames to suppress random per-pixel noise, normalise to a
//! robust (percentile-clipped) range, map to greyscale and nearest-neighbour
//! scale the sensor grid to the caller-chosen output resolution.

use std::time::Duration;

use sstv::RgbPixel;

use super::interface::CameraInterface;
use super::sensor::FrameTrigger;
use super::{Camera, CameraError, Image, PixelFormat};

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

pub struct ThermalCamera<S, I> {
    sensor: S,
    interface: I,
    /// Sensor grid in pixels (from the sensor's frame format).
    size: (usize, usize),
    output: (usize, usize),
    /// Mirror left↔right so the scene matches a co-mounted camera's orientation.
    mirror: bool,
}

impl<S: FrameTrigger, I: CameraInterface> ThermalCamera<S, I> {
    /// Compose an initialized sensor with its frame transport.
    ///
    /// The sensor must already have had [`init`](super::sensor::CameraSensor::init)
    /// run (board bring-up owns bus settling and diagnostics around it).
    pub fn try_new(
        sensor: S,
        interface: I,
        output: (usize, usize),
        mirror: bool,
    ) -> Result<Self, CameraError> {
        let format = sensor.format();
        if format.pixel != PixelFormat::Gray16 {
            // The thermal pipeline only reads 16-bit grayscale temperature frames.
            return Err(CameraError::UnsupportedFormat);
        }
        Ok(Self {
            sensor,
            interface,
            size: (format.width, format.height),
            output,
            mirror,
        })
    }

    /// Trigger and read exactly one single-shot frame. Returns None (and leaves
    /// the chip idle) on timeout or a failed readout.
    fn capture_one(&mut self) -> Option<&[u8]> {
        // Surface a failed trigger write: if it does not land, the sensor is never
        // told to capture, so frame-ready can never assert. Logging it distinguishes
        // "trigger didn't reach the chip" from "chip triggered but produced no frame".
        if let Err(e) = self.sensor.trigger() {
            log::warn!("thermal: frame trigger write failed: {e:?}");
        }

        let mut waited = 0u32;
        let ready = loop {
            if matches!(self.sensor.frame_ready(), Ok(true)) {
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
            let _ = self.sensor.stop();
            log::warn!("thermal: frame timed out waiting for frame-ready");
            return None;
        }
        self.interface.wait_frame(Duration::ZERO).ok()
    }
}

impl<S: FrameTrigger, I: CameraInterface> Camera for ThermalCamera<S, I> {
    /// Single-shot sensors are triggered per frame and manage their own
    /// calibration, so there is nothing to power on.
    fn power_on(&mut self) {
        let _ = self.sensor.start();
    }

    /// See [`power_on`](Self::power_on); [`stop`](super::sensor::CameraSensor::stop)
    /// leaves the chip idle.
    fn power_off(&mut self) {
        let _ = self.sensor.stop();
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
                    // Words are 16-bit, MSB byte first on the wire.
                    *slot += u16::from_be_bytes([frame[2 * i], frame[2 * i + 1]]) as u32;
                }
                n += 1;
            }
        }

        if n == 0 {
            log::error!("thermal: no frame captured — producing a blank image");
            return build_image(&vec![0u16; words_per_frame], self.size, self.output, self.mirror);
        }
        let averaged: Vec<u16> = acc.iter().map(|&s| (s / n) as u16).collect();
        build_image(&averaged, self.size, self.output, self.mirror)
    }
}

/// Turn an averaged temperature frame into a greyscale, nearest-neighbour
/// scaled [`Image`], optionally flipped horizontally.
fn build_image(
    words: &[u16],
    size: (usize, usize),
    output: (usize, usize),
    mirror: bool,
) -> Image {
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
