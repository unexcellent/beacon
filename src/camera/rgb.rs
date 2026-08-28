//! An RGB camera: any Bayer [`CameraSensor`](crate::camera::sensor::CameraSensor)
//! paired with any [`CameraInterface`], demosaiced and colour-corrected into an
//! [`Image`] at a caller-chosen output resolution, with closed-loop auto-exposure.

use std::time::Duration;

use esp_idf_sys::vTaskDelay;
use sstv::RgbPixel;

use super::format::BayerChannel;
use super::interface::CameraInterface;
use super::sensor::{BayerSensor, ExposureControl};
use super::{BayerOrder, Camera, CameraError, FrameFormat, Image, PixelFormat};

// Auto-exposure tuning. Metering works in the 8-bit MSB space (0..255) on the high percentile
// of the frame (see `meter`) rather than the mean, so bimodal orbit scenes meter sanely.
const AE_TARGET_LUMA: f32 = 230.0; // drive the high percentile to just below saturation
const AE_LUMA_LOW: f32 = 205.0; // converged band (lower bound)
const AE_LUMA_HIGH: f32 = 248.0; // converged band (upper bound)
const AE_CLIP_LIMIT: f32 = 0.02; // tolerate up to 2% near-saturated pixels
const AE_HL_SCALE: f32 = 0.7; // forced exposure cut per step while highlights clip
const AE_MAX_ITERS: u32 = 6; // frame budget for convergence

/// Frames to capture and discard on activation: the sensor stream just needs to
/// stabilise after power-on before the kept capture.
const WARMUP_FRAMES: u32 = 2;

/// The stream runs continuously, so a frame always arrives; waiting is bounded
/// only by the transport staying alive (matches the reference behaviour).
const FRAME_WAIT: Duration = Duration::MAX;

pub struct RgbCamera<S, I> {
    sensor: S,
    interface: I,
    format: FrameFormat,
    order: BayerOrder,
    output: (usize, usize),
    black_level: u8,
    wb_r: f32,
    wb_b: f32,
    /// Current sensor integration time in lines (AE state, persists across activations).
    exposure: u32,
    /// Current analog gain as a linear multiplier (1.0 = unity).
    gain: f32,
}

impl<S: BayerSensor + ExposureControl, I: CameraInterface> RgbCamera<S, I> {
    /// Compose an initialized sensor with its frame transport.
    ///
    /// The sensor must already have had [`init`](super::sensor::CameraSensor::init)
    /// run (board bring-up owns the reset/retry sequencing around it); this
    /// starts streaming, verifies frames arrive, and parks the sensor in standby.
    pub fn try_new(mut sensor: S, interface: I, output: (usize, usize)) -> Result<Self, CameraError> {
        let format = sensor.format();
        let PixelFormat::Raw10(order) = format.pixel else {
            // The demosaic pipeline only reads packed RAW10 mosaics.
            return Err(CameraError::UnsupportedFormat);
        };

        if sensor.start().is_err() {
            return Err(CameraError::Bus);
        }

        let calibration = sensor.color_calibration();
        let mut cam = Self {
            black_level: calibration.black_level,
            wb_r: calibration.red_gain,
            wb_b: calibration.blue_gain,
            exposure: sensor.default_exposure(),
            gain: 1.0,
            sensor,
            interface,
            format,
            order,
            output,
        };

        // Discard a few frames to verify streaming works, then go to standby.
        for _ in 0..3 {
            if cam.interface.wait_frame(Duration::from_secs(5)).is_err() {
                return Err(CameraError::Transport);
            }
        }
        cam.power_off();

        Ok(cam)
    }

    /// Closed-loop auto-exposure: meter the raw frame and drive the sensor's integration
    /// time (and analog gain, only once exposure saturates) so bright scenes stop clipping.
    ///
    /// Exposure is the primary lever and is preferred all the way to its ceiling before any
    /// gain is added, since gain only amplifies noise. Highlight-priority metering forces a
    /// cut whenever a bright source clips, independent of the average brightness.
    fn auto_expose(&mut self) {
        let format = self.format;
        let order = self.order;
        for i in 0..AE_MAX_ITERS {
            let Ok(frame) = self.interface.wait_frame(FRAME_WAIT) else {
                continue;
            };
            let (p_high, clip) = meter(frame, &format, order);

            let converged =
                clip <= AE_CLIP_LIMIT && p_high >= AE_LUMA_LOW && p_high <= AE_LUMA_HIGH;
            log::info!(
                "AE iter {i}: p95={p_high:.0} clip={:.1}% exp={} gain={:.2}x -> {}",
                clip * 100.0,
                self.exposure,
                self.gain,
                if converged { "converged" } else { "adjust" }
            );
            if converged {
                return;
            }

            // Desired multiplicative change in total light. Clipping forces a reduction even
            // if the percentile looks fine (a small, very bright spot in an otherwise dim scene).
            let luma_scale = AE_TARGET_LUMA / p_high.max(1.0);
            let scale = if clip > AE_CLIP_LIMIT {
                luma_scale.min(AE_HL_SCALE)
            } else {
                luma_scale
            };

            // Distribute the change over exposure (preferred) then gain.
            let max_exp = self.sensor.exposure_range().1 as f32;
            let max_gain = self.sensor.max_gain();
            let light = self.exposure as f32 * self.gain;
            let desired = (light * scale).clamp(1.0, max_exp * max_gain);
            let (exp, gain) = if desired <= max_exp {
                (desired.max(1.0), 1.0)
            } else {
                (max_exp, (desired / max_exp).min(max_gain))
            };
            self.exposure = exp.round() as u32;
            self.gain = gain;

            let _ = self.sensor.set_exposure(self.exposure);
            let _ = self.sensor.set_analog_gain(self.gain);

            // Let the new exposure/gain take effect (applied on the next frame) before re-metering.
            let _ = self.interface.wait_frame(FRAME_WAIT);
            let _ = self.interface.wait_frame(FRAME_WAIT);
        }
    }
}

impl<S: BayerSensor + ExposureControl, I: CameraInterface> Camera for RgbCamera<S, I> {
    /// Enable sensor streaming and converge auto-exposure before the next capture.
    fn power_on(&mut self) {
        let _ = self.sensor.start();
        self.auto_expose();
    }

    /// Put the sensor into standby (stops streaming, saves power).
    fn power_off(&mut self) {
        let _ = self.sensor.stop();
    }

    fn calibrate(&mut self) {
        for _ in 0..WARMUP_FRAMES {
            let _ = self.interface.wait_frame(FRAME_WAIT);
        }
    }

    fn receive_frame(&mut self) -> Image {
        let format = self.format;
        let order = self.order;
        let output = self.output;
        let (black_level, wb_r, wb_b) = (self.black_level, self.wb_r, self.wb_b);
        match self.interface.wait_frame(FRAME_WAIT) {
            Ok(frame) if frame.len() >= format.bytes_per_frame() => {
                build_image(frame, &format, order, output, black_level, wb_r, wb_b)
            }
            _ => {
                log::error!("RGB: no frame available — producing a blank image");
                let pixels = vec![RgbPixel::new(0, 0, 0); output.0 * output.1];
                Image::from_pixels(output.0, output.1, pixels)
            }
        }
    }
}

// ── Raw frame → Image ───────────────────────────────────────────────────────────

/// Demosaic, colour-correct and rotate-crop the raw Bayer capture buffer into an
/// [`Image`] at the requested output size.
fn build_image(
    frame: &[u8],
    format: &FrameFormat,
    order: BayerOrder,
    output: (usize, usize),
    black_level: u8,
    wb_r: f32,
    wb_b: f32,
) -> Image {
    let (out_w, out_h) = output;
    let bl = black_level as f32;
    let bl_scale = 255.0 / (255.0 - bl).max(1.0);
    let src = frame.as_ptr();
    let row_bytes = format.row_bytes();

    let mut pixels = Vec::with_capacity(out_w * out_h);

    for dy in 0..out_h {
        if dy % 40 == 0 {
            unsafe { vTaskDelay(1) };
        }
        for dx in 0..out_w {
            let (ar, ag, ab) = unsafe {
                sample_bayer_region(src, format.width, format.height, row_bytes, order, dx, dy, output)
            };
            let lr = ((ar - bl) * bl_scale).max(0.0);
            let lg = ((ag - bl) * bl_scale).max(0.0);
            let lb = ((ab - bl) * bl_scale).max(0.0);

            pixels.push(RgbPixel::new(
                apply_gamma(lr * wb_r),
                apply_gamma(lg),
                apply_gamma(lb * wb_b),
            ));
        }
    }

    Image::from_pixels(out_w, out_h, pixels)
}

/// Meter the raw capture buffer for auto-exposure.
///
/// Samples the green channel (the luma proxy in a Bayer mosaic) on a coarse grid and returns
/// `(p_high, clipped_fraction)` from the 8-bit MSB path: `p_high` is the high-percentile luma
/// (0..255) and `clipped_fraction` the share of samples at/above the near-saturation threshold.
///
/// We meter a high percentile rather than the mean because in orbit the frame is often bimodal
/// (bright Earth + black space + specular glint): a mean is meaningless there, but the percentile
/// tracks "how bright are the bright parts" and degrades gracefully. The clip fraction is the
/// hard guard on top of it.
fn meter(frame: &[u8], format: &FrameFormat, order: BayerOrder) -> (f32, f32) {
    const CLIP_LEVEL: usize = 250;
    const PERCENTILE: f32 = 0.95;
    let (width, height) = (format.width, format.height);
    let row_bytes = format.row_bytes();
    if frame.len() < format.bytes_per_frame() {
        return (0.0, 0.0);
    }
    let src = frame.as_ptr();
    // Even steps so we always land on the same-parity (green) sites; ~64 samples/axis.
    let ystep = (height / 64).max(2) & !1;
    let xstep = (width / 64).max(2) & !1;

    let mut hist = [0u32; 256];
    let mut count = 0u64;

    let mut sy = 0;
    while sy < height {
        let row = unsafe { src.add(sy * row_bytes) };
        let mut sx = order.green_col_parity_on_even_rows();
        while sx < width {
            hist[unsafe { raw10_pixel(row, sx) } as usize & 0xff] += 1;
            count += 1;
            sx += xstep;
        }
        sy += ystep;
    }

    if count == 0 {
        return (0.0, 0.0);
    }

    // Percentile: smallest level whose cumulative count reaches PERCENTILE of the samples.
    let target = (PERCENTILE * count as f32) as u64;
    let mut cum = 0u64;
    let mut p_high = 255usize;
    for (level, &n) in hist.iter().enumerate() {
        cum += n as u64;
        if cum >= target {
            p_high = level;
            break;
        }
    }

    let clipped: u64 = hist[CLIP_LEVEL..].iter().map(|&n| n as u64).sum();
    (p_high as f32, clipped as f32 / count as f32)
}

// Read the 8 MSBs of the x-th pixel from a packed RAW10 row.
// Layout: 4 pixels per 5-byte group — bytes 0-3 hold each pixel's top 8 bits,
// byte 4 holds the 2 LSBs of all four (discarded here, not needed for an 8-bit path).
#[inline(always)]
unsafe fn raw10_pixel(row: *const u8, x: usize) -> u32 {
    unsafe { *row.add((x >> 2) * 5 + (x & 3)) as u32 }
}

// Box-filter the Bayer source region that maps to output pixel (dx, dy), returning
// per-channel averages. The source is rotated 90° counter-clockwise and cropped to
// fill the output undistorted: output x scans full source rows, output y scans a
// centred crop of source columns (reversed, which makes the rotation counter-clockwise).
// Demosaic parity uses the real source (sy, sx), so mosaic reconstruction stays correct
// for any Bayer order.
#[allow(clippy::too_many_arguments)]
unsafe fn sample_bayer_region(
    src: *const u8,
    src_width: usize,
    src_height: usize,
    row_bytes: usize,
    order: BayerOrder,
    dx: usize,
    dy: usize,
    output: (usize, usize),
) -> (f32, f32, f32) {
    let (out_w, out_h) = output;
    // Keep the centred span of source columns whose width, once the full source height
    // maps to the output width, gives the output's aspect ratio — i.e. crop-to-fill.
    let kept = out_h * src_height / out_w;
    let crop_lo = (src_width - kept) / 2;

    // output x → source rows (full height)
    let sy0 = (dx * src_height) / out_w;
    let sy1 = (((dx + 1) * src_height) / out_w)
        .max(sy0 + 2)
        .min(src_height);
    // output y → source columns (centred crop), reversed for counter-clockwise
    let c0 = crop_lo + (dy * kept) / out_h;
    let c1 = (crop_lo + ((dy + 1) * kept) / out_h)
        .max(c0 + 2)
        .min(src_width);
    let sx0 = src_width - c1;
    let sx1 = src_width - c0;

    let mut sr = 0u32;
    let mut cr = 0u32;
    let mut sg = 0u32;
    let mut cg = 0u32;
    let mut sb = 0u32;
    let mut cb = 0u32;

    for sy in sy0..sy1 {
        let row = unsafe { src.add(sy * row_bytes) };
        for sx in sx0..sx1 {
            let v = unsafe { raw10_pixel(row, sx) };
            match order.channel_at(sy, sx) {
                BayerChannel::Red => {
                    sr += v;
                    cr += 1;
                }
                BayerChannel::Blue => {
                    sb += v;
                    cb += 1;
                }
                BayerChannel::Green => {
                    sg += v;
                    cg += 1;
                }
            }
        }
    }

    (
        if cr > 0 { (sr / cr) as f32 } else { 0.0 },
        if cg > 0 { (sg / cg) as f32 } else { 0.0 },
        if cb > 0 { (sb / cb) as f32 } else { 0.0 },
    )
}

fn apply_gamma(v: f32) -> u8 {
    ((v / 255.0).powf(1.0 / 2.2) * 255.0 + 0.5) as u8
}
