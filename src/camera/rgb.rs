//! The RGB camera: an SC850SL Bayer sensor over the ESP32-P4 CSI transport,
//! demosaiced and colour-corrected into an [`Image`] at a caller-chosen output
//! resolution, with closed-loop auto-exposure. The exposure control law lives
//! in [`auto_exposure`](super::auto_exposure); this module wires it to the
//! frame stream.

use std::time::Duration;

use esp_idf_sys::vTaskDelay;
use sstv::RgbPixel;

use super::auto_exposure::{
    AutoExposureLimits, AutoExposureState, AutoExposureStep, MeterResult, auto_exposure_step,
};
use super::esp::{CsiInterface, EspI2c};
use super::format::BayerChannel;
use super::sensors::Sc850sl;
use super::{BayerOrder, Camera, CameraError, FrameFormat, Image, PixelFormat};

/// Frame budget for auto-exposure convergence: at most this many metered frames
/// before the capture proceeds with whatever operating point was reached.
const AUTO_EXPOSURE_MAX_ITERS: u32 = 6;

/// Frames to capture and discard on activation: the sensor stream just needs to
/// stabilise after power-on before the kept capture.
const WARMUP_FRAMES: u32 = 2;

/// The stream runs continuously, so a frame always arrives; waiting is bounded
/// only by the transport staying alive (matches the reference behaviour).
const FRAME_WAIT: Duration = Duration::MAX;

pub struct RgbCamera {
    sensor: Sc850sl<EspI2c>,
    interface: CsiInterface,
    format: FrameFormat,
    order: BayerOrder,
    output: (usize, usize),
    black_level: u8,
    wb_r: f32,
    wb_b: f32,
    /// Current sensor integration time in lines (auto-exposure state, persists across activations).
    exposure: u32,
    /// Current analog gain as a linear multiplier (1.0 = unity).
    gain: f32,
}

impl RgbCamera {
    /// Compose an initialized sensor with its frame transport.
    ///
    /// The sensor must already have had [`init`](Sc850sl::init)
    /// run (board bring-up owns the reset/retry sequencing around it); this
    /// starts streaming, verifies frames arrive, and parks the sensor in standby.
    pub fn try_new(
        mut sensor: Sc850sl<EspI2c>,
        interface: CsiInterface,
        output: (usize, usize),
    ) -> Result<Self, CameraError> {
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

    /// Push a new operating point to the sensor, then wait for it to take effect
    /// (exposure and gain apply a frame or two later) before the next metering.
    fn apply_exposure(&mut self, point: AutoExposureState) {
        self.exposure = point.exposure;
        self.gain = point.gain;
        let _ = self.sensor.set_exposure(point.exposure);
        let _ = self.sensor.set_analog_gain(point.gain);
        let _ = self.interface.wait_frame(FRAME_WAIT);
        let _ = self.interface.wait_frame(FRAME_WAIT);
    }
}

impl Camera for RgbCamera {
    /// Enable sensor streaming.
    fn power_on(&mut self) {
        let _ = self.sensor.start();
    }

    /// Put the sensor into standby (stops streaming, saves power).
    fn power_off(&mut self) {
        let _ = self.sensor.stop();
    }

    /// Settle before the kept capture: converge auto-exposure, then discard a
    /// few warm-up frames so the on-chip filters stabilise.
    fn calibrate(&mut self) {
        let format = self.format;
        let order = self.order;
        let limits = AutoExposureLimits {
            max_exposure: self.sensor.exposure_range().1,
            max_gain: self.sensor.max_gain(),
        };

        // Closed-loop auto-exposure: meter each frame and drive the sensor
        // toward the converged band, up to a fixed frame budget. Exposure is the
        // primary lever, gain only once it saturates (gain amplifies noise).
        for iteration in 0..AUTO_EXPOSURE_MAX_ITERS {
            let Ok(frame) = self.interface.wait_frame(FRAME_WAIT) else {
                continue;
            };
            let metered = meter(frame, &format, order);
            log::info!(
                "auto-exposure iter {iteration}: p95={:.0} clip={:.1}% exp={} gain={:.2}x",
                metered.p_high,
                metered.clip * 100.0,
                self.exposure,
                self.gain,
            );

            let state = AutoExposureState {
                exposure: self.exposure,
                gain: self.gain,
            };
            match auto_exposure_step(state, limits, &metered) {
                AutoExposureStep::Converged => break,
                AutoExposureStep::Adjust(next) => self.apply_exposure(next),
            }
        }

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
fn meter(frame: &[u8], format: &FrameFormat, order: BayerOrder) -> MeterResult {
    const CLIP_LEVEL: usize = 250;
    const PERCENTILE: f32 = 0.95;
    let (width, height) = (format.width, format.height);
    let row_bytes = format.row_bytes();
    if frame.len() < format.bytes_per_frame() {
        return MeterResult {
            p_high: 0.0,
            clip: 0.0,
        };
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
        return MeterResult {
            p_high: 0.0,
            clip: 0.0,
        };
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
    MeterResult {
        p_high: p_high as f32,
        clip: clipped as f32 / count as f32,
    }
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
