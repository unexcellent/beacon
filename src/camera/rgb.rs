//! The SC850SL RGB camera: a MIPI CSI-2 Bayer sensor streamed through the ESP32-P4
//! camera controller + ISP, demosaiced and colour-corrected into a Robot36 [`Image`].

use core::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use esp_idf_sys::*;
use sstv::RgbPixel;

use super::{Camera, CameraInterface, CameraSensor, Image, OUTPUT_HEIGHT, OUTPUT_WIDTH, watermark};

static FRAME_READY: AtomicBool = AtomicBool::new(false);

// Auto-exposure tuning. Metering works in the 8-bit MSB space (0..255) on the high percentile
// of the frame (see `meter`) rather than the mean, so bimodal orbit scenes meter sanely.
const AE_TARGET_LUMA: f32 = 230.0; // drive the high percentile to just below saturation
const AE_LUMA_LOW: f32 = 205.0; // converged band (lower bound)
const AE_LUMA_HIGH: f32 = 248.0; // converged band (upper bound)
const AE_CLIP_LIMIT: f32 = 0.02; // tolerate up to 2% near-saturated pixels
const AE_HL_SCALE: f32 = 0.7; // forced exposure cut per step while highlights clip
const AE_GAIN_MAX: f32 = 48.0; // analog-gain ceiling (sensor table tops out ~49.6x)
const AE_MAX_ITERS: u32 = 6; // frame budget for convergence

// Holds the capture buffer pointer and length for the CSI DMA callbacks.
// Heap-allocated so its address is stable after being registered as userdata.
pub(crate) struct CaptureBuffer {
    buf: *mut c_void,
    len: usize,
    row_bytes: usize,
}
impl CaptureBuffer {
    fn len(&self) -> usize {
        self.len
    }
    fn row_bytes(&self) -> usize {
        self.row_bytes
    }
}

pub(crate) unsafe extern "C" fn on_get_new_trans(
    _: esp_cam_ctlr_handle_t,
    trans: *mut esp_cam_ctlr_trans_t,
    user_ctx: *mut c_void,
) -> bool {
    let cap = &*(user_ctx as *const CaptureBuffer);
    (*trans).buffer = cap.buf;
    (*trans).buflen = cap.len;
    false
}

pub(crate) unsafe extern "C" fn on_trans_finished(
    _: esp_cam_ctlr_handle_t,
    _: *mut esp_cam_ctlr_trans_t,
    _: *mut c_void,
) -> bool {
    FRAME_READY.store(true, Ordering::Release);
    false
}

pub(crate) struct InnerCamera {
    pub(crate) csi: esp_cam_ctlr_handle_t,
    pub(crate) i2c_dev: i2c_master_dev_handle_t,
}

impl InnerCamera {
    unsafe fn queue_receive(&self, capture_buf: &CaptureBuffer) -> Result<(), EspError> {
        let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
        trans.buffer = capture_buf.buf;
        trans.buflen = capture_buf.len();
        esp!(esp_cam_ctlr_receive(self.csi, &mut trans, 100))
    }
}

pub struct RgbCamera {
    sensor: CameraSensor,
    capture_buffer: Box<CaptureBuffer>,
    inner: InnerCamera,
    wb_r: f32,
    wb_b: f32,
    /// Current sensor integration time in lines (AE state, persists across activations).
    exposure: u32,
    /// Current analog gain as a linear multiplier (1.0 = unity).
    gain: f32,
}

impl RgbCamera {
    pub fn try_new(sensor: CameraSensor, interface: CameraInterface) -> crate::Result<Self> {
        unsafe {
            let capture_buffer = Self::allocate_capture_buffer(&sensor);

            let inner = interface
                .init(&sensor, &capture_buffer)
                .map_err(|_| crate::Error::RgbInit)?;
            inner
                .queue_receive(&capture_buffer)
                .map_err(|_| crate::Error::RgbInit)?;

            if !sensor.enable(inner.i2c_dev) {
                return Err(crate::Error::RgbInit);
            }

            let mut cam = Self {
                wb_r: sensor.red_gain_seed,
                wb_b: sensor.blue_gain_seed,
                exposure: sensor.default_exposure as u32,
                gain: 1.0,
                sensor,
                inner,
                capture_buffer,
            };

            // Calibrate briefly to verify streaming works, then go to standby.
            cam.calibrate(3);
            cam.power_off();

            Ok(cam)
        }
    }

    /// Closed-loop auto-exposure: meter the raw frame and drive the sensor's integration
    /// time (and analog gain, only once exposure saturates) so bright scenes stop clipping.
    ///
    /// Exposure is the primary lever and is preferred all the way to its ceiling before any
    /// gain is added, since gain only amplifies noise. Highlight-priority metering forces a
    /// cut whenever a bright source clips, independent of the average brightness.
    fn auto_expose(&mut self) {
        for i in 0..AE_MAX_ITERS {
            self.wait_for_frame();
            let (p_high, clip) = unsafe {
                meter(
                    self.capture_buffer.buf as *const u8,
                    self.sensor.resolution.0,
                    self.sensor.resolution.1,
                    self.capture_buffer.row_bytes(),
                )
            };

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
            let max_exp = self.sensor.max_exposure() as f32;
            let light = self.exposure as f32 * self.gain;
            let desired = (light * scale).clamp(1.0, max_exp * AE_GAIN_MAX);
            let (exp, gain) = if desired <= max_exp {
                (desired.max(1.0), 1.0)
            } else {
                (max_exp, (desired / max_exp).min(AE_GAIN_MAX))
            };
            self.exposure = exp.round() as u32;
            self.gain = gain;

            unsafe {
                self.sensor.set_exposure(self.inner.i2c_dev, self.exposure);
                self.sensor.set_analog_gain(self.inner.i2c_dev, self.gain);
            }

            // Let the new exposure/gain take effect (applied on the next frame) before re-metering.
            self.wait_for_frame();
            self.wait_for_frame();
        }
    }

    fn wait_for_frame(&self) {
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    unsafe fn allocate_capture_buffer(sensor: &CameraSensor) -> Box<CaptureBuffer> {
        let cap_row_bytes = sensor.resolution.0 * 10 / 8;
        let capture_fb_bytes = cap_row_bytes * sensor.resolution.1;
        let cap_buf =
            heap_caps_aligned_calloc(64, 1, capture_fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        assert!(!cap_buf.is_null(), "capture buffer allocation failed");
        Box::new(CaptureBuffer {
            buf: cap_buf,
            len: capture_fb_bytes,
            row_bytes: cap_row_bytes,
        })
    }
}

impl Camera for RgbCamera {
    /// Enable sensor streaming and converge auto-exposure before the next capture.
    fn power_on(&mut self) {
        unsafe {
            self.sensor.enable(self.inner.i2c_dev);
        }
        self.auto_expose();
    }

    /// Put the sensor into standby (stops streaming, saves power).
    fn power_off(&mut self) {
        unsafe {
            self.sensor.disable(self.inner.i2c_dev);
        }
    }

    fn calibrate(&mut self, frames: u32) {
        for _ in 0..frames {
            self.wait_for_frame();
        }
    }

    fn capture(&mut self) -> Image {
        self.wait_for_frame();
        unsafe {
            build_image(
                self.capture_buffer.buf as *const u8,
                &self.sensor,
                self.capture_buffer.row_bytes(),
                self.wb_r,
                self.wb_b,
            )
        }
    }
}

impl Drop for RgbCamera {
    fn drop(&mut self) {
        unsafe {
            heap_caps_free(self.capture_buffer.buf);
        }
    }
}

// ── Raw frame → Image ───────────────────────────────────────────────────────────

/// Demosaic, colour-correct and rotate-crop the raw Bayer capture buffer into a
/// Robot36 [`Image`], overlaying the shared MOVE-IIIa watermark.
unsafe fn build_image(
    src: *const u8,
    sensor: &CameraSensor,
    row_bytes: usize,
    wb_r: f32,
    wb_b: f32,
) -> Image {
    let bl = sensor.black_level as f32;
    let bl_scale = 255.0 / (255.0 - bl).max(1.0);

    let mut pixels = Vec::with_capacity(OUTPUT_WIDTH * OUTPUT_HEIGHT);

    for dy in 0..OUTPUT_HEIGHT {
        if dy % 40 == 0 {
            vTaskDelay(1);
        }
        for dx in 0..OUTPUT_WIDTH {
            let (ar, ag, ab) = sample_bayer_region(
                src,
                sensor.resolution.0,
                sensor.resolution.1,
                row_bytes,
                dx,
                dy,
            );
            let lr = ((ar - bl) * bl_scale).max(0.0);
            let lg = ((ag - bl) * bl_scale).max(0.0);
            let lb = ((ab - bl) * bl_scale).max(0.0);

            let pixel = if watermark::is_white_at(dx, dy) {
                RgbPixel::new(255, 255, 255)
            } else {
                RgbPixel::new(
                    apply_gamma(lr * wb_r),
                    apply_gamma(lg),
                    apply_gamma(lb * wb_b),
                )
            };
            pixels.push(pixel);
        }
    }

    Image::from_pixels(OUTPUT_WIDTH, OUTPUT_HEIGHT, pixels)
}

/// Meter the raw capture buffer for auto-exposure.
///
/// Samples the green channel (the luma proxy in an RGGB mosaic) on a coarse grid and returns
/// `(p_high, clipped_fraction)` from the 8-bit MSB path: `p_high` is the high-percentile luma
/// (0..255) and `clipped_fraction` the share of samples at/above the near-saturation threshold.
///
/// We meter a high percentile rather than the mean because in orbit the frame is often bimodal
/// (bright Earth + black space + specular glint): a mean is meaningless there, but the percentile
/// tracks "how bright are the bright parts" and degrades gracefully. The clip fraction is the
/// hard guard on top of it.
unsafe fn meter(src: *const u8, width: usize, height: usize, row_bytes: usize) -> (f32, f32) {
    const CLIP_LEVEL: usize = 250;
    const PERCENTILE: f32 = 0.95;
    // Even steps so we always land on green sites (even row + odd col in RGGB); ~64 samples/axis.
    let ystep = (height / 64).max(2) & !1;
    let xstep = (width / 64).max(2) & !1;

    let mut hist = [0u32; 256];
    let mut count = 0u64;

    let mut sy = 0;
    while sy < height {
        let row = src.add(sy * row_bytes);
        let mut sx = 1; // odd column on an even row -> green
        while sx < width {
            hist[raw10_pixel(row, sx) as usize & 0xff] += 1;
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
    *row.add((x >> 2) * 5 + (x & 3)) as u32
}

// Box-filter the Bayer source region that maps to output pixel (dx, dy), returning
// per-channel averages. The source is rotated 90° counter-clockwise and cropped to
// fill the output undistorted: output x scans full source rows, output y scans a
// centred crop of source columns (reversed, which makes the rotation counter-clockwise).
// Demosaic parity uses the real source (sy, sx), so RGGB reconstruction stays correct.
unsafe fn sample_bayer_region(
    src: *const u8,
    src_width: usize,
    src_height: usize,
    row_bytes: usize,
    dx: usize,
    dy: usize,
) -> (f32, f32, f32) {
    // Keep the centred span of source columns whose width, once the full source height
    // maps to the output width, gives the output's aspect ratio — i.e. crop-to-fill.
    let kept = OUTPUT_HEIGHT * src_height / OUTPUT_WIDTH;
    let crop_lo = (src_width - kept) / 2;

    // output x → source rows (full height)
    let sy0 = (dx * src_height) / OUTPUT_WIDTH;
    let sy1 = (((dx + 1) * src_height) / OUTPUT_WIDTH)
        .max(sy0 + 2)
        .min(src_height);
    // output y → source columns (centred crop), reversed for counter-clockwise
    let c0 = crop_lo + (dy * kept) / OUTPUT_HEIGHT;
    let c1 = (crop_lo + ((dy + 1) * kept) / OUTPUT_HEIGHT)
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
        let row = src.add(sy * row_bytes);
        let yodd = sy & 1;
        for sx in sx0..sx1 {
            let v = raw10_pixel(row, sx);
            // RGGB Bayer: R at even row + even col, B at odd row + odd col
            match (yodd, sx & 1) {
                (0, 0) => {
                    sr += v;
                    cr += 1;
                }
                (1, 1) => {
                    sb += v;
                    cb += 1;
                }
                _ => {
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
