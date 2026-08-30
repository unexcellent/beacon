//! The SmartSens SC850SL 8MP Bayer camera, in its 4K / 15 fps / 2-lane / RAW10
//! mode: register control plus the demosaic / auto-exposure pipeline. All ESP32
//! logic (the I2C bus and the CSI frame transport) lives behind the
//! [`CameraInterface`] this owns; the driver itself is platform-agnostic.

use std::time::Duration;

use embedded_hal::i2c::{ErrorType, I2c};
use sstv::RgbPixel;

use crate::camera::auto_exposure::{
    AutoExposureLimits, AutoExposureState, AutoExposureStep, MeterResult, auto_exposure_step,
};
use crate::camera::format::BayerChannel;
use crate::camera::{
    BayerOrder, Camera, CameraInterface, ColorCalibration, FrameFormat, Image, PixelFormat,
};

/// Frame length in lines (VTS, regs 0x320e/0x320f in the init table). Bounds the exposure time.
const VTS: u16 = 0x08ca;
/// Bayer mosaic order of this sensor's RAW10 output.
const ORDER: BayerOrder = BayerOrder::Rggb;

/// Frame budget for auto-exposure convergence: at most this many metered frames
/// before the capture proceeds with whatever operating point was reached.
const AUTO_EXPOSURE_MAX_ITERS: u32 = 6;
/// Frames to capture and discard on activation so the stream stabilises before
/// the kept capture.
const WARMUP_FRAMES: u32 = 2;
/// The stream runs continuously, so a frame always arrives; waiting is bounded
/// only by the transport staying alive (matches the reference behaviour).
const FRAME_WAIT: Duration = Duration::MAX;

pub struct Sc850sl<I> {
    interface: I,
    address: u8,
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

impl<I> Sc850sl<I> {
    /// Strap-default 7-bit I2C address.
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x30;

    /// The raw frame format this sensor produces on its data interface. Exposed
    /// as a constant so the transport can be sized before the camera exists.
    pub const FORMAT: FrameFormat = FrameFormat {
        width: 3840,
        height: 2160,
        pixel: PixelFormat::Raw10(ORDER),
    };

    /// Wrap a frame transport (which also carries the I2C control bus) as an RGB
    /// camera. `address` is the board-strapped 7-bit I2C address
    /// ([`DEFAULT_I2C_ADDRESS`](Self::DEFAULT_I2C_ADDRESS) unless re-strapped).
    pub fn new(interface: I, address: u8, output: (usize, usize)) -> Self {
        let calibration = Self::color_calibration();
        Self {
            interface,
            address,
            format: Self::FORMAT,
            order: ORDER,
            output,
            black_level: calibration.black_level,
            wb_r: calibration.red_gain,
            wb_b: calibration.blue_gain,
            exposure: Self::default_exposure(),
            gain: 1.0,
        }
    }

    /// Inclusive (min, max) integration time the sensor accepts, in lines.
    ///
    /// Upper bound is VTS minus a small guard band (datasheet limit is VTS-4;
    /// we keep a couple of extra lines of margin).
    fn exposure_range() -> (u32, u32) {
        (1, (VTS as u32).saturating_sub(8).max(1))
    }

    /// Integration time in lines after [`init`](Self::init).
    fn default_exposure() -> u32 {
        2080 // reg 0x3e01=0x82 in the init table -> 0x820 = 2080 lines
    }

    /// Largest usable analog gain as a linear multiplier.
    fn max_gain() -> f32 {
        48.0 // the analog-gain table tops out ~49.6x
    }

    /// Color-processing seeds consumed by the demosaic pipeline.
    fn color_calibration() -> ColorCalibration {
        ColorCalibration {
            black_level: 16,
            red_gain: 1.5,
            blue_gain: 1.5,
        }
    }
}

impl<I: CameraInterface> Sc850sl<I> {
    /// Write an 8-bit value to a 16-bit register address, retrying on bus errors.
    fn write(&mut self, reg: u16, val: u8) -> Result<(), <I::Bus as ErrorType>::Error> {
        let buf = [(reg >> 8) as u8, reg as u8, val];
        let mut result = self.interface.bus().write(self.address, &buf);
        for _ in 0..2 {
            if result.is_ok() {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
            result = self.interface.bus().write(self.address, &buf);
        }
        result
    }

    /// Write the sensor's register init sequence, out of reset and not streaming.
    pub fn init(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        // PLL latching regs require a settle delay after writing.
        const DELAYED_REGISTERS: [u16; 2] = [0x36e9, 0x36f9];

        for &(reg, val) in INIT_TABLE {
            self.write(reg, val)?;
            if DELAYED_REGISTERS.contains(&reg) {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        Ok(())
    }

    /// Start streaming frames.
    fn start(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        self.write(0x302c, 0x00)?;
        self.write(0x0100, 0x01)?;
        std::thread::sleep(Duration::from_millis(200));
        Ok(())
    }

    /// Stop streaming and enter a low-power idle state.
    fn stop(&mut self) -> Result<(), <I::Bus as ErrorType>::Error> {
        self.write(0x0100, 0x00)
    }

    /// Set the linear-mode integration time in lines.
    ///
    /// The SC850SL exposure field is a 20-bit value spread over three registers, in units
    /// of 1/16 line: {0x3e00[3:0], 0x3e01[7:0], 0x3e02[7:4]} == lines << 4. Equivalently we
    /// write the 16-bit line count as 0x3e00=[15:12], 0x3e01=[11:4], 0x3e02=[3:0]<<4.
    /// (Packing per SmartSens datasheet V1.10 / MOVE-IIIa cmos_inttime_update.)
    fn set_exposure(&mut self, lines: u32) -> Result<(), <I::Bus as ErrorType>::Error> {
        let (min, max) = Self::exposure_range();
        let l = lines.clamp(min, max);
        self.write(0x3e00, ((l & 0xf000) >> 12) as u8)?;
        self.write(0x3e01, ((l & 0x0ff0) >> 4) as u8)?;
        self.write(0x3e02, ((l & 0x000f) << 4) as u8)
    }

    /// Coarse gain (0x3e08) doubles per bucket — 0x03/0x07/0x23/0x27/0x2f/0x3f = 1/2/4/8/16/32x;
    /// fine gain (0x3e09) runs 0x40 (= bucket base) up to 0x7f (≈2x the base). Values and
    /// bucket layout are from the datasheet-derived AgainInfo table.
    fn set_analog_gain(&mut self, gain: f32) -> Result<(), <I::Bus as ErrorType>::Error> {
        const BUCKETS: [(f32, u8); 6] = [
            (1.0, 0x03),
            (2.0, 0x07),
            (4.0, 0x23),
            (8.0, 0x27),
            (16.0, 0x2f),
            (32.0, 0x3f),
        ];
        let g = gain.clamp(1.0, Self::max_gain());
        let (base, coarse) = BUCKETS
            .iter()
            .rev()
            .find(|&&(b, _)| g >= b)
            .copied()
            .unwrap_or(BUCKETS[0]);
        let fine = ((0x40 as f32 * g / base).round() as i32).clamp(0x40, 0x7f) as u8;
        self.write(0x3e08, coarse)?;
        self.write(0x3e09, fine)
    }

    /// Push a new operating point to the sensor, then wait for it to take effect
    /// (exposure and gain apply a frame or two later) before the next metering.
    fn apply_exposure(&mut self, point: AutoExposureState) {
        self.exposure = point.exposure;
        self.gain = point.gain;
        let _ = self.set_exposure(point.exposure);
        let _ = self.set_analog_gain(point.gain);
        let _ = self.interface.wait_frame(FRAME_WAIT);
        let _ = self.interface.wait_frame(FRAME_WAIT);
    }
}

impl<I: CameraInterface> Camera for Sc850sl<I> {
    /// Enable sensor streaming.
    fn power_on(&mut self) {
        let _ = self.start();
    }

    /// Put the sensor into standby (stops streaming, saves power).
    fn power_off(&mut self) {
        let _ = self.stop();
    }

    /// Settle before the kept capture: converge auto-exposure, then discard a
    /// few warm-up frames so the on-chip filters stabilise.
    fn calibrate(&mut self) {
        let format = self.format;
        let order = self.order;
        let limits = AutoExposureLimits {
            max_exposure: Self::exposure_range().1,
            max_gain: Self::max_gain(),
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

// Init table for SC850SL mode: 4K / 15 fps / 2-lane / RAW10 (table_2, 191 entries)
// Source: MOVE-IIIa-Imager/firmware/components/sc850sl/init_tables/sc850sl_table_2.c
#[rustfmt::skip]
const INIT_TABLE: &[(u16, u8)] = &[
    (0x0103, 0x01), (0x0100, 0x00), (0x36e9, 0x80), (0x36f9, 0x80),
    (0x36ea, 0x17), (0x36eb, 0x0c), (0x36ec, 0x4a), (0x36ed, 0x24),
    (0x36fa, 0xcb), (0x36fb, 0x13), (0x36fc, 0x00), (0x36fd, 0x07),
    (0x36e9, 0x20), (0x36f9, 0x53), (0x3018, 0x3a), (0x3019, 0xfc),
    (0x301a, 0x30), (0x301e, 0x3c), (0x301f, 0x38), (0x302a, 0x00),
    (0x3031, 0x0a), (0x3032, 0x20), (0x3033, 0x22), (0x3037, 0x00),
    (0x303e, 0xb4), (0x320c, 0x04), (0x320d, 0x4c), (0x320e, 0x08),
    (0x320f, 0xca), (0x3211, 0x0f), (0x3213, 0x07), (0x3226, 0x00),
    (0x3227, 0x03), (0x3250, 0x40), (0x3253, 0x08), (0x327e, 0x00),
    (0x3280, 0x00), (0x3281, 0x00), (0x3301, 0x3c), (0x3304, 0x30),
    (0x3306, 0xe8), (0x3308, 0x10), (0x3309, 0x70), (0x330a, 0x01),
    (0x330b, 0xe0), (0x330d, 0x10), (0x3314, 0x92), (0x331e, 0x29),
    (0x331f, 0x69), (0x3333, 0x10), (0x3347, 0x05), (0x3348, 0xd0),
    (0x3352, 0x01), (0x3356, 0x38), (0x335d, 0x60), (0x3362, 0x70),
    (0x338f, 0x80), (0x33af, 0x48), (0x33fe, 0x00), (0x3400, 0x12),
    (0x3406, 0x04), (0x3410, 0x12), (0x3416, 0x06), (0x3433, 0x01),
    (0x3440, 0x12), (0x3446, 0x08), (0x3478, 0x01), (0x3479, 0x01),
    (0x347a, 0x02), (0x347b, 0x01), (0x347c, 0x04), (0x347d, 0x01),
    (0x3616, 0x0c), (0x3620, 0x92), (0x3622, 0x74), (0x3629, 0x74),
    (0x362a, 0xf0), (0x362b, 0x0f), (0x362d, 0x00), (0x3630, 0x68),
    (0x3633, 0x22), (0x3634, 0x22), (0x3635, 0x20), (0x3637, 0x06),
    (0x3638, 0x26), (0x363b, 0x06), (0x363c, 0x07), (0x363d, 0x05),
    (0x363e, 0x8f), (0x3648, 0xe0), (0x3649, 0x0a), (0x364a, 0x06),
    (0x364c, 0x6a), (0x3650, 0x3d), (0x3654, 0x40), (0x3656, 0x68),
    (0x3657, 0x0f), (0x3658, 0x3d), (0x365c, 0x40), (0x365e, 0x68),
    (0x3901, 0x04), (0x3904, 0x20), (0x3905, 0x91), (0x391e, 0x83),
    (0x3928, 0x04), (0x3933, 0xa0), (0x3934, 0x0a), (0x3935, 0x68),
    (0x3936, 0x00), (0x3937, 0x20), (0x3938, 0x0a), (0x3946, 0x20),
    (0x3961, 0x40), (0x3962, 0x40), (0x3963, 0xc8), (0x3964, 0xc8),
    (0x3965, 0x40), (0x3966, 0x40), (0x3967, 0x00), (0x39cd, 0xc8),
    (0x39ce, 0xc8), (0x3e01, 0x82), (0x3e02, 0x00), (0x3e0e, 0x02),
    (0x3e0f, 0x00), (0x3e1c, 0x0f), (0x3e23, 0x00), (0x3e24, 0x00),
    (0x3e53, 0x00), (0x3e54, 0x00), (0x3e68, 0x00), (0x3e69, 0x80),
    (0x3e73, 0x00), (0x3e74, 0x00), (0x3e86, 0x03), (0x3e87, 0x40),
    (0x3f02, 0x24), (0x4424, 0x02), (0x4501, 0xc4), (0x4509, 0x20),
    (0x4561, 0x12), (0x4800, 0x24), (0x4837, 0x0b), (0x4900, 0x24),
    (0x4937, 0x0b), (0x5000, 0x0e), (0x500f, 0x35), (0x5020, 0x00),
    (0x5787, 0x10), (0x5788, 0x06), (0x5789, 0x00), (0x578a, 0x18),
    (0x578b, 0x0c), (0x578c, 0x00), (0x5790, 0x10), (0x5791, 0x06),
    (0x5792, 0x01), (0x5793, 0x18), (0x5794, 0x0c), (0x5795, 0x01),
    (0x5799, 0x06), (0x57a2, 0x60), (0x59e0, 0xfe), (0x59e1, 0x40),
    (0x59e2, 0x38), (0x59e3, 0x30), (0x59e4, 0x20), (0x59e5, 0x38),
    (0x59e6, 0x30), (0x59e7, 0x20), (0x59e8, 0x3f), (0x59e9, 0x38),
    (0x59ea, 0x30), (0x59eb, 0x3f), (0x59ec, 0x38), (0x59ed, 0x30),
    (0x59ee, 0xfe), (0x59ef, 0x40), (0x59f4, 0x38), (0x59f5, 0x30),
    (0x59f6, 0x20), (0x59f7, 0x38), (0x59f8, 0x30), (0x59f9, 0x20),
    (0x59fa, 0x3f), (0x59fb, 0x38), (0x59fc, 0x30), (0x59fd, 0x3f),
    (0x59fe, 0x38), (0x59ff, 0x30), (0x0100, 0x00),
];

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
        // Yield periodically so the demosaic of a multi-megapixel frame does not
        // starve the scheduler / trip the task watchdog.
        if dy % 40 == 0 {
            std::thread::sleep(Duration::from_millis(1));
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
