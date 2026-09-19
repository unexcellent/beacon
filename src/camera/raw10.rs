//! Shared pixel pipeline for packed-RAW10 Bayer sensors: metering for
//! auto-exposure and the demosaic / colour-correct / rotate-crop path that
//! turns a raw capture buffer into an [`Image`]. Used by the SC850SL (MOVE-IIIa)
//! and SC202CS (Tab5) drivers, which differ only in registers, not pixels.

use std::time::Duration;

use sstv::RgbPixel;

use super::auto_exposure::MeterResult;
use super::format::BayerChannel;
use super::{BayerOrder, FrameFormat, Image};

/// Demosaic, colour-correct and rotate-crop the raw Bayer capture buffer into an
/// [`Image`] at the requested output size.
pub fn build_image(
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
                sample_bayer_region(
                    src,
                    format.width,
                    format.height,
                    row_bytes,
                    order,
                    dx,
                    dy,
                    output,
                )
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
pub fn meter(frame: &[u8], format: &FrameFormat, order: BayerOrder) -> MeterResult {
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
        sr.checked_div(cr).map_or(0.0, |v| v as f32),
        sg.checked_div(cg).map_or(0.0, |v| v as f32),
        sb.checked_div(cb).map_or(0.0, |v| v as f32),
    )
}

fn apply_gamma(v: f32) -> u8 {
    ((v / 255.0).powf(1.0 / 2.2) * 255.0 + 0.5) as u8
}
