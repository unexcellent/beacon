use esp_idf_sys::vTaskDelay;
use sstv::RgbPixel;

use crate::camera::CameraSensor;

use super::{OUTPUT_HEIGHT, OUTPUT_WIDTH};

#[derive(Clone)]
pub struct Image {
    pixels: Vec<RgbPixel>,
    index: usize,
}

impl Image {
    pub(super) unsafe fn new(
        src: *const u8,
        sensor: &CameraSensor,
        row_bytes: usize,
        wb_r: f32,
        wb_b: f32,
    ) -> Self {
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

                let pixel = if super::watermark::is_white_at(dx, dy) {
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

        Self { pixels, index: 0 }
    }

    pub fn width(&self) -> usize { OUTPUT_WIDTH }
    pub fn height(&self) -> usize { OUTPUT_HEIGHT }
}

impl Iterator for Image {
    type Item = RgbPixel;

    fn next(&mut self) -> Option<RgbPixel> {
        let pixel = self.pixels.get(self.index).copied();
        self.index += 1;
        pixel
    }
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

    let mut sr = 0u32; let mut cr = 0u32;
    let mut sg = 0u32; let mut cg = 0u32;
    let mut sb = 0u32; let mut cb = 0u32;

    for sy in sy0..sy1 {
        let row = src.add(sy * row_bytes);
        let yodd = sy & 1;
        for sx in sx0..sx1 {
            let v = raw10_pixel(row, sx);
            // RGGB Bayer: R at even row + even col, B at odd row + odd col
            match (yodd, sx & 1) {
                (0, 0) => { sr += v; cr += 1; }
                (1, 1) => { sb += v; cb += 1; }
                _      => { sg += v; cg += 1; }
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
