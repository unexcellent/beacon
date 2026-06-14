use esp_idf_sys::vTaskDelay;
use sstv::RgbPixel;

use super::{OUTPUT_HEIGHT, OUTPUT_WIDTH};

static mut WB_R: f32 = 0.0;
static mut WB_B: f32 = 0.0;
static mut GAMMA_LUT: [u8; 256] = [0u8; 256];

pub(super) unsafe fn init(wb_r_seed: f32, wb_b_seed: f32) {
    WB_R = wb_r_seed;
    WB_B = wb_b_seed;
    init_gamma();
}

unsafe fn init_gamma() {
    for i in 0..256usize {
        let c = (i as f32 / 255.0).powf(1.0 / 2.2);
        let v = (c * 255.0 + 0.5) as u32;
        GAMMA_LUT[i] = if v > 255 { 255 } else { v as u8 };
    }
}

pub unsafe fn current_wb_gains() -> (f32, f32) {
    (*core::ptr::addr_of!(WB_R), *core::ptr::addr_of!(WB_B))
}

unsafe fn gamma_lookup(v: usize) -> u8 {
    GAMMA_LUT[v]
}

unsafe fn update_wb(fr: u64, fg: u64, fb: u64) {
    if fr > 0 && fg > 0 && fb > 0 {
        let gr = (fg as f32 / fr as f32).clamp(0.5, 4.0);
        let gb = (fg as f32 / fb as f32).clamp(0.5, 4.0);
        WB_R = WB_R * 0.5 + gr * 0.5;
        WB_B = WB_B * 0.5 + gb * 0.5;
    }
}

#[derive(Clone)]
pub struct Image {
    src: *const u8,
    src_width: usize,
    src_height: usize,
    row_bytes: usize,
    black_level: u8,
    wb_r: f32,
    wb_b: f32,
    bl_scale: f32,
    index: usize,
    fr: u64,
    fg: u64,
    fb: u64,
}

impl Image {
    pub(super) unsafe fn new(
        src: *const u8,
        src_width: usize,
        src_height: usize,
        row_bytes: usize,
        black_level: u8,
    ) -> Self {
        let (wb_r, wb_b) = current_wb_gains();
        let bl = black_level as f32;
        let bl_scale = 255.0 / (255.0 - bl).max(1.0);
        Self {
            src,
            src_width,
            src_height,
            row_bytes,
            black_level,
            wb_r,
            wb_b,
            bl_scale,
            index: 0,
            fr: 0,
            fg: 0,
            fb: 0,
        }
    }

    pub fn width(&self) -> usize { OUTPUT_WIDTH }
    pub fn height(&self) -> usize { OUTPUT_HEIGHT }

    // Read the 8 MSBs of the x-th pixel from a packed RAW10 row.
    // Layout: 4 pixels per 5-byte group — bytes 0-3 hold each pixel's top 8 bits,
    // byte 4 holds the 2 LSBs of all four (discarded here, not needed for an 8-bit path).
    #[inline(always)]
    unsafe fn raw10_pixel(row: *const u8, x: usize) -> u32 {
        *row.add((x >> 2) * 5 + (x & 3)) as u32
    }

    // Box-filter the Bayer source region that maps to output pixel (dx, dy),
    // returning per-channel averages as floats.
    unsafe fn sample_bayer_region(&self, dx: usize, dy: usize) -> (f32, f32, f32) {
        let sy0 = (dy * self.src_height) / OUTPUT_HEIGHT;
        let sy1 = (((dy + 1) * self.src_height) / OUTPUT_HEIGHT)
            .max(sy0 + 2)
            .min(self.src_height);
        let sx0 = (dx * self.src_width) / OUTPUT_WIDTH;
        let sx1 = (((dx + 1) * self.src_width) / OUTPUT_WIDTH)
            .max(sx0 + 2)
            .min(self.src_width);

        let mut sr = 0u32; let mut cr = 0u32;
        let mut sg = 0u32; let mut cg = 0u32;
        let mut sb = 0u32; let mut cb = 0u32;

        for sy in sy0..sy1 {
            let row = self.src.add(sy * self.row_bytes);
            let yodd = sy & 1;
            for sx in sx0..sx1 {
                let v = Self::raw10_pixel(row, sx);
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

    fn correct_black_level(&self, r: f32, g: f32, b: f32) -> (f32, f32, f32) {
        let bl = self.black_level as f32;
        (
            ((r - bl) * self.bl_scale).max(0.0),
            ((g - bl) * self.bl_scale).max(0.0),
            ((b - bl) * self.bl_scale).max(0.0),
        )
    }

    fn apply_white_balance(&self, r: f32, b: f32) -> (f32, f32) {
        (r * self.wb_r, b * self.wb_b)
    }

    unsafe fn apply_gamma(v: f32) -> u8 {
        gamma_lookup(((v + 0.5) as i32).clamp(0, 255) as usize)
    }
}

impl Iterator for Image {
    type Item = RgbPixel;

    fn next(&mut self) -> Option<RgbPixel> {
        if self.index >= OUTPUT_WIDTH * OUTPUT_HEIGHT {
            return None;
        }

        // Yield to IDLE task every 40 rows so the task watchdog can reset.
        // Without this, the ~3 s frame computation starves IDLE for > 5 s and the WDT fires.
        if self.index % (OUTPUT_WIDTH * 40) == 0 {
            unsafe { vTaskDelay(1) };
        }

        let dx = self.index % OUTPUT_WIDTH;
        let dy = self.index / OUTPUT_WIDTH;
        self.index += 1;

        let (ar, ag, ab) = unsafe { self.sample_bayer_region(dx, dy) };
        let (lr, lg, lb) = self.correct_black_level(ar, ag, ab);

        self.fr += lr as u64;
        self.fg += lg as u64;
        self.fb += lb as u64;

        let (wr, wb) = self.apply_white_balance(lr, lb);
        Some(RgbPixel::new(
            unsafe { Self::apply_gamma(wr) },
            unsafe { Self::apply_gamma(lg) },
            unsafe { Self::apply_gamma(wb) },
        ))
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        unsafe { update_wb(self.fr, self.fg, self.fb) };
    }
}
