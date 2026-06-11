#![allow(unsafe_op_in_unsafe_fn)]

use esp_idf_sys::vTaskDelay;

static mut WB_R: f32 = 0.0;
static mut WB_B: f32 = 0.0;
static mut GAMMA_LUT: [u8; 256] = [0u8; 256];

pub unsafe fn init(wb_r_seed: f32, wb_b_seed: f32) {
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

// Read the 8 MSBs of the x-th pixel from a packed RAW10 row.
// Layout: 4 pixels per 5-byte group — bytes 0-3 hold each pixel's top 8 bits,
// byte 4 holds the 2 LSBs of all four (discarded here, not needed for an 8-bit path).
#[inline(always)]
unsafe fn raw10_get8(row: *const u8, x: usize) -> u32 {
    *row.add((x >> 2) * 5 + (x & 3)) as u32
}

// Box-filter downscale from capture resolution (packed RAW10) → dw×dh RGB888.
//
// Pipeline per output pixel:
//   box-filter demosaic → black-level subtract → range-restore → gray-world WB
//   → sRGB gamma → pack RGB888
//
// Gray-world AWB is updated once per frame from BL-corrected linear means; gains
// are IIR-smoothed (α = 0.5) so they converge within ~10 frames.
pub unsafe fn process_frame(
    src: *const u8,
    dst: *mut u8,
    dw: usize,
    dh: usize,
    sw: usize,
    sh: usize,
    row_bytes: usize,
    black_level: u8,
) {
    // Precompute which (row_parity, col_parity) pair maps to R vs B so the match
    // is lifted out of the hot inner loop by the compiler.
    let (red_yodd, red_xodd) = (0, 0);
    let (blue_yodd, blue_xodd) = (1, 1);

    let out = core::slice::from_raw_parts_mut(dst, dw * dh * 3);
    let mut fr: u64 = 0;
    let mut fg: u64 = 0;
    let mut fb: u64 = 0;
    let wb_r = *core::ptr::addr_of!(WB_R);
    let wb_b = *core::ptr::addr_of!(WB_B);
    let bl = black_level as f32;
    let bl_scale = 255.0f32 / (255.0f32 - bl).max(1.0);

    for dy in 0..dh {
        // Yield every 40 rows so the IDLE task can run and reset the task watchdog.
        // Without this, the ~3 s ISP computation starves IDLE for > 5 s and the WDT fires.
        if dy % 40 == 0 {
            vTaskDelay(1);
        }

        let sy0 = (dy * sh) / dh;
        let sy1 = (((dy + 1) * sh) / dh).max(sy0 + 2).min(sh);
        for dx in 0..dw {
            let sx0 = (dx * sw) / dw;
            let sx1 = (((dx + 1) * sw) / dw).max(sx0 + 2).min(sw);
            let mut sr: u32 = 0;
            let mut cr: u32 = 0;
            let mut sg: u32 = 0;
            let mut cg: u32 = 0;
            let mut sb: u32 = 0;
            let mut cb: u32 = 0;
            for sy in sy0..sy1 {
                let row = src.add(sy * row_bytes);
                let yodd = sy & 1;
                for sx in sx0..sx1 {
                    let v = raw10_get8(row, sx);
                    let xodd = sx & 1;
                    if yodd == red_yodd && xodd == red_xodd {
                        sr += v;
                        cr += 1;
                    } else if yodd == blue_yodd && xodd == blue_xodd {
                        sb += v;
                        cb += 1;
                    } else {
                        sg += v;
                        cg += 1;
                    }
                }
            }
            let ar = if cr > 0 { (sr / cr) as f32 } else { 0.0 };
            let ag = if cg > 0 { (sg / cg) as f32 } else { 0.0 };
            let ab = if cb > 0 { (sb / cb) as f32 } else { 0.0 };
            let lr = ((ar - bl) * bl_scale).max(0.0);
            let lg = ((ag - bl) * bl_scale).max(0.0);
            let lb = ((ab - bl) * bl_scale).max(0.0);
            fr += lr as u64;
            fg += lg as u64;
            fb += lb as u64;
            let lr = lr * wb_r;
            let lb = lb * wb_b;
            let ir = (lr + 0.5) as i32;
            let ig = (lg + 0.5) as i32;
            let ib = (lb + 0.5) as i32;
            let ir = ir.clamp(0, 255) as usize;
            let ig = ig.clamp(0, 255) as usize;
            let ib = ib.clamp(0, 255) as usize;
            let idx = (dy * dw + dx) * 3;
            out[idx] = GAMMA_LUT[ir];
            out[idx + 1] = GAMMA_LUT[ig];
            out[idx + 2] = GAMMA_LUT[ib];
        }
    }
    if fr > 0 && fg > 0 && fb > 0 {
        let gr = (fg as f32 / fr as f32).clamp(0.5, 4.0);
        let gb = (fg as f32 / fb as f32).clamp(0.5, 4.0);
        WB_R = WB_R * 0.5 + gr * 0.5;
        WB_B = WB_B * 0.5 + gb * 0.5;
    }
}

/// Returns the current white-balance gains as `(wb_r, wb_b)` with green as the reference (1.0).
///
/// Reads the IIR-smoothed gains updated by the last `process_frame` call.
pub unsafe fn current_wb_gains() -> (f32, f32) {
    (*core::ptr::addr_of!(WB_R), *core::ptr::addr_of!(WB_B))
}
