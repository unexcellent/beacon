use std::sync::OnceLock;

use super::{OUTPUT_HEIGHT, OUTPUT_WIDTH};

static DATA: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/watermark.bin"));

const MARGIN: usize = 5;

struct Bounds { w: usize, h: usize, x0: usize, y0: usize }

static BOUNDS: OnceLock<Bounds> = OnceLock::new();

fn bounds() -> &'static Bounds {
    BOUNDS.get_or_init(|| {
        let w = u16::from_le_bytes([DATA[0], DATA[1]]) as usize;
        let h = u16::from_le_bytes([DATA[2], DATA[3]]) as usize;
        Bounds { w, h, x0: OUTPUT_WIDTH - w - MARGIN, y0: OUTPUT_HEIGHT - h - MARGIN }
    })
}

pub fn is_white_at(dx: usize, dy: usize) -> bool {
    let b = bounds();
    if dx < b.x0 || dx >= b.x0 + b.w || dy < b.y0 || dy >= b.y0 + b.h {
        return false;
    }
    let idx = (dy - b.y0) * b.w + (dx - b.x0);
    (DATA[4 + idx / 8] >> (7 - (idx % 8))) & 1 == 1
}
