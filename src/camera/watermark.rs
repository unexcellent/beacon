use super::{OUTPUT_HEIGHT, OUTPUT_WIDTH};

static DATA: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/watermark.bin"));

const MARGIN: usize = 5;

fn width() -> usize {
    u16::from_le_bytes([DATA[0], DATA[1]]) as usize
}

fn height() -> usize {
    u16::from_le_bytes([DATA[2], DATA[3]]) as usize
}

pub fn is_white_at(dx: usize, dy: usize) -> bool {
    let w = width();
    let h = height();
    let x0 = OUTPUT_WIDTH - w - MARGIN;
    let y0 = OUTPUT_HEIGHT - h - MARGIN;

    if dx < x0 || dx >= x0 + w || dy < y0 || dy >= y0 + h {
        return false;
    }

    let idx = (dy - y0) * w + (dx - x0);
    (DATA[4 + idx / 8] >> (7 - (idx % 8))) & 1 == 1
}
