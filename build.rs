fn main() {
    embuild::espidf::sysenv::output();
    build_watermark();
}

fn build_watermark() {
    use image::GenericImageView;

    println!("cargo:rerun-if-changed=assets/watermark.png");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let src = image::open("assets/watermark.png").unwrap();

    let target_w = 320u32 / 3;
    let target_h = (src.height() as f32 * target_w as f32 / src.width() as f32).round() as u32;
    let scaled = src.resize_exact(target_w, target_h, image::imageops::FilterType::Triangle);

    let w = scaled.width() as usize;
    let h = scaled.height() as usize;

    // Pack each pixel into 1 bit (MSB first, row-major, no row padding).
    // A pixel is "white" if all RGB channels are bright, regardless of alpha.
    let total_bits = w * h;
    let mut bits = vec![0u8; (total_bits + 7) / 8];
    for y in 0..h {
        for x in 0..w {
            let p = scaled.get_pixel(x as u32, y as u32);
            if p[0] > 128 && p[1] > 128 && p[2] > 128 {
                let idx = y * w + x;
                bits[idx / 8] |= 1 << (7 - (idx % 8));
            }
        }
    }

    let mut out = Vec::with_capacity(4 + bits.len());
    out.extend_from_slice(&(w as u16).to_le_bytes());
    out.extend_from_slice(&(h as u16).to_le_bytes());
    out.extend_from_slice(&bits);

    std::fs::write(format!("{out_dir}/watermark.bin"), out).unwrap();
}
