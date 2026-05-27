mod audio;

use audio::{CHUNK_SAMPLES, I2s, SAMPLE_RATE};
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};

// Decoded at compile time by build.rs: 320 × 240 pixels, 3 bytes each.
static PATCH_RGB: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/patch_rgb.bin"));

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("SSTV beacon starting — Robot36 / 16 kHz / PCM5102A");

    let mut i2s = I2s::new().unwrap();

    loop {
        i2s.enable().unwrap();
        transmit(&mut i2s);
        i2s.disable();
        log::info!("Waiting 30 s before next transmission");
        std::thread::sleep(std::time::Duration::from_secs(30));
    }
}

fn transmit(i2s: &mut I2s) {
    log::info!("Transmission start");

    let pixels = PATCH_RGB
        .chunks_exact(3)
        .map(|p| RgbPixel::new(p[0], p[1], p[2]));

    let encoder = Encoder::new(Mode::Robot36, pixels).unwrap();

    // Stereo interleaved 16-bit: [L_lo, L_hi, R_lo, R_hi] per frame.
    // Left channel is silent; audio goes on right.
    let mut buf = [0u8; CHUNK_SAMPLES * 4];
    let mut buf_pos = 0usize;

    for sample in Synthesizer::new(encoder, SAMPLE_RATE) {
        let [lo, hi] = sample.to_le_bytes();

        buf[buf_pos] = 0; // left lo  (silent)
        buf[buf_pos + 1] = 0; // left hi  (silent)
        buf[buf_pos + 2] = lo; // right lo (audio)
        buf[buf_pos + 3] = hi; // right hi (audio)
        buf_pos += 4;

        if buf_pos == buf.len() {
            i2s.write_all(&buf).unwrap();
            buf_pos = 0;
        }
    }

    if buf_pos > 0 {
        i2s.write_all(&buf[..buf_pos]).unwrap();
    }

    log::info!("Transmission complete (~36 s Robot36 frame)");
}
