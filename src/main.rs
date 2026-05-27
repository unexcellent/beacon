mod audio;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};

// Decoded at compile time by build.rs: 320 × 240 pixels, 3 bytes each.
static PATCH_RGB: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/patch_rgb.bin"));

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("SSTV beacon starting — Robot36 / 16 kHz / PCM5102A");

    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S).unwrap();

    loop {
        channel.enable().unwrap();
        transmit(&mut channel);
        channel.disable();
        log::info!("Waiting 30 s before next transmission");
        std::thread::sleep(std::time::Duration::from_secs(30));
    }
}

fn transmit(channel: &mut AudioChannel) {
    log::info!("Transmission start");

    let pixels = PATCH_RGB
        .chunks_exact(3)
        .map(|p| RgbPixel::new(p[0], p[1], p[2]));

    let encoder = Encoder::new(Mode::Robot36, pixels).unwrap();

    // Stereo interleaved 16-bit: [L_lo, L_hi, R_lo, R_hi] per frame.
    let mut buf = [0u8; PHILLIPS_I2S.chunk_size * 4];
    let mut buf_pos = 0usize;

    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        let [lo, hi] = sample.to_le_bytes();

        let (audio_off, silent_off) = if PCM5102A.left_channel { (0, 2) } else { (2, 0) };
        buf[buf_pos + audio_off] = lo;
        buf[buf_pos + audio_off + 1] = hi;
        buf[buf_pos + silent_off] = 0;
        buf[buf_pos + silent_off + 1] = 0;
        buf_pos += 4;

        if buf_pos == buf.len() {
            channel.transmit(&buf).unwrap();
            buf_pos = 0;
        }
    }

    if buf_pos > 0 {
        channel.transmit(&buf[..buf_pos]).unwrap();
    }

    log::info!("Transmission complete (~36 s Robot36 frame)");
}
