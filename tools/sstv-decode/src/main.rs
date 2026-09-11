//! Decode a recorded Robot36 SSTV WAV into a PNG and report image statistics.
//!
//! Usage: sstv-decode <input.wav> <output.png>
//!
//! Reads a 16-bit WAV (mono or stereo — the ESP puts the signal on one I2S
//! channel, so for stereo we pick the higher-energy channel), decodes it with
//! the same `sstv` crate the firmware encodes with, saves the first decoded
//! image, and prints one line of stats for the caller to validate:
//!
//!     <width> <height> <std_r> <std_g> <std_b> <smoothness>
//!
//! where std_* are per-channel standard deviations (0..255; ~0 means a flat /
//! blank frame) and smoothness is the fraction of horizontally-adjacent pixels
//! that differ by <= 24 (high for a real image, low for noise / a bad decode).

use std::process::exit;

use sstv::{Decoder, Mode};

fn die(msg: &str) -> ! {
    eprintln!("sstv-decode: {msg}");
    exit(1);
}

fn main() {
    let mut args = std::env::args().skip(1);
    let (Some(input), Some(output)) = (args.next(), args.next()) else {
        eprintln!("usage: sstv-decode <input.wav> <output.png>");
        exit(2);
    };

    let mut reader =
        hound::WavReader::open(&input).unwrap_or_else(|e| die(&format!("open {input}: {e}")));
    let spec = reader.spec();
    let channels = spec.channels.max(1) as usize;
    let sample_rate = spec.sample_rate;
    let samples: Vec<i16> = reader.samples::<i16>().map(|s| s.unwrap_or(0)).collect();

    // Deinterleave and keep the channel carrying the signal (highest energy).
    let mono: Vec<i16> = if channels <= 1 {
        samples
    } else {
        let mut energy = vec![0i64; channels];
        for (i, &s) in samples.iter().enumerate() {
            energy[i % channels] += i64::from(s) * i64::from(s);
        }
        let best = energy
            .iter()
            .enumerate()
            .max_by_key(|(_, &e)| e)
            .map_or(0, |(c, _)| c);
        samples.into_iter().skip(best).step_by(channels).collect()
    };

    let decoder = Decoder::from_samples(Mode::Robot36, mono.into_iter(), sample_rate);
    let Some(img) = decoder.rgb_images().next() else {
        die("no SSTV image could be decoded from the recording");
    };

    img.save(&output)
        .unwrap_or_else(|e| die(&format!("save {output}: {e}")));

    let (w, h) = (img.width(), img.height());
    let stats = image_stats(&img);
    println!(
        "{w} {h} {:.1} {:.1} {:.1} {:.3}",
        stats.std[0], stats.std[1], stats.std[2], stats.smoothness
    );
}

struct Stats {
    std: [f64; 3],
    smoothness: f64,
}

fn image_stats(img: &image::RgbImage) -> Stats {
    let (w, h) = (img.width() as usize, img.height() as usize);
    let n = (w * h).max(1) as f64;

    let mut sum = [0f64; 3];
    let mut sumsq = [0f64; 3];
    for p in img.pixels() {
        for c in 0..3 {
            let v = f64::from(p.0[c]);
            sum[c] += v;
            sumsq[c] += v * v;
        }
    }
    let mut std = [0f64; 3];
    for c in 0..3 {
        let mean = sum[c] / n;
        std[c] = (sumsq[c] / n - mean * mean).max(0.0).sqrt();
    }

    // Horizontal neighbour similarity, averaged over channels.
    let mut similar = 0u64;
    let mut pairs = 0u64;
    for y in 0..h {
        for x in 1..w {
            let a = img.get_pixel(x as u32, y as u32).0;
            let b = img.get_pixel((x - 1) as u32, y as u32).0;
            for c in 0..3 {
                if a[c].abs_diff(b[c]) <= 24 {
                    similar += 1;
                }
                pairs += 1;
            }
        }
    }
    let smoothness = if pairs == 0 {
        0.0
    } else {
        similar as f64 / pairs as f64
    };

    Stats { std, smoothness }
}
