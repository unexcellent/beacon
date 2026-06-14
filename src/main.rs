mod audio;
mod camera;

use std::io::Write;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};

unsafe extern "C" {
    // ESP_LINE_ENDINGS_LF = 2: pass LF bytes through unchanged (no CRLF expansion).
    // Critical for binary frame data — every 0x0A pixel byte would otherwise become
    // 0x0D 0x0A, inserting bytes and corrupting the stream on the host side.
    fn usb_serial_jtag_vfs_set_tx_line_endings(mode: i32);
}

// Magic header recognised by local/receive_frame.py
const FRAME_MAGIC: [u8; 4] = [0xFF, 0xFE, 0xFD, 0xFC];

// Emit a frame over USB-JTAG serial in the format expected by receive_frame.py:
//   [0xFF 0xFE 0xFD 0xFC] [w u16-LE] [h u16-LE] [byte_len u32-LE] [RGB888 pixels]
fn send_frame(data: &[u8], w: usize, h: usize) {
    let byte_len = (w * h * 3) as u32;
    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(&FRAME_MAGIC);
    header[4..6].copy_from_slice(&(w as u16).to_le_bytes());
    header[6..8].copy_from_slice(&(h as u16).to_le_bytes());
    header[8..12].copy_from_slice(&byte_len.to_le_bytes());

    let mut out = std::io::stdout();
    let _ = out.write_all(&header);
    let _ = out.write_all(data);
    let _ = out.flush();
}

fn transmit_sstv(channel: &mut AudioChannel, rgb: &[u8]) {
    log::info!("SSTV transmission start");

    let pixels: Vec<RgbPixel> = rgb
        .chunks_exact(3)
        .map(|p| RgbPixel::new(p[0], p[1], p[2]))
        .collect();

    let encoder = Encoder::new(Mode::Robot36, pixels.into_iter()).unwrap();

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

    log::info!("SSTV transmission complete (~36 s Robot36 frame)");
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("MIPI-CSI capture + UART + SSTV beacon starting");

    unsafe { usb_serial_jtag_vfs_set_tx_line_endings(2) };

    let mut camera = unsafe { camera::Camera::new(camera::SC850SL, camera::MIPI) }.unwrap();
    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S).unwrap();

    // 230 KB — exceeds CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL (16 KB), so Vec goes to PSRAM.
    let mut out_buf = vec![0u8; 320 * 240 * 3];

    unsafe { camera.calibrate(3) };

    for (i, pixel) in unsafe { camera.capture() }.enumerate() {
        out_buf[i * 3]     = pixel.red();
        out_buf[i * 3 + 1] = pixel.green();
        out_buf[i * 3 + 2] = pixel.blue();
    }

    send_frame(&out_buf, 320, 240);

    channel.enable().unwrap();
    transmit_sstv(&mut channel, &out_buf);
    channel.disable();

    log::info!("Done. Sleeping.");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
