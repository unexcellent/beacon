mod audio;
mod camera;
mod uart;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use camera::Image;
use sstv::{Encoder, Mode, Synthesizer};
use uart::UartChannel;

fn transmit_sstv(channel: &mut AudioChannel, image: Image) {
    log::info!("SSTV transmission start");

    let pixels: Vec<_> = image.collect();
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

    let uart = UartChannel::new();
    let mut camera = unsafe { camera::Camera::new(camera::SC850SL, camera::MIPI) }.unwrap();
    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S).unwrap();

    unsafe { camera.calibrate(3) };

    let image = unsafe { camera.capture() };

    uart.send_image(image.clone());

    channel.enable().unwrap();
    transmit_sstv(&mut channel, image);
    channel.disable();

    log::info!("Done. Sleeping.");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
