mod audio;
mod camera;
mod uart;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use sstv::{Encoder, Mode, Synthesizer};
use uart::UartChannel;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let uart = UartChannel::new();
    let mut camera = unsafe { camera::Camera::new(camera::SC850SL, camera::MIPI) }.unwrap();
    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S).unwrap();

    unsafe { camera.calibrate(3) };

    let image = unsafe { camera.capture() };

    uart.send_image(image.clone());

    channel.enable().unwrap();

    log::info!("SSTV transmission start");
    let encoder = Encoder::new(Mode::Robot36, image).unwrap();
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        channel.transmit(sample).unwrap();
    }

    log::info!("SSTV transmission complete (~36 s Robot36 frame)");

    channel.disable();

    log::info!("Done. Sleeping.");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
