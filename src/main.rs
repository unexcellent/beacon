mod audio;
mod camera;
mod uart;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use sstv::{Encoder, Mode, Synthesizer};
use uart::UartChannel;

fn main() -> ! {
    init_esp32();

    match run() {
        Ok(_) => (),
        Err(e) => log::error!("Fatal error: {e}"),
    };

    sleep_forever();
}

fn init_esp32() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let uart = UartChannel::new();
    let mut camera = unsafe { camera::Camera::new(camera::SC850SL, camera::MIPI) }?;
    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S)?;

    unsafe { camera.calibrate(3) };

    let image = unsafe { camera.capture() };

    uart.send_image(image.clone())?;

    channel.enable()?;

    log::info!("SSTV transmission start");
    let encoder = Encoder::new(Mode::Robot36, image)?;
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        channel.transmit(sample)?;
    }
    log::info!("SSTV transmission complete (~36 s Robot36 frame)");

    channel.disable();

    Ok(())
}

fn sleep_forever() -> ! {
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
