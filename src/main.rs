mod audio;
mod camera;
mod debug;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use camera::{Camera, MIPI, SC850SL};
use debug::DebugChannel;
use sstv::{Encoder, Mode, Synthesizer};

use crate::camera::Image;

fn main() -> ! {
    init_esp32();
    let debug = DebugChannel::new();

    match run(&debug) {
        Ok(_) => (),
        Err(e) => debug.error(&format!("Fatal error: {e}")),
    };

    sleep_forever(&debug);
}

fn init_esp32() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
}

fn run(debug: &DebugChannel) -> Result<(), Box<dyn std::error::Error>> {
    let mut camera = Camera::new(SC850SL, MIPI)?;
    let mut audio = AudioChannel::new(PCM5102A, PHILLIPS_I2S)?;

    let image = camera.capture();

    debug.send_image(image.clone())?;
    transmit_sstv(image, &mut audio, debug)?;

    Ok(())
}

fn transmit_sstv(
    image: Image,
    audio: &mut AudioChannel,
    debug: &DebugChannel,
) -> Result<(), Box<dyn std::error::Error>> {
    debug.log("Starting SSTV transmission...");
    let encoder = Encoder::new(Mode::Robot36, image)?;
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        audio.transmit(sample)?;
    }
    debug.log("SSTV transmission complete.");

    Ok(())
}

fn sleep_forever(debug: &DebugChannel) -> ! {
    debug.log("Going to sleep. Good night.");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
