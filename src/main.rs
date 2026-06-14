mod audio;
mod camera;
mod debug;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use debug::DebugChannel;
use sstv::{Encoder, Mode, Synthesizer};

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
    let mut camera = unsafe { camera::Camera::new(camera::SC850SL, camera::MIPI) }?;
    let mut channel = AudioChannel::new(PCM5102A, PHILLIPS_I2S)?;

    unsafe { camera.calibrate(3) };

    let image = unsafe { camera.capture() };

    debug.send_image(image.clone())?;

    channel.enable()?;

    debug.log("Starting SSTV transmission...");
    let encoder = Encoder::new(Mode::Robot36, image)?;
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        channel.transmit(sample)?;
    }
    debug.log("SSTV transmission complete.");

    channel.disable();

    Ok(())
}

fn sleep_forever(debug: &DebugChannel) -> ! {
    debug.log("Going to sleep. Good night!");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(u64::MAX));
    }
}
