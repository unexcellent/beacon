use crate::error::{Error, Result};
use beacon::audio::AudioChannel;
use beacon::camera::{Camera, Image, capture_image};
use esp_idf_hal::delay;
use sstv::{Encoder, Mode, Synthesizer};

pub fn transmit_sstv(
    cameras: Vec<Option<&mut dyn Camera>>,
    audio: &mut impl AudioChannel,
) -> Result<()> {
    let images = capture_images_from_working_cameras(cameras);

    for (i, image) in images.into_iter().enumerate() {
        let is_not_first_image = i > 0;
        if is_not_first_image {
            wait(5);
        }
        transmit_image(image, audio)?;
    }

    Ok(())
}

fn capture_images_from_working_cameras(cameras: Vec<Option<&mut dyn Camera>>) -> Vec<Image> {
    cameras.into_iter().filter_map(capture_image).collect()
}

fn wait(seconds: u32) {
    log::info!("Waiting {} seconds...", seconds);
    delay::FreeRtos::delay_ms(seconds * 1_000);
}

fn transmit_image(image: Image, audio: &mut impl AudioChannel) -> Result<()> {
    log::info!("SSTV: encoding and transmitting...");

    let encoder = Encoder::new(Mode::Robot36, image).map_err(|_| Error::EmptyImage)?;
    for sample in Synthesizer::new(encoder, audio.sample_rate()) {
        audio.transmit(sample)?;
    }
    audio.flush()?;

    log::info!("SSTV: transmission complete");
    Ok(())
}
