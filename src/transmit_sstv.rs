use crate::board::PHILLIPS_I2S;
use crate::error::{Error, Result};
use beacon::audio::AudioChannel;
use beacon::camera::{Camera, Image, capture_image};
use esp_idf_hal::delay;
use sstv::{Encoder, Mode, Synthesizer};

pub fn transmit_sstv(
    rgb: Option<&mut impl Camera>,
    thermal: Option<&mut impl Camera>,
    audio: &mut AudioChannel,
) -> Result<()> {
    let rgb_image = capture_image(rgb);
    let thermal_image = capture_image(thermal);

    if let Some(rgb_image) = rgb_image {
        transmit_image(rgb_image, audio)?;
        sleep(5);
    }

    if let Some(thermal_image) = thermal_image {
        transmit_image(thermal_image, audio)?;
    }

    Ok(())
}

fn sleep(seconds: u32) {
    log::info!("Waiting {} seconds...", seconds);
    delay::FreeRtos::delay_ms(seconds * 1_000);
}

fn transmit_image(image: Image, audio: &mut AudioChannel) -> Result<()> {
    log::info!("SSTV: encoding and transmitting...");

    let encoder = Encoder::new(Mode::Robot36, image).map_err(|_| Error::EmptyImage)?;
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        audio.transmit(sample)?;
    }
    audio.flush()?;

    log::info!("SSTV: transmission complete");
    Ok(())
}
