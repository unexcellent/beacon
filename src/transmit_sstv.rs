use crate::devices::audio::{AudioChannel, PHILLIPS_I2S};
use crate::devices::camera::{Camera, Image, RgbCamera, ThermalCamera};
use crate::devices::link::{Message, PayloadLink};
use crate::error::{Error, Result};
use esp_idf_hal::delay;
use sstv::{Encoder, Mode, Synthesizer};

pub fn transmit_sstv(
    link: &mut PayloadLink,
    rgb: Option<&mut RgbCamera>,
    thermal: Option<&mut ThermalCamera>,
    audio: Option<&mut AudioChannel>,
) -> Result<()> {
    link.send(Message::Busy);

    let audio = audio.ok_or(Error::AudioInit)?;
    if rgb.is_none() && thermal.is_none() {
        return Err(Error::AllCamerasInit);
    }

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

fn capture_image<C: Camera>(camera: Option<&mut C>) -> Option<Image> {
    camera.map(|cam| cam.capture())
}

fn sleep(seconds: u32) {
    log::info!("Sleeping {}s...", seconds);
    delay::FreeRtos::delay_ms(seconds * 1_000);
}

fn transmit_image(image: Image, audio: &mut AudioChannel) -> Result<()> {
    log::info!("SSTV: encoding and transmitting...");
    let encoder = Encoder::new(Mode::Robot36, image).unwrap();
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        audio.transmit(sample).map_err(|_| Error::Peripheral)?;
    }
    audio.flush().map_err(|_| Error::Peripheral)?;
    log::info!("SSTV: transmission complete");
    Ok(())
}
