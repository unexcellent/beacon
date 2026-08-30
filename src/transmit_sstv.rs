//! SSTV downlink: capture each camera's current frame, encode it as Robot36
//! SSTV and play it out over an [`AudioChannel`].

use crate::audio::AudioChannel;
use crate::camera::{Camera, Image, capture_image};
use crate::error::{Error, Result};
use esp_idf_hal::delay;
use sstv::{Encoder, Mode, Synthesizer};

/// Capture and transmit one SSTV image for each working camera.
///
/// Every camera is captured up front — before the first (multi-second)
/// transmission begins — so all frames show the same moment. Empty slots (a
/// camera that failed to initialize) are skipped. The captured images are then
/// encoded and played out in order, spaced apart, with no pause after the last.
///
/// Returns the first transmission error, if any; captures themselves never fail
/// (an unavailable camera is simply absent from the output).
pub fn transmit_sstv(
    cameras: &mut [Option<Box<dyn Camera>>],
    audio: &mut impl AudioChannel,
) -> Result<()> {
    let images = capture_images(cameras);

    for (i, image) in images.into_iter().enumerate() {
        let is_not_first_image = i > 0;
        if is_not_first_image {
            wait(5);
        }
        transmit_image(image, audio)?;
    }

    Ok(())
}

/// Capture the current frame from every present camera, dropping empty slots.
fn capture_images(cameras: &mut [Option<Box<dyn Camera>>]) -> Vec<Image> {
    cameras
        .iter_mut()
        .filter_map(|slot| capture_image(slot.as_deref_mut()))
        .collect()
}

/// Block the current task for `seconds`, spacing consecutive transmissions.
fn wait(seconds: u32) {
    log::info!("Waiting {} seconds...", seconds);
    delay::FreeRtos::delay_ms(seconds * 1_000);
}

/// Robot36-encode one image and stream its samples through the audio channel.
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
