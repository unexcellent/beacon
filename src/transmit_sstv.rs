//! SSTV downlink: capture each camera's current frame, encode it as Robot36
//! SSTV and play it out over an [`AudioChannel`].

use crate::audio::AudioChannel;
use crate::camera::{Camera, Image, capture_image};
use crate::error::{Error, Result};
use sstv::modes::ROBOT_36;
use sstv::{Encoder, Synthesizer};

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
    #[cfg(target_os = "espidf")]
    esp_idf_hal::delay::FreeRtos::delay_ms(seconds * 1_000);
}

/// Robot36-encode one image and stream its samples through the audio channel.
fn transmit_image(image: Image, audio: &mut impl AudioChannel) -> Result<()> {
    log::info!("SSTV: encoding and transmitting...");

    let encoder = Encoder::new(ROBOT_36, image).map_err(|_| Error::EmptyImage)?;
    for sample in Synthesizer::new(encoder, audio.sample_rate()) {
        audio.transmit(sample)?;
    }
    audio.flush()?;

    log::info!("SSTV: transmission complete");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::transmit_sstv;
    use crate::test_support::{FakeAudio, FakeCamera, missing_camera, working_camera};

    #[test]
    fn single_camera_captures_then_encodes_and_flushes_once() {
        let (camera, log) = FakeCamera::boxed();
        let mut audio = FakeAudio::new();

        transmit_sstv(&mut vec![Some(camera)], &mut audio).expect("transmission");

        assert_eq!(
            log.calls(), // contains all method calls of FakeCamera
            ["power_on", "calibrate", "receive_frame", "power_off"]
        );
        assert_eq!(audio.completed_transmissions(), 1);
        assert!(audio.samples > 0);
    }

    #[test]
    fn empty_slots_are_skipped() {
        let mut cameras = vec![missing_camera(), working_camera(), missing_camera()];
        let mut audio = FakeAudio::new();

        transmit_sstv(&mut cameras, &mut audio).expect("transmission");

        assert_eq!(
            audio.completed_transmissions(),
            1,
            "only the working camera transmits"
        );
    }

    #[test]
    fn every_present_camera_transmits() {
        let mut cameras = vec![working_camera(), working_camera()];
        let mut audio = FakeAudio::new();

        transmit_sstv(&mut cameras, &mut audio).expect("transmission");

        assert_eq!(
            audio.completed_transmissions(),
            2,
            "one image per working camera"
        );
    }

    #[test]
    fn no_cameras_transmit_nothing() {
        let mut audio = FakeAudio::new();

        transmit_sstv(&mut [], &mut audio).expect("no-op");

        assert_eq!(audio.samples, 0);
        assert_eq!(audio.completed_transmissions(), 0);
    }

    #[test]
    fn audio_error_propagates_and_skips_flush() {
        let mut cameras = vec![working_camera()];
        let mut audio = FakeAudio::failing_after(100);

        let result = transmit_sstv(&mut cameras, &mut audio);

        assert!(result.is_err());
        assert_eq!(
            audio.completed_transmissions(),
            0,
            "a failed transmit must not complete"
        );
    }
}
