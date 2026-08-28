#![allow(unsafe_op_in_unsafe_fn)]

mod image;
mod interface;
mod rgb;
mod sensor;
mod thermal;
pub(crate) mod watermark;

pub use image::Image;
pub use interface::{CameraInterface, MIPI};
pub use rgb::RgbCamera;
pub use sensor::{CameraSensor, SC850SL};
pub use thermal::ThermalCamera;

/// Robot36 output grid, shared by both cameras and the watermark overlay.
pub(crate) const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
pub(crate) const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;

/// A camera the beacon can power, settle, and read as a Robot36-sized RGB [`Image`].
///
/// The RGB and thermal cameras have very different transports and pipelines, but the
/// firmware only ever needs this lifecycle: power it on, let it settle, capture, power
/// it off. Each camera keeps its own `new(...)` for hardware bring-up (their
/// construction arguments differ); this trait covers everything after that.
pub trait Camera {
    /// Bring the sensor out of standby and prepare it to capture.
    fn power_on(&mut self);

    /// Return the sensor to a low-power idle state.
    fn power_off(&mut self);

    /// Capture and discard warm-up frames so the sensor / on-chip filters settle.
    /// Each implementation knows how many frames it needs.
    fn calibrate(&mut self);

    /// Receive a single frame as a ready-to-encode image.
    fn receive_frame(&mut self) -> Image;

    /// Capture a frame from turned off state and turn the camera back off.
    fn capture(&mut self) -> Image {
        self.power_on();
        self.calibrate();
        let image = self.receive_frame();
        self.power_off();
        image
    }
}

/// Capture one frame, or None if the camera is unavailable (failed to initialize).
pub fn capture_image<C: Camera>(camera: Option<&mut C>) -> Option<Image> {
    camera.map(|cam| cam.capture())
}
