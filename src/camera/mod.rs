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
    /// Capture and discard `frames` frames so the sensor / on-chip filters settle.
    fn calibrate(&mut self, frames: u32);
    /// Capture a single frame as a ready-to-encode image.
    fn capture(&mut self) -> Image;
}
