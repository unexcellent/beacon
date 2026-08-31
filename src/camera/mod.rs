//! Camera abstraction: the application-facing [`Camera`] lifecycle and the two
//! cameras that implement it.
//!
//! Each camera is a sensor driver ([`Sc850sl`](sensors::Sc850sl) for RGB,
//! [`Mi48`](sensors::Mi48) for thermal) that owns a [`CameraInterface`] and a
//! pixel pipeline. The drivers carry no platform code — all ESP32-P4 logic (the
//! I2C bus and the CSI/SPI frame transport) lives behind the interface, in
//! [`esp`]. `Camera` is the outward lifecycle; `CameraInterface` is the inward
//! platform boundary the sensors depend on.

mod auto_exposure;
#[cfg(target_os = "espidf")]
pub mod esp;
mod format;
mod image;
pub mod interface;
pub mod sensors;

/// MOVE-IIIa carrier bring-up.
#[cfg(target_os = "espidf")]
#[path = "move-iiia.rs"]
pub mod move_iiia;

pub use format::{BayerOrder, ColorCalibration, FrameFormat, PixelFormat};
pub use image::Image;
pub use interface::CameraInterface;

/// Error raised while bringing up or operating a camera.
#[derive(Clone, Copy, Debug)]
pub enum CameraError {
    /// Register access over the control bus failed.
    Bus,
    /// The frame transport (CSI/ISP/SPI/DMA) could not be set up.
    Transport,
    /// The sensor's [`FrameFormat`] is not supported by this pipeline or transport.
    UnsupportedFormat,
}

/// A camera that can be powered, settled, and read as an RGB [`Image`].
///
/// Individual cameras have very different transports and pipelines, but users
/// only ever need this lifecycle: power it on, let it settle, capture, power
/// it off. Each camera keeps its own constructor for hardware bring-up (their
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
pub fn capture_image<C: Camera + ?Sized>(camera: Option<&mut C>) -> Option<Image> {
    camera.map(|cam| cam.capture())
}
