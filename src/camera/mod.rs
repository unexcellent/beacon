//! Camera abstraction: the application-facing [`Camera`] lifecycle and the two
//! concrete cameras that implement it.
//!
//! [`RgbCamera`] (SC850SL over CSI) and [`ThermalCamera`] (MI48Dx over SPI) each
//! compose a chip driver from [`sensors`] with a frame transport from [`esp`]
//! and a pixel pipeline. The drivers are register-level and generic over an
//! `embedded-hal` bus; the transports are ESP32-P4 platform code. Only [`Camera`]
//! is a trait — the pieces are concrete structs, not a swappable framework.

mod auto_exposure;
pub mod esp;
mod format;
mod image;
mod rgb;
pub mod sensors;
mod thermal;

pub use format::{BayerOrder, ColorCalibration, FrameFormat, PixelFormat};
pub use image::Image;
pub use rgb::RgbCamera;
pub use thermal::ThermalCamera;

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
pub fn capture_image<C: Camera>(camera: Option<&mut C>) -> Option<Image> {
    camera.map(|cam| cam.capture())
}
