//! Camera abstraction: sensors (control plane), interfaces (data plane), and
//! the capture lifecycle that composes them.
//!
//! The layering follows the split between *what chip is connected* and *how its
//! frames arrive*:
//!
//! - [`sensor`] — control-plane traits. A [`CameraSensor`](sensor::CameraSensor)
//!   is a chip driver over its register bus (`embedded-hal` I2C), with capability
//!   traits for exposure control, Bayer calibration, and single-shot triggering.
//! - [`interface`] — data-plane trait. A [`CameraInterface`](interface::CameraInterface)
//!   delivers raw frames of a negotiated [`FrameFormat`], knowing nothing about
//!   the chip that produces them.
//! - [`Camera`] — the application-facing lifecycle both camera types implement.
//!
//! [`RgbCamera`] and [`ThermalCamera`] are generic compositions of one sensor,
//! one interface, and a pixel pipeline. Platform transports live in [`esp`],
//! chip drivers in [`sensors`].

pub mod esp;
mod format;
mod image;
pub mod interface;
mod rgb;
pub mod sensor;
pub mod sensors;
mod thermal;

pub use format::{BayerOrder, FrameFormat, PixelFormat};
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
