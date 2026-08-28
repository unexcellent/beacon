//! Control-plane traits: chip drivers over their register bus.
//!
//! A sensor driver owns its bus (the standard `embedded-hal` driver pattern:
//! `Sc850sl<I2C>` is constructed with an [`I2c`](embedded_hal::i2c::I2c)
//! implementation and its board-strapped address), so these traits stay
//! bus-agnostic. Board-level facts — pins, strap-selected addresses, bus
//! clocking — live where the bus is constructed.

use super::format::FrameFormat;

/// A camera sensor: register protocol, capture lifecycle, and what it emits.
pub trait CameraSensor {
    type Error: core::fmt::Debug;

    /// The raw frame format this sensor produces on its data interface.
    fn format(&self) -> FrameFormat;

    /// Write the sensor's register init sequence. Called once at bring-up,
    /// with the sensor out of reset and not yet streaming.
    fn init(&mut self) -> Result<(), Self::Error>;

    /// Start producing frames: enable streaming, or arm a single-shot chip.
    fn start(&mut self) -> Result<(), Self::Error>;

    /// Stop producing frames and enter a low-power idle state.
    fn stop(&mut self) -> Result<(), Self::Error>;
}

/// Manual exposure and analog-gain control, for sensors that support it.
pub trait ExposureControl: CameraSensor {
    /// Inclusive (min, max) integration time the sensor accepts, in lines.
    fn exposure_range(&self) -> (u32, u32);

    /// Integration time in lines after [`init`](CameraSensor::init)
    /// (starting point for auto-exposure).
    fn default_exposure(&self) -> u32;

    /// Largest usable analog gain as a linear multiplier.
    fn max_gain(&self) -> f32;

    /// Set the integration time in lines. Implementations clamp to
    /// [`exposure_range`](Self::exposure_range).
    fn set_exposure(&mut self, lines: u32) -> Result<(), Self::Error>;

    /// Set the analog gain as a linear multiplier (1.0 = unity).
    fn set_analog_gain(&mut self, gain: f32) -> Result<(), Self::Error>;
}

/// Color-processing seeds for a Bayer sensor, consumed by the demosaic pipeline.
pub struct ColorCalibration {
    /// The sensor's black-level pedestal in 8-bit space.
    pub black_level: u8,
    /// Initial white-balance red gain (green = 1.0 reference).
    pub red_gain: f32,
    /// Initial white-balance blue gain (green = 1.0 reference).
    pub blue_gain: f32,
}

/// A Bayer color sensor whose raw frames are demosaiced with these calibration seeds.
pub trait BayerSensor: CameraSensor {
    fn color_calibration(&self) -> ColorCalibration;
}

/// Single-shot sensors: frames are triggered and polled per capture rather
/// than streamed. [`start`](CameraSensor::start) arms the chip (often a no-op);
/// each frame is then requested with [`trigger`](Self::trigger) and fetched
/// from the data interface once [`frame_ready`](Self::frame_ready) reports true.
pub trait FrameTrigger: CameraSensor {
    /// Request capture of exactly one frame.
    fn trigger(&mut self) -> Result<(), Self::Error>;

    /// Whether a triggered frame is ready to be read from the data interface.
    fn frame_ready(&mut self) -> Result<bool, Self::Error>;
}
