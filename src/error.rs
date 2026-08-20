use core::fmt;

use crate::devices::link::{Message, PayloadLink};

/// Crate-wide error type wrapping the errors of the underlying drivers.
#[derive(Clone)]
pub enum Error {
    /// Raised if all cameras failed to initialize.
    AllCamerasInit,
    /// Raised if the audio channel failed to initilize.
    AudioInit,
    /// Raised when writing samples to the I2S channel fails.
    AudioTransmission,
    /// Raised when an error with creating the libcsp node occurred.
    CspInit,
    /// Raised when an error with the ESP32 hardware occurred.
    Peripheral,
    /// Raised if the RGB camera failed to initilize.
    RgbInit,
    /// Raised if the thermal camera failed to initilize.
    ThermalInit,
    /// Raised when an error with the UART interface occurs.
    UartAllocation,
    /// Raised if the update session could not be started.
    UpdateBegin,
    UpdateChunkIncomplete(u32, u32, u32),
    UpdateCorrupt,
    UpdateIncomplete(u32, u32),
    /// Raised when an update data package is received but no update is in progress.
    UpdateNotInProgress,
    /// Raised if no update partition could be created.
    UpdatePartition,
    /// Raised if the offset of an update package is not as expected.
    UpdatePackageOffset(u32, u32),
    UpdateWrite(u32, i32),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AllCamerasInit => write!(f, "all cameras failed to initialize"),
            Self::AudioInit => write!(f, "audio channel initialization failed"),
            Self::AudioTransmission => write!(f, "audio transmission failed"),
            Self::CspInit => write!(f, "csp initialization failed"),
            Self::Peripheral => write!(f, "peripheral allocation failed"),
            Self::RgbInit => write!(f, "rgb camera initialization failed"),
            Self::ThermalInit => write!(f, "thermal camera initialization failed"),
            Self::UartAllocation => write!(f, "UART initialization failed"),
            Self::UpdateBegin => write!(f, "update begin failed"),
            Self::UpdateCorrupt => write!(f, "update corrupt"),
            Self::UpdateChunkIncomplete(offset, is, should) => write!(
                f,
                "chunk {:#x} incomplete (is={}, should={})",
                offset, is, should
            ),
            Self::UpdateIncomplete(is, should) => {
                write!(f, "update incomplete (is={}, should={})", is, should)
            }
            Self::UpdateNotInProgress => write!(f, "update not announced or begun"),
            Self::UpdatePartition => write!(f, "update partition could not be created"),
            Self::UpdatePackageOffset(is, should) => write!(f, "is: {}, should: {}", is, should),
            Self::UpdateWrite(offset, error_code) => {
                write!(f, "at: {}, is: {}", offset, error_code)
            }
        }
    }
}

/// Forwards to [`Display`](fmt::Display) so that `expect`/`unwrap` panics show
/// the human-readable message instead of the variant structure.
impl fmt::Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// Result with the crate-wide [`Error`].
pub type Result<T> = core::result::Result<T, Error>;

/// Extension trait for downlinking failures via the payload link.
pub trait ReportIfErr<T> {
    /// Log the error and send it via the payload link before passing it on.
    fn report_if_err(self, link: &PayloadLink) -> Result<T>;
}

impl<T> ReportIfErr<T> for Result<T> {
    fn report_if_err(self, link: &PayloadLink) -> Result<T> {
        if let Err(e) = &self {
            log::error!("{e}");
            link.send(Message::Error(e.clone()));
        }
        self
    }
}
