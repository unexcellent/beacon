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
    /// Raised when a firmware update fails or is aborted.
    Update,
    /// Raised when an update data package is received but no update is in progress.
    UpdateNotInProgress,
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
            Self::Update => write!(f, "firmware update failed"),
            Self::UpdateNotInProgress => {
                write!(f, "update chunk received but no update in progress")
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
