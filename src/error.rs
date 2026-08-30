use core::fmt;

use crate::link::{CommandLink, Message};

/// Crate-wide error type wrapping the errors of the underlying drivers.
///
/// The derived Debug output (variant name plus any fields) doubles as the
/// ground-report wire format of [`Message::Error`] — renaming a variant is a
/// protocol change, ground tooling may parse these names.
#[derive(Clone, Debug)]
// The tuple fields are only ever read through the derived Debug impl (which the
// dead_code lint does not count as a use), but they are downlinked in every
// ground report.
#[allow(dead_code)]
pub enum Error {
    /// Raised if the audio channel failed to initialize.
    AudioInit,
    /// Raised when writing samples to the I2S channel fails.
    AudioTransmission,
    /// Raised when an error with creating the libcsp node occurred.
    CspInit,
    /// Raised when a camera returned an empty image.
    EmptyImage,
    /// Raised when an error with the ESP32 hardware occurred.
    Peripheral,
    /// Raised if the RGB camera failed to initialize.
    RgbInit,
    /// Raised if the thermal camera failed to initialize.
    ThermalInit,
    /// Raised when an error with the UART interface occurs.
    UartAllocation,
    /// Raised when reading from the UART fails.
    UartReceive,
    /// Raised if the update session could not be started.
    UpdateBegin,
    /// Raised if a mid-transfer chunk is shorter than the announced chunk size.
    /// Fields: chunk offset, received length, expected length.
    UpdateChunkIncomplete(u32, u32, u32),
    /// Raised if the fully received image fails validation or could not be
    /// activated as the boot partition.
    UpdateCorrupt,
    /// Raised if the update ends before all bytes arrived.
    /// Fields: received bytes, expected total bytes.
    UpdateIncomplete(u32, u32),
    /// Raised when an update data package is received but no update is in progress.
    UpdateNotInProgress,
    /// Raised if no update partition could be created.
    UpdatePartition,
    /// Raised if the offset of an update package is not as expected.
    /// Fields: expected offset (bytes flashed so far), received offset.
    UpdatePackageOffset(u32, u32),
    /// Raised if writing a chunk to flash fails.
    /// Fields: chunk offset, raw esp-idf error code.
    UpdateWrite(u32, i32),
}

impl From<crate::audio::AudioError> for Error {
    fn from(e: crate::audio::AudioError) -> Self {
        match e {
            crate::audio::AudioError::Init => Error::AudioInit,
            crate::audio::AudioError::Transmission => Error::AudioTransmission,
        }
    }
}

/// Forwards to the derived [`Debug`](fmt::Debug), so that logs, `expect`
/// panics and the ground report all show the same variant-name format.
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

/// Result with the crate-wide [`Error`].
pub type Result<T> = core::result::Result<T, Error>;

/// Extension trait for downlinking failures via the payload link.
pub trait ReportIfErr<T> {
    /// Log the error and send it via the payload link before passing it on.
    fn report_if_err<L: CommandLink>(self, link: &L) -> Result<T>;
}

impl<T> ReportIfErr<T> for Result<T> {
    fn report_if_err<L: CommandLink>(self, link: &L) -> Result<T> {
        if let Err(e) = &self {
            log::error!("{e}");
            link.send(Message::Error(e.clone()));
        }
        self
    }
}
