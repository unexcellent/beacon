use core::fmt;

/// Crate-wide error type wrapping the errors of the underlying drivers.
#[derive(Debug)]
pub enum Error {
    /// Raised when an error with the ESP32 hardware occurred.
    Peripheral,
    /// SSTV encoding failed.
    Sstv(sstv::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Peripheral => write!(f, "peripheral allocation"),
            Self::Sstv(e) => write!(f, "sstv error: {e}"),
        }
    }
}

impl core::error::Error for Error {}

impl From<sstv::Error> for Error {
    fn from(e: sstv::Error) -> Self {
        Self::Sstv(e)
    }
}

/// Result with the crate-wide [`Error`].
pub type Result<T> = core::result::Result<T, Error>;
