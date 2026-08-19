use core::fmt;

/// Crate-wide error type wrapping the errors of the underlying drivers.
#[derive(Clone)]
pub enum Error {
    /// Raised when an error with creating the libcsp node occurred.
    CspInit,
    /// Raised when an error with the ESP32 hardware occurred.
    Peripheral,
    /// Raised if the RGB camera failed to initilize.
    RgbInit,
    /// Raised when an error with the UART interface occurs.
    UartAllocation,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CspInit => write!(f, "csp initialization failed"),
            Self::Peripheral => write!(f, "peripheral allocation failed"),
            Self::RgbInit => write!(f, "rgb camera initialization failed"),
            Self::UartAllocation => write!(f, "UART initialization failed"),
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
