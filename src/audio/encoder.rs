//! The serial data format a DAC expects on the wire.

/// Describes the serial data format expected by a DAC or audio codec.
///
/// These settings map directly to the I2S slot configuration and determine
/// how samples are packed on the wire.
pub struct AudioEncoder {
    /// Use 16-bit samples (`true`) or 24-bit samples (`false`).
    pub width_16bit: bool,
    /// Transmit both left and right channels (`true`) or a single mono channel (`false`).
    pub stereo: bool,
    /// Delay data by one BCLK cycle relative to WS (Philips I2S standard). Set `false` for left-justified format.
    pub bit_shift: bool,
    /// In stereo mode: which channel carries the audio signal (`true` = left, `false` = right).
    /// In mono mode: which I2S slot to use (`true` = left/WS-low, `false` = right/WS-high).
    pub left_channel: bool,
    /// Transmit the most significant byte first (`true`) or least significant byte first (`false`).
    pub big_endian: bool,
    /// Transmit the least significant bit first within each byte (`true`) or most significant bit first (`false`).
    pub least_significant_bit_first: bool,
    /// Align sample data to the left (MSB) edge of the slot (`true`) or right (LSB) edge (`false`).
    pub left_align_data: bool,
}
