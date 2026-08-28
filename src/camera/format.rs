/// Color-filter-array layout of a Bayer sensor, named by the colors of the
/// top-left 2×2 quad in row-major order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BayerOrder {
    Rggb,
    Bggr,
    Grbg,
    Gbrg,
}

/// One color site of a Bayer mosaic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BayerChannel {
    Red,
    Green,
    Blue,
}

impl BayerOrder {
    /// The color at a pixel given its row/column parity (0 or 1 each).
    pub(crate) fn channel_at(self, row_parity: usize, col_parity: usize) -> BayerChannel {
        use BayerChannel::*;
        match (self, row_parity & 1, col_parity & 1) {
            (BayerOrder::Rggb, 0, 0) | (BayerOrder::Bggr, 1, 1) => Red,
            (BayerOrder::Rggb, 1, 1) | (BayerOrder::Bggr, 0, 0) => Blue,
            (BayerOrder::Grbg, 0, 1) | (BayerOrder::Gbrg, 1, 0) => Red,
            (BayerOrder::Grbg, 1, 0) | (BayerOrder::Gbrg, 0, 1) => Blue,
            _ => Green,
        }
    }

    /// Column parity of the green sites on even rows (every order has green on
    /// exactly one column parity there) — used for green-only luma sampling.
    pub(crate) fn green_col_parity_on_even_rows(self) -> usize {
        match self {
            BayerOrder::Rggb | BayerOrder::Bggr => 1,
            BayerOrder::Grbg | BayerOrder::Gbrg => 0,
        }
    }
}

/// Pixel layout of the raw frames a sensor emits on its data interface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PixelFormat {
    /// 8-bit raw Bayer mosaic, one byte per pixel.
    Raw8(BayerOrder),
    /// Packed 10-bit raw Bayer mosaic: 4 pixels per 5-byte group, with each
    /// pixel's 8 MSBs in bytes 0–3 and the four 2-bit LSB pairs in byte 4.
    Raw10(BayerOrder),
    /// 16-bit grayscale samples, most significant byte first on the wire
    /// (e.g. thermal temperature counts).
    Gray16,
}

/// What a sensor puts on the wire: the pairing contract between a
/// [`CameraSensor`](super::sensor::CameraSensor) (which declares it) and a
/// [`CameraInterface`](super::interface::CameraInterface) (which is configured
/// with it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameFormat {
    /// Width of the frame in pixels.
    pub width: usize,
    /// Height of the frame in pixels.
    pub height: usize,
    /// Layout of the pixel data.
    pub pixel: PixelFormat,
}

impl FrameFormat {
    /// Bytes per row of raw frame data.
    pub fn row_bytes(&self) -> usize {
        match self.pixel {
            PixelFormat::Raw8(_) => self.width,
            PixelFormat::Raw10(_) => self.width * 10 / 8,
            PixelFormat::Gray16 => self.width * 2,
        }
    }

    /// Total bytes in one raw frame.
    pub fn bytes_per_frame(&self) -> usize {
        self.row_bytes() * self.height
    }
}
