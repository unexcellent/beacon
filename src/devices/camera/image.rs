use sstv::RgbPixel;

/// A ready-to-encode image: a row-major grid of RGB pixels, produced by any
/// [`Camera`](super::Camera) and consumed by the SSTV encoder.
///
/// Both cameras emit the same Robot36 grid, so this is the single image type the
/// rest of the firmware sees; each camera builds its own pixels (RGB demosaic vs.
/// false-coloured thermal) and hands them here via [`Image::from_pixels`].
pub struct Image {
    pixels: Vec<RgbPixel>,
    index: usize,
    width: usize,
    height: usize,
}

impl Image {
    /// Wrap an already-built, row-major `width`x`height` pixel buffer.
    pub(crate) fn from_pixels(width: usize, height: usize, pixels: Vec<RgbPixel>) -> Self {
        debug_assert_eq!(pixels.len(), width * height, "pixel count must match dimensions");
        Self { pixels, index: 0, width, height }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }
}

impl Iterator for Image {
    type Item = RgbPixel;

    fn next(&mut self) -> Option<RgbPixel> {
        let pixel = self.pixels.get(self.index).copied();
        self.index += 1;
        pixel
    }
}
