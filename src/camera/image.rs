use sstv::RgbPixel;

/// A ready-to-encode image: a row-major grid of RGB pixels, produced by any
/// [`Camera`](super::Camera).
///
/// Each camera builds its own pixels (RGB demosaic vs. grayscale thermal) at
/// its configured output resolution and hands them here via
/// [`Image::from_pixels`].
pub struct Image {
    pixels: Vec<RgbPixel>,
    index: usize,
    width: usize,
    height: usize,
}

impl Image {
    /// Wrap an already-built, row-major `width`x`height` pixel buffer.
    pub fn from_pixels(width: usize, height: usize, pixels: Vec<RgbPixel>) -> Self {
        debug_assert_eq!(pixels.len(), width * height, "pixel count must match dimensions");
        Self { pixels, index: 0, width, height }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Mutable access to the row-major pixel grid, e.g. for overlays.
    pub fn pixels_mut(&mut self) -> &mut [RgbPixel] {
        &mut self.pixels
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
