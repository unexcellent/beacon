use std::io::Write;

use sstv::RgbPixel;

unsafe extern "C" {
    fn usb_serial_jtag_vfs_set_tx_line_endings(mode: i32);
}

// Magic header recognised by local/receive_frame.py
const FRAME_MAGIC: [u8; 4] = [0xFF, 0xFE, 0xFD, 0xFC];

pub struct DebugChannel;

impl DebugChannel {
    pub fn new() -> Self {
        // ESP_LINE_ENDINGS_LF = 2: pass LF bytes through unchanged (no CRLF expansion).
        // Critical for binary frame data — every 0x0A pixel byte would otherwise become
        // 0x0D 0x0A, inserting bytes and corrupting the stream on the host side.
        unsafe { usb_serial_jtag_vfs_set_tx_line_endings(2) };
        Self
    }

    #[allow(dead_code)] // parity with the RGB path; available for ad-hoc debug logging
    pub fn log(&self, msg: &str) {
        log::info!("{msg}");
    }

    #[allow(dead_code)] // parity with the RGB path; available for ad-hoc debug logging
    pub fn error(&self, msg: &str) {
        log::error!("{msg}");
    }

    // Emit a frame over USB-JTAG serial in the format expected by receive_frame.py:
    //   [0xFF 0xFE 0xFD 0xFC] [w u16-LE] [h u16-LE] [byte_len u32-LE] [RGB888 pixels]
    // Generic over the pixel source so both the RGB camera and the thermal camera
    // (which yield RgbPixel iterators) can be dumped over the same debug link.
    pub fn send_image<I>(&self, width: usize, height: usize, pixels: I) -> std::io::Result<()>
    where
        I: Iterator<Item = RgbPixel>,
    {
        let byte_len = (width * height * 3) as u32;

        let mut header = [0u8; 12];
        header[0..4].copy_from_slice(&FRAME_MAGIC);
        header[4..6].copy_from_slice(&(width as u16).to_le_bytes());
        header[6..8].copy_from_slice(&(height as u16).to_le_bytes());
        header[8..12].copy_from_slice(&byte_len.to_le_bytes());

        let data: Vec<u8> = pixels.flat_map(|p| [p.red(), p.green(), p.blue()]).collect();

        let mut out = std::io::stdout();
        out.write_all(&header)?;
        out.write_all(&data)?;
        out.flush()
    }
}
