use core::ffi::c_void;
use std::io::Write;

use esp_idf_sys::{
    usb_serial_jtag_driver_config_t, usb_serial_jtag_driver_install, usb_serial_jtag_read_bytes,
};
use sstv::RgbPixel;

unsafe extern "C" {
    fn usb_serial_jtag_vfs_set_tx_line_endings(mode: i32);
}

// Magic header recognised by local/capture_frame_via_usb_c.sh
const FRAME_MAGIC: [u8; 4] = [0xFF, 0xFE, 0xFD, 0xFC];

/// ASCII token the host sends over the USB-C serial link to request a capture.
const TRIGGER: &[u8] = b"CAPTURE";

pub struct DebugChannel {
    /// Rolling buffer of bytes read from USB-C stdin, scanned for `TRIGGER`.
    rx: Vec<u8>,
}

impl DebugChannel {
    pub fn new() -> Self {
        unsafe {
            // ESP_LINE_ENDINGS_LF = 2: pass LF bytes through unchanged (no CRLF expansion).
            // Critical for binary frame data — every 0x0A pixel byte would otherwise become
            // 0x0D 0x0A, inserting bytes and corrupting the stream on the host side.
            usb_serial_jtag_vfs_set_tx_line_endings(2);
            // Install the interrupt-driven USB-Serial-JTAG driver so host input can be read
            // reliably: the default console VFS does not deliver RX bytes to read(stdin).
            // TX (logs + frames) keeps using the std stdout path; we only read via the
            // driver's RX ring buffer in poll_trigger().
            let mut cfg = usb_serial_jtag_driver_config_t {
                tx_buffer_size: 1024,
                rx_buffer_size: 1024,
            };
            let _ = usb_serial_jtag_driver_install(&mut cfg);
        }
        Self { rx: Vec::new() }
    }

    /// Drain whatever is waiting on USB-C stdin and report whether the capture
    /// trigger token has arrived since the last call. Non-blocking.
    pub fn poll_trigger(&mut self) -> bool {
        let mut tmp = [0u8; 64];
        loop {
            // ticks_to_wait = 0 → non-blocking; returns the number of bytes drained.
            let n = unsafe {
                usb_serial_jtag_read_bytes(tmp.as_mut_ptr() as *mut c_void, tmp.len() as u32, 0)
            };
            if n <= 0 {
                break; // nothing waiting in the RX ring buffer
            }
            self.rx.extend_from_slice(&tmp[..n as usize]);
        }
        // Bound the buffer so stray console input can't grow it without limit.
        if self.rx.len() > 256 {
            let excess = self.rx.len() - 256;
            self.rx.drain(..excess);
        }
        match self.rx.windows(TRIGGER.len()).position(|w| w == TRIGGER) {
            Some(pos) => {
                self.rx.drain(..pos + TRIGGER.len());
                true
            }
            None => false,
        }
    }

    /// Emit a named frame over USB-C serial for local/capture_frame_via_usb_c.sh:
    ///   [0xFF 0xFE 0xFD 0xFC][name_len u8][name][w u16-LE][h u16-LE][byte_len u32-LE][RGB888]
    /// `name` identifies the source camera (e.g. "rgb", "thermal") so the host can
    /// name the saved file. Generic over the pixel source so both the RGB camera and
    /// the (false-coloured) thermal camera can be dumped over the same link.
    pub fn send_image<I>(
        &self,
        name: &str,
        width: usize,
        height: usize,
        pixels: I,
    ) -> std::io::Result<()>
    where
        I: Iterator<Item = RgbPixel>,
    {
        let name = name.as_bytes();
        let byte_len = (width * height * 3) as u32;

        let mut header = Vec::with_capacity(13 + name.len());
        header.extend_from_slice(&FRAME_MAGIC);
        header.push(name.len() as u8);
        header.extend_from_slice(name);
        header.extend_from_slice(&(width as u16).to_le_bytes());
        header.extend_from_slice(&(height as u16).to_le_bytes());
        header.extend_from_slice(&byte_len.to_le_bytes());

        let data: Vec<u8> = pixels
            .flat_map(|p| [p.red(), p.green(), p.blue()])
            .collect();

        let mut out = std::io::stdout();
        out.write_all(&header)?;
        out.write_all(&data)?;
        out.flush()
    }
}
