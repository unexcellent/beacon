#![allow(unsafe_op_in_unsafe_fn)]

mod audio;
mod camera;

use std::io::Write;

unsafe extern "C" {
    // ESP_LINE_ENDINGS_LF = 2: pass LF bytes through unchanged (no CRLF expansion).
    // Critical for binary frame data — every 0x0A pixel byte would otherwise become
    // 0x0D 0x0A, inserting bytes and corrupting the stream on the host side.
    fn usb_serial_jtag_vfs_set_tx_line_endings(mode: i32);
}

// Magic header recognised by local/receive_frame.py
const FRAME_MAGIC: [u8; 4] = [0xFF, 0xFE, 0xFD, 0xFC];

// Emit a frame over USB-JTAG serial in the format expected by receive_frame.py:
//   [0xFF 0xFE 0xFD 0xFC] [w u16-LE] [h u16-LE] [byte_len u32-LE] [RGB888 pixels]
unsafe fn send_frame(data: &[u8], w: usize, h: usize) {
    let byte_len = (w * h * 3) as u32;
    let mut header = [0u8; 12];
    header[0..4].copy_from_slice(&FRAME_MAGIC);
    header[4..6].copy_from_slice(&(w as u16).to_le_bytes());
    header[6..8].copy_from_slice(&(h as u16).to_le_bytes());
    header[8..12].copy_from_slice(&byte_len.to_le_bytes());

    let mut out = std::io::stdout();
    let _ = out.write_all(&header);
    let _ = out.write_all(data);
    let _ = out.flush();
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("MIPI-CSI frame capture starting");
    unsafe { camera_capture_loop() }
}

unsafe fn camera_capture_loop() -> ! {
    // ESP_LINE_ENDINGS_LF = 2: disable the VFS LF→CRLF expansion so pixel bytes
    // (including any 0x0A values) pass through to the host without insertion.
    usb_serial_jtag_vfs_set_tx_line_endings(2);

    let mut camera =
        camera::CameraChannel::new(camera::SC850SL, camera::BEACON_INTERFACE).unwrap();
    let (out_w, out_h) = camera.output_size();

    // Skip the first AWB_WARMUP_FRAMES so the IIR AWB can settle before
    // any frame is transmitted; the seeded WB_R/WB_B values mean only a handful
    // of frames are needed to reach a stable colour balance.
    const AWB_WARMUP_FRAMES: u32 = 5;
    let mut frame_count: u32 = 0;

    loop {
        let t0 = std::time::Instant::now();
        let rgb = camera.capture_rgb888();
        let t_isp = t0.elapsed();

        frame_count += 1;
        if frame_count <= AWB_WARMUP_FRAMES {
            let (wb_r, wb_b) = camera::current_wb_gains();
            log::info!(
                "awb warmup {}/{}: wb_r={:.2} wb_b={:.2}",
                frame_count,
                AWB_WARMUP_FRAMES,
                wb_r,
                wb_b,
            );
            continue;
        }

        send_frame(rgb, out_w, out_h);

        let (wb_r, wb_b) = camera::current_wb_gains();
        log::info!(
            "frame sent: isp={}ms total={}ms wb_r={:.2} wb_b={:.2}",
            t_isp.as_millis(),
            t0.elapsed().as_millis(),
            wb_r,
            wb_b,
        );

        loop {
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
    }
}
