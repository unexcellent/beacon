#![allow(unsafe_op_in_unsafe_fn)]

mod image;
mod interface;
mod sensor;
mod watermark;

use core::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use esp_idf_sys::*;

pub use image::Image;
pub use interface::{CameraInterface, MIPI};
pub use sensor::{CameraSensor, SC850SL};

static FRAME_READY: AtomicBool = AtomicBool::new(false);

// Holds the capture buffer pointer and length for the CSI DMA callbacks.
// Heap-allocated so its address is stable after being registered as userdata.
struct CaptureBuffer {
    buf: *mut c_void,
    len: usize,
    row_bytes: usize,
}
impl CaptureBuffer {
    fn len(&self) -> usize {
        self.len
    }
    fn row_bytes(&self) -> usize {
        self.row_bytes
    }
}

unsafe extern "C" fn on_get_new_trans(
    _: esp_cam_ctlr_handle_t,
    trans: *mut esp_cam_ctlr_trans_t,
    user_ctx: *mut c_void,
) -> bool {
    let cap = &*(user_ctx as *const CaptureBuffer);
    (*trans).buffer = cap.buf;
    (*trans).buflen = cap.len;
    false
}

unsafe extern "C" fn on_trans_finished(
    _: esp_cam_ctlr_handle_t,
    _: *mut esp_cam_ctlr_trans_t,
    _: *mut c_void,
) -> bool {
    FRAME_READY.store(true, Ordering::Release);
    false
}

struct InnerCamera {
    csi: esp_cam_ctlr_handle_t,
    i2c_dev: i2c_master_dev_handle_t,
}

impl InnerCamera {
    unsafe fn queue_receive(&self, capture_buf: &CaptureBuffer) -> Result<(), EspError> {
        let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
        trans.buffer = capture_buf.buf;
        trans.buflen = capture_buf.len();
        esp!(esp_cam_ctlr_receive(self.csi, &mut trans, 100))
    }
}

const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;

pub struct Camera {
    sensor: CameraSensor,
    capture_buffer: Box<CaptureBuffer>,
    _inner: InnerCamera,
    wb_r: f32,
    wb_b: f32,
}

impl Camera {
    pub fn new(sensor: CameraSensor, interface: CameraInterface) -> Result<Self, EspError> {
        unsafe {
            let capture_buffer = Self::allocate_capture_buffer(&sensor);

            let inner = interface.init(&sensor, &capture_buffer)?;
            inner.queue_receive(&capture_buffer)?;

            sensor.enable(inner.i2c_dev);

            let mut cam = Self {
                wb_r: sensor.red_gain_seed,
                wb_b: sensor.blue_gain_seed,
                sensor,
                _inner: inner,
                capture_buffer,
            };

            cam.calibrate(3);

            Ok(cam)
        }
    }

    pub fn calibrate(&mut self, frames: u32) {
        for _ in 0..frames {
            let (fr, fg, fb) = self.capture().channel_sums();
            if fr > 0 && fg > 0 && fb > 0 {
                let gr = (fg as f32 / fr as f32).clamp(0.5, 4.0);
                let gb = (fg as f32 / fb as f32).clamp(0.5, 4.0);
                self.wb_r = self.wb_r * 0.5 + gr * 0.5;
                self.wb_b = self.wb_b * 0.5 + gb * 0.5;
            }
        }
    }

    pub fn capture(&mut self) -> Image {
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
        }

        unsafe {
            Image::new(
                self.capture_buffer.buf as *const u8,
                self.sensor.resolution.0,
                self.sensor.resolution.1,
                self.capture_buffer.row_bytes(),
                self.sensor.black_level,
                self.wb_r,
                self.wb_b,
            )
        }
    }

    unsafe fn allocate_capture_buffer(sensor: &CameraSensor) -> Box<CaptureBuffer> {
        let cap_row_bytes = sensor.resolution.0 * 10 / 8;
        let capture_fb_bytes = cap_row_bytes * sensor.resolution.1;
        let cap_buf =
            heap_caps_aligned_calloc(64, 1, capture_fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        assert!(!cap_buf.is_null(), "capture buffer allocation failed");
        Box::new(CaptureBuffer {
            buf: cap_buf,
            len: capture_fb_bytes,
            row_bytes: cap_row_bytes,
        })
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        unsafe {
            heap_caps_free(self.capture_buffer.buf);
        }
    }
}
