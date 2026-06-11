#![allow(unsafe_op_in_unsafe_fn)]

mod interface;
mod sensor;

use core::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use esp_idf_sys::*;

pub use interface::{CameraInterface, MIPI};
pub use sensor::{CameraSensor, SC850SL};

static FRAME_READY: AtomicBool = AtomicBool::new(false);

// Holds the capture buffer pointer and length for the CSI DMA callbacks.
// Heap-allocated so its address is stable after being registered as userdata.
struct CaptureBuffer {
    buf: *mut c_void,
    len: usize,
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
    isp: isp_proc_handle_t,
    i2c_dev: i2c_master_dev_handle_t,
    i2c_bus: i2c_master_bus_handle_t,
    ldo_chan: esp_ldo_channel_handle_t,
}

pub struct Camera {
    sensor: CameraSensor,
    _inner: InnerCamera,
    capture_buf: Box<CaptureBuffer>,
    output_buf: *mut u8,
    output_resolution: (usize, usize),
    cap_row_bytes: usize,
}

impl Camera {
    pub unsafe fn new(sensor: CameraSensor, interface: CameraInterface) -> Result<Self, EspError> {
        // Allocate capture buffer in PSRAM (64-byte aligned for L2 cache)
        let cap_row_bytes = sensor.resolution.0 * 10 / 8;
        let capture_fb_bytes = cap_row_bytes * sensor.resolution.1;
        let cap_buf =
            heap_caps_aligned_calloc(64, 1, capture_fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        assert!(!cap_buf.is_null(), "capture buffer allocation failed");
        let capture_buf = Box::new(CaptureBuffer {
            buf: cap_buf,
            len: capture_fb_bytes,
        });

        let inner = interface.init(&sensor, &capture_buf)?;

        // Seed first DMA receive transaction
        let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
        trans.buffer = capture_buf.buf;
        trans.buflen = capture_fb_bytes;
        esp!(esp_cam_ctlr_receive(inner.csi, &mut trans, 100))?;

        // Enable sensor streaming; wait 200 ms for MIPI lanes to stabilize
        sensor.write(inner.i2c_dev, 0x302c, 0x00);
        sensor.write(inner.i2c_dev, 0x0100, 0x01);
        std::thread::sleep(Duration::from_millis(200));

        // One-time image processing state initialization
        crate::image::init(sensor.red_gain_seed, sensor.blue_gain_seed);

        log::info!(
            "camera streaming — capturing {}×{}",
            sensor.resolution.0,
            sensor.resolution.1,
        );

        Ok(Self {
            sensor,
            _inner: inner,
            capture_buf,
            output_buf: core::ptr::null_mut(),
            output_resolution: (0, 0),
            cap_row_bytes,
        })
    }

    pub unsafe fn capture(&mut self, resolution: &(usize, usize)) -> &[u8] {
        // Reallocate output buffer if resolution changed
        if *resolution != self.output_resolution {
            if !self.output_buf.is_null() {
                heap_caps_free(self.output_buf as *mut c_void);
            }
            let out_bytes = resolution.0 * resolution.1 * 3;
            self.output_buf = heap_caps_malloc(out_bytes, MALLOC_CAP_SPIRAM) as *mut u8;
            assert!(
                !self.output_buf.is_null(),
                "output buffer allocation failed"
            );
            self.output_resolution = *resolution;
        }

        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
        }

        crate::image::process_frame(
            self.capture_buf.buf as *const u8,
            self.output_buf,
            resolution.0,
            resolution.1,
            self.sensor.resolution.0,
            self.sensor.resolution.1,
            self.cap_row_bytes,
            self.sensor.black_level,
        );

        core::slice::from_raw_parts(self.output_buf, resolution.0 * resolution.1 * 3)
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        unsafe {
            if !self.output_buf.is_null() {
                heap_caps_free(self.output_buf as *mut c_void);
            }
            heap_caps_free(self.capture_buf.buf);
        }
    }
}
