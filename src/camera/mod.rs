#![allow(unsafe_op_in_unsafe_fn)]

mod image;
mod interface;
mod sensor;
pub(crate) mod watermark;

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

pub(super) struct InnerCamera {
    pub(super) csi: esp_cam_ctlr_handle_t,
    pub(super) i2c_dev: i2c_master_dev_handle_t,
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

// Auto-exposure tuning. Metering works in the 8-bit MSB space (0..255) on the high percentile
// of the frame (see image::meter) rather than the mean, so bimodal orbit scenes meter sanely.
const AE_TARGET_LUMA: f32 = 230.0; // drive the high percentile to just below saturation
const AE_LUMA_LOW: f32 = 205.0; // converged band (lower bound)
const AE_LUMA_HIGH: f32 = 248.0; // converged band (upper bound)
const AE_CLIP_LIMIT: f32 = 0.02; // tolerate up to 2% near-saturated pixels
const AE_HL_SCALE: f32 = 0.7; // forced exposure cut per step while highlights clip
const AE_GAIN_MAX: f32 = 48.0; // analog-gain ceiling (sensor table tops out ~49.6x)
const AE_MAX_ITERS: u32 = 6; // frame budget for convergence

pub struct Camera {
    sensor: CameraSensor,
    capture_buffer: Box<CaptureBuffer>,
    inner: InnerCamera,
    wb_r: f32,
    wb_b: f32,
    /// Current sensor integration time in lines (AE state, persists across activations).
    exposure: u32,
    /// Current analog gain as a linear multiplier (1.0 = unity).
    gain: f32,
}

impl Camera {
    pub fn new(sensor: CameraSensor, interface: CameraInterface) -> Result<Self, EspError> {
        unsafe {
            let capture_buffer = Self::allocate_capture_buffer(&sensor);

            let inner = interface.init(&sensor, &capture_buffer)?;
            inner.queue_receive(&capture_buffer)?;

            if !sensor.enable(inner.i2c_dev) {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
            }

            let mut cam = Self {
                wb_r: sensor.red_gain_seed,
                wb_b: sensor.blue_gain_seed,
                exposure: sensor.default_exposure as u32,
                gain: 1.0,
                sensor,
                inner,
                capture_buffer,
            };

            // Calibrate briefly to verify streaming works, then go to standby.
            cam.calibrate(3);
            cam.deactivate();

            Ok(cam)
        }
    }

    /// Enable sensor streaming and converge auto-exposure before the next capture.
    pub fn activate(&mut self) {
        unsafe { self.sensor.enable(self.inner.i2c_dev); }
        self.auto_expose();
    }

    /// Closed-loop auto-exposure: meter the raw frame and drive the sensor's integration
    /// time (and analog gain, only once exposure saturates) so bright scenes stop clipping.
    ///
    /// Exposure is the primary lever and is preferred all the way to its ceiling before any
    /// gain is added, since gain only amplifies noise. Highlight-priority metering forces a
    /// cut whenever a bright source clips, independent of the average brightness.
    fn auto_expose(&mut self) {
        for i in 0..AE_MAX_ITERS {
            self.wait_for_frame();
            let (p_high, clip) = unsafe {
                image::meter(
                    self.capture_buffer.buf as *const u8,
                    self.sensor.resolution.0,
                    self.sensor.resolution.1,
                    self.capture_buffer.row_bytes(),
                )
            };

            let converged =
                clip <= AE_CLIP_LIMIT && p_high >= AE_LUMA_LOW && p_high <= AE_LUMA_HIGH;
            log::info!(
                "AE iter {i}: p95={p_high:.0} clip={:.1}% exp={} gain={:.2}x -> {}",
                clip * 100.0,
                self.exposure,
                self.gain,
                if converged { "converged" } else { "adjust" }
            );
            if converged {
                return;
            }

            // Desired multiplicative change in total light. Clipping forces a reduction even
            // if the percentile looks fine (a small, very bright spot in an otherwise dim scene).
            let luma_scale = AE_TARGET_LUMA / p_high.max(1.0);
            let scale = if clip > AE_CLIP_LIMIT {
                luma_scale.min(AE_HL_SCALE)
            } else {
                luma_scale
            };

            // Distribute the change over exposure (preferred) then gain.
            let max_exp = self.sensor.max_exposure() as f32;
            let light = self.exposure as f32 * self.gain;
            let desired = (light * scale).clamp(1.0, max_exp * AE_GAIN_MAX);
            let (exp, gain) = if desired <= max_exp {
                (desired.max(1.0), 1.0)
            } else {
                (max_exp, (desired / max_exp).min(AE_GAIN_MAX))
            };
            self.exposure = exp.round() as u32;
            self.gain = gain;

            unsafe {
                self.sensor.set_exposure(self.inner.i2c_dev, self.exposure);
                self.sensor.set_analog_gain(self.inner.i2c_dev, self.gain);
            }

            // Let the new exposure/gain take effect (applied on the next frame) before re-metering.
            self.wait_for_frame();
            self.wait_for_frame();
        }
    }

    /// Put sensor into standby (stops streaming, saves power).
    pub fn deactivate(&mut self) {
        unsafe { self.sensor.disable(self.inner.i2c_dev); }
    }

    pub fn calibrate(&mut self, frames: u32) {
        for _ in 0..frames {
            self.wait_for_frame();
        }
    }

    pub fn capture(&mut self) -> Image {
        self.wait_for_frame();
        unsafe {
            Image::new(
                self.capture_buffer.buf as *const u8,
                &self.sensor,
                self.capture_buffer.row_bytes(),
                self.wb_r,
                self.wb_b,
            )
        }
    }

    fn wait_for_frame(&self) {
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
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
