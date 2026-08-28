//! MIPI CSI-2 frame transport: the ESP32-P4 camera controller + ISP, streaming
//! raw frames into a DMA capture buffer.

use core::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use esp_idf_sys::*;

use crate::camera::interface::CameraInterface;
use crate::camera::{BayerOrder, CameraError, FrameFormat, PixelFormat};

unsafe extern "C" {
    fn isp_bypass_raw10_patch(h_res: u32, v_res: u32);
}

static FRAME_READY: AtomicBool = AtomicBool::new(false);

/// Holds the capture buffer pointer and length for the CSI DMA callbacks.
/// Heap-allocated so its address is stable after being registered as userdata.
struct CaptureBuffer {
    buf: *mut c_void,
    len: usize,
}

unsafe extern "C" fn on_get_new_trans(
    _: esp_cam_ctlr_handle_t,
    trans: *mut esp_cam_ctlr_trans_t,
    user_ctx: *mut c_void,
) -> bool {
    unsafe {
        let cap = &*(user_ctx as *const CaptureBuffer);
        (*trans).buffer = cap.buf;
        (*trans).buflen = cap.len;
    }
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

/// Board/PHY parameters of the CSI link (the sensor's [`FrameFormat`] supplies
/// the rest at construction).
pub struct CsiConfig {
    /// Number of MIPI CSI-2 data lanes.
    pub data_lane_num: u8,
    /// Per-lane MIPI bit rate in Mbps.
    pub lane_bit_rate_mbps: u16,
    /// LDO channel ID used to power the MIPI CSI-2 PHY.
    pub ldo_channel: i32,
    /// LDO output voltage in millivolts for the MIPI PHY supply.
    pub ldo_voltage_mv: i32,
}

/// The CSI + ISP transport. Push-based: frames stream continuously into the
/// capture buffer via DMA; [`wait_frame`](CameraInterface::wait_frame) blocks
/// on the frame-finished signal.
pub struct CsiInterface {
    _csi: esp_cam_ctlr_handle_t,
    buffer: Box<CaptureBuffer>,
}

impl CsiInterface {
    pub fn new(config: CsiConfig, format: &FrameFormat) -> Result<Self, CameraError> {
        let (color_type, bayer_order) = match format.pixel {
            PixelFormat::Raw10(order) => (cam_ctlr_color_t_CAM_CTLR_COLOR_RAW10, order),
            PixelFormat::Raw8(order) => (cam_ctlr_color_t_CAM_CTLR_COLOR_RAW8, order),
            PixelFormat::Gray16 => return Err(CameraError::UnsupportedFormat),
        };

        unsafe {
            Self::enable_phy_power(&config)?;

            let len = format.bytes_per_frame();
            let buf = heap_caps_aligned_calloc(64, 1, len, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
            assert!(!buf.is_null(), "capture buffer allocation failed");
            let buffer = Box::new(CaptureBuffer { buf, len });

            let csi = Self::set_up_controller(&config, format, color_type, &buffer)?;
            Self::set_up_signal_processor(format, bayer_order)?;

            // Only this first transaction is queued by hand; the callbacks
            // re-supply the same buffer for every following frame.
            let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
            trans.buffer = buffer.buf;
            trans.buflen = buffer.len;
            if esp_cam_ctlr_receive(csi, &mut trans, 100) != ESP_OK {
                return Err(CameraError::Transport);
            }

            Ok(Self { _csi: csi, buffer })
        }
    }

    unsafe fn enable_phy_power(config: &CsiConfig) -> Result<(), CameraError> {
        unsafe {
            let mut ldo_cfg: esp_ldo_channel_config_t = core::mem::zeroed();
            ldo_cfg.chan_id = config.ldo_channel;
            ldo_cfg.voltage_mv = config.ldo_voltage_mv;
            let mut ldo_chan: esp_ldo_channel_handle_t = core::ptr::null_mut();
            if esp_ldo_acquire_channel(&ldo_cfg, &mut ldo_chan) != ESP_OK {
                return Err(CameraError::Transport);
            }
            Ok(())
        }
    }

    unsafe fn set_up_controller(
        config: &CsiConfig,
        format: &FrameFormat,
        color_type: cam_ctlr_color_t,
        buffer: &CaptureBuffer,
    ) -> Result<esp_cam_ctlr_handle_t, CameraError> {
        unsafe {
            let mut csi_cfg: esp_cam_ctlr_csi_config_t = core::mem::zeroed();
            csi_cfg.ctlr_id = 0;
            csi_cfg.h_res = format.width as u32;
            csi_cfg.v_res = format.height as u32;
            csi_cfg.data_lane_num = config.data_lane_num;
            csi_cfg.lane_bit_rate_mbps = config.lane_bit_rate_mbps as i32;
            csi_cfg.input_data_color_type = color_type;
            csi_cfg.output_data_color_type = color_type;
            csi_cfg.queue_items = 1;
            csi_cfg.__bindgen_anon_1.set_bk_buffer_dis(1);
            let mut csi: esp_cam_ctlr_handle_t = core::ptr::null_mut();
            if esp_cam_new_csi_ctlr(&csi_cfg, &mut csi) != ESP_OK {
                return Err(CameraError::Transport);
            }

            let cbs = esp_cam_ctlr_evt_cbs_t {
                on_get_new_trans: Some(on_get_new_trans),
                on_trans_finished: Some(on_trans_finished),
            };
            let user_ctx = buffer as *const CaptureBuffer as *mut c_void;
            if esp_cam_ctlr_register_event_callbacks(csi, &cbs, user_ctx) != ESP_OK
                || esp_cam_ctlr_enable(csi) != ESP_OK
                || esp_cam_ctlr_start(csi) != ESP_OK
            {
                return Err(CameraError::Transport);
            }

            Ok(csi)
        }
    }

    unsafe fn set_up_signal_processor(
        format: &FrameFormat,
        order: BayerOrder,
    ) -> Result<(), CameraError> {
        unsafe {
            let mut isp_cfg: esp_isp_processor_cfg_t = core::mem::zeroed();
            isp_cfg.clk_hz = 80_000_000;
            isp_cfg.input_data_source = isp_input_data_source_t_ISP_INPUT_DATA_SOURCE_CSI;
            isp_cfg.input_data_color_type = isp_color_t_ISP_COLOR_RAW8;
            isp_cfg.output_data_color_type = isp_color_t_ISP_COLOR_RGB565;
            isp_cfg.h_res = format.width as u32;
            isp_cfg.v_res = format.height as u32;
            isp_cfg.bayer_order = match order {
                BayerOrder::Rggb => color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_RGGB,
                BayerOrder::Bggr => color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_BGGR,
                BayerOrder::Grbg => color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_GRBG,
                BayerOrder::Gbrg => color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_GBRG,
            };
            let mut isp: isp_proc_handle_t = core::ptr::null_mut();
            if esp_isp_new_processor(&isp_cfg, &mut isp) != ESP_OK {
                return Err(CameraError::Transport);
            }
            // Route the RAW10 stream around the (RAW8-configured) ISP untouched.
            if matches!(format.pixel, PixelFormat::Raw10(_)) {
                isp_bypass_raw10_patch(format.width as u32, format.height as u32);
            }
            Ok(())
        }
    }
}

impl CameraInterface for CsiInterface {
    type Error = CameraError;

    fn wait_frame(&mut self, timeout: Duration) -> Result<&[u8], Self::Error> {
        let mut waited = Duration::ZERO;
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            if waited >= timeout {
                return Err(CameraError::Transport);
            }
            std::thread::sleep(Duration::from_millis(1));
            waited += Duration::from_millis(1);
        }
        Ok(unsafe { core::slice::from_raw_parts(self.buffer.buf as *const u8, self.buffer.len) })
    }
}

impl Drop for CsiInterface {
    fn drop(&mut self) {
        unsafe { heap_caps_free(self.buffer.buf) };
    }
}
