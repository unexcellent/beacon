#![allow(unsafe_op_in_unsafe_fn)]

use core::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use esp_idf_sys::*;

unsafe extern "C" {
    fn isp_bypass_raw10_patch(h_res: u32, v_res: u32);
}

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

unsafe fn sensor_write(dev: i2c_master_dev_handle_t, reg: u16, val: u8) -> bool {
    let buf = [(reg >> 8) as u8, reg as u8, val];
    for attempt in 0..3u8 {
        if i2c_master_transmit(dev, buf.as_ptr(), 3, 50) == ESP_OK as i32 {
            return true;
        }
        if attempt < 2 {
            std::thread::sleep(Duration::from_millis(5));
        }
    }
    false
}

/// Describes sensor-specific settings: resolution, MIPI config, I2C address, and init registers.
///
/// These map to the sensor datasheet and are fixed for a given sensor variant.
pub struct OldCameraSensor {
    /// 7-bit I2C device address.
    pub i2c_address: u16,
    /// Full-resolution capture width in pixels.
    pub capture_width: usize,
    /// Full-resolution capture height in pixels.
    pub capture_height: usize,
    /// Number of MIPI CSI-2 data lanes.
    pub data_lane_num: u8,
    /// Per-lane MIPI bit rate in Mbps.
    pub lane_bit_rate_mbps: i32,
    /// Black-level pedestal in 8-bit space (10-bit sensor value >> 2).
    pub black_level: u8,
    /// Initial white-balance red gain seed (green = 1.0 reference).
    pub wb_r_seed: f32,
    /// Initial white-balance blue gain seed (green = 1.0 reference).
    pub wb_b_seed: f32,
    /// Register init table: (address, value) pairs written over I2C at startup.
    pub init_table: &'static [(u16, u8)],
    /// Register addresses that require a 20 ms delay after writing (e.g., PLL latching regs).
    pub post_write_delay_regs: &'static [u16],
}

/// Describes the physical camera interface: I2C wiring, GPIO, power, and output resolution.
///
/// These settings are board-level and independent of the sensor variant.
pub struct OldCameraInterface {
    /// GPIO pin for I2C SDA.
    pub sda_pin: gpio_num_t,
    /// GPIO pin for I2C SCL.
    pub scl_pin: gpio_num_t,
    /// GPIO pin for sensor reset (active-low XSHUTDN).
    pub xshutdn_pin: gpio_num_t,
    /// LDO channel ID used to power the MIPI CSI-2 PHY.
    pub ldo_chan_id: i32,
    /// LDO output voltage in millivolts for the MIPI PHY supply.
    pub ldo_voltage_mv: i32,
    /// Output image width after software downscaling.
    pub output_width: usize,
    /// Output image height after software downscaling.
    pub output_height: usize,
}

/// SC850SL sensor at 4K/15 fps, 2-lane MIPI, RAW10.
pub const SC850SL: OldCameraSensor = OldCameraSensor {
    i2c_address: 0x30,
    capture_width: 3840,
    capture_height: 2160,
    data_lane_num: 2,
    lane_bit_rate_mbps: 1080_i32,
    black_level: 16,
    wb_r_seed: 1.5,
    wb_b_seed: 1.5,
    init_table: SC850SL_INIT_TABLE,
    post_write_delay_regs: &[0x36e9, 0x36f9],
};

/// Default ESP32-P4 camera board wiring for the beacon hardware.
pub const BEACON_INTERFACE: OldCameraInterface = OldCameraInterface {
    sda_pin: 11,
    scl_pin: 9,
    xshutdn_pin: 54,
    ldo_chan_id: 3,
    ldo_voltage_mv: 2500,
    output_width: 320,
    output_height: 240,
};

// Init table for SC850SL mode: 4K / 15 fps / 2-lane / RAW10 (table_2, 191 entries)
// Source: MOVE-IIIa-Imager/firmware/components/sc850sl/init_tables/sc850sl_table_2.c
#[rustfmt::skip]
const SC850SL_INIT_TABLE: &[(u16, u8)] = &[
    (0x0103, 0x01), (0x0100, 0x00), (0x36e9, 0x80), (0x36f9, 0x80),
    (0x36ea, 0x17), (0x36eb, 0x0c), (0x36ec, 0x4a), (0x36ed, 0x24),
    (0x36fa, 0xcb), (0x36fb, 0x13), (0x36fc, 0x00), (0x36fd, 0x07),
    (0x36e9, 0x20), (0x36f9, 0x53), (0x3018, 0x3a), (0x3019, 0xfc),
    (0x301a, 0x30), (0x301e, 0x3c), (0x301f, 0x38), (0x302a, 0x00),
    (0x3031, 0x0a), (0x3032, 0x20), (0x3033, 0x22), (0x3037, 0x00),
    (0x303e, 0xb4), (0x320c, 0x04), (0x320d, 0x4c), (0x320e, 0x08),
    (0x320f, 0xca), (0x3211, 0x0f), (0x3213, 0x07), (0x3226, 0x00),
    (0x3227, 0x03), (0x3250, 0x40), (0x3253, 0x08), (0x327e, 0x00),
    (0x3280, 0x00), (0x3281, 0x00), (0x3301, 0x3c), (0x3304, 0x30),
    (0x3306, 0xe8), (0x3308, 0x10), (0x3309, 0x70), (0x330a, 0x01),
    (0x330b, 0xe0), (0x330d, 0x10), (0x3314, 0x92), (0x331e, 0x29),
    (0x331f, 0x69), (0x3333, 0x10), (0x3347, 0x05), (0x3348, 0xd0),
    (0x3352, 0x01), (0x3356, 0x38), (0x335d, 0x60), (0x3362, 0x70),
    (0x338f, 0x80), (0x33af, 0x48), (0x33fe, 0x00), (0x3400, 0x12),
    (0x3406, 0x04), (0x3410, 0x12), (0x3416, 0x06), (0x3433, 0x01),
    (0x3440, 0x12), (0x3446, 0x08), (0x3478, 0x01), (0x3479, 0x01),
    (0x347a, 0x02), (0x347b, 0x01), (0x347c, 0x04), (0x347d, 0x01),
    (0x3616, 0x0c), (0x3620, 0x92), (0x3622, 0x74), (0x3629, 0x74),
    (0x362a, 0xf0), (0x362b, 0x0f), (0x362d, 0x00), (0x3630, 0x68),
    (0x3633, 0x22), (0x3634, 0x22), (0x3635, 0x20), (0x3637, 0x06),
    (0x3638, 0x26), (0x363b, 0x06), (0x363c, 0x07), (0x363d, 0x05),
    (0x363e, 0x8f), (0x3648, 0xe0), (0x3649, 0x0a), (0x364a, 0x06),
    (0x364c, 0x6a), (0x3650, 0x3d), (0x3654, 0x40), (0x3656, 0x68),
    (0x3657, 0x0f), (0x3658, 0x3d), (0x365c, 0x40), (0x365e, 0x68),
    (0x3901, 0x04), (0x3904, 0x20), (0x3905, 0x91), (0x391e, 0x83),
    (0x3928, 0x04), (0x3933, 0xa0), (0x3934, 0x0a), (0x3935, 0x68),
    (0x3936, 0x00), (0x3937, 0x20), (0x3938, 0x0a), (0x3946, 0x20),
    (0x3961, 0x40), (0x3962, 0x40), (0x3963, 0xc8), (0x3964, 0xc8),
    (0x3965, 0x40), (0x3966, 0x40), (0x3967, 0x00), (0x39cd, 0xc8),
    (0x39ce, 0xc8), (0x3e01, 0x82), (0x3e02, 0x00), (0x3e0e, 0x02),
    (0x3e0f, 0x00), (0x3e1c, 0x0f), (0x3e23, 0x00), (0x3e24, 0x00),
    (0x3e53, 0x00), (0x3e54, 0x00), (0x3e68, 0x00), (0x3e69, 0x80),
    (0x3e73, 0x00), (0x3e74, 0x00), (0x3e86, 0x03), (0x3e87, 0x40),
    (0x3f02, 0x24), (0x4424, 0x02), (0x4501, 0xc4), (0x4509, 0x20),
    (0x4561, 0x12), (0x4800, 0x24), (0x4837, 0x0b), (0x4900, 0x24),
    (0x4937, 0x0b), (0x5000, 0x0e), (0x500f, 0x35), (0x5020, 0x00),
    (0x5787, 0x10), (0x5788, 0x06), (0x5789, 0x00), (0x578a, 0x18),
    (0x578b, 0x0c), (0x578c, 0x00), (0x5790, 0x10), (0x5791, 0x06),
    (0x5792, 0x01), (0x5793, 0x18), (0x5794, 0x0c), (0x5795, 0x01),
    (0x5799, 0x06), (0x57a2, 0x60), (0x59e0, 0xfe), (0x59e1, 0x40),
    (0x59e2, 0x38), (0x59e3, 0x30), (0x59e4, 0x20), (0x59e5, 0x38),
    (0x59e6, 0x30), (0x59e7, 0x20), (0x59e8, 0x3f), (0x59e9, 0x38),
    (0x59ea, 0x30), (0x59eb, 0x3f), (0x59ec, 0x38), (0x59ed, 0x30),
    (0x59ee, 0xfe), (0x59ef, 0x40), (0x59f4, 0x38), (0x59f5, 0x30),
    (0x59f6, 0x20), (0x59f7, 0x38), (0x59f8, 0x30), (0x59f9, 0x20),
    (0x59fa, 0x3f), (0x59fb, 0x38), (0x59fc, 0x30), (0x59fd, 0x3f),
    (0x59fe, 0x38), (0x59ff, 0x30), (0x0100, 0x00),
];

/// An active camera channel configured for a specific sensor and interface.
///
/// The channel is fully initialized and streaming on return from [`new`](OldCameraChannel::new).
/// Call [`capture_rgb888`](OldCameraChannel::capture_rgb888) to wait for the next frame and
/// retrieve the downscaled, white-balanced RGB888 image. The underlying hardware is released
/// automatically on drop.
pub struct OldCameraChannel {
    _csi: esp_cam_ctlr_handle_t,
    _isp: isp_proc_handle_t,
    _i2c_dev: i2c_master_dev_handle_t,
    _i2c_bus: i2c_master_bus_handle_t,
    _ldo_chan: esp_ldo_channel_handle_t,
    capture: Box<CaptureBuffer>,
    output_buf: *mut u8,
    out_w: usize,
    out_h: usize,
    cap_w: usize,
    cap_h: usize,
    cap_row_bytes: usize,
    black_level: u8,
}

impl OldCameraChannel {
    /// Initializes the camera sensor and CSI controller, then starts streaming.
    ///
    /// Performs GPIO reset, I2C sensor programming, PSRAM buffer allocation, CSI/ISP
    /// hardware setup, and seeds the first DMA transaction. Returns when the sensor is
    /// actively streaming and the first frame may arrive at any moment.
    pub unsafe fn new(
        sensor: OldCameraSensor,
        interface: OldCameraInterface,
    ) -> Result<Self, EspError> {
        // 1. Hold sensor in reset (XSHUTDN active-low)
        let gpio_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << interface.xshutdn_pin,
            mode: gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        esp!(gpio_config(&gpio_cfg))?;
        gpio_set_level(interface.xshutdn_pin, 0);
        std::thread::sleep(Duration::from_millis(500)); // 500ms: ensures SDA is released after warm resets

        // 2. LP I2C bus (LP_I2C_NUM_0=2, LP_GPIO9 SCL / LP_GPIO11 SDA, 100 kHz)
        let mut i2c_bus_cfg: i2c_master_bus_config_t = core::mem::zeroed();
        i2c_bus_cfg.i2c_port = i2c_port_t_LP_I2C_NUM_0 as i32;
        i2c_bus_cfg.sda_io_num = interface.sda_pin;
        i2c_bus_cfg.scl_io_num = interface.scl_pin;
        i2c_bus_cfg.__bindgen_anon_1.lp_source_clk =
            soc_periph_lp_i2c_clk_src_t_LP_I2C_SCLK_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        let mut i2c_bus: i2c_master_bus_handle_t = core::ptr::null_mut();
        esp!(i2c_new_master_bus(&i2c_bus_cfg, &mut i2c_bus))?;

        let mut i2c_dev_cfg: i2c_device_config_t = core::mem::zeroed();
        i2c_dev_cfg.dev_addr_length = i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7;
        i2c_dev_cfg.device_address = sensor.i2c_address;
        i2c_dev_cfg.scl_speed_hz = 100_000;
        i2c_dev_cfg.scl_wait_us = 50_000;
        let mut i2c_dev: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(i2c_master_bus_add_device(
            i2c_bus,
            &i2c_dev_cfg,
            &mut i2c_dev
        ))?;

        // 3. Release reset; give the sensor 300 ms to bring its I2C interface up,
        //    then retry bus recovery until the stuck SDA/SCL lines are clear.
        gpio_set_level(interface.xshutdn_pin, 1);
        std::thread::sleep(Duration::from_millis(300));
        let mut bus_ok = false;
        for attempt in 0..5u8 {
            if i2c_master_bus_reset(i2c_bus) == ESP_OK as i32 {
                bus_ok = true;
                break;
            }
            log::warn!("I2C bus reset attempt {} failed, retrying", attempt + 1);
            std::thread::sleep(Duration::from_millis(200));
        }
        if !bus_ok {
            log::warn!(
                "I2C bus recovery failed after 5 attempts — continuing with default sensor state"
            );
        }

        // 4. Program sensor registers
        let mut i2c_failures: u32 = 0;
        for &(reg, val) in sensor.init_table {
            if !sensor_write(i2c_dev, reg, val) {
                i2c_failures += 1;
            }
            if sensor.post_write_delay_regs.contains(&reg) {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        if i2c_failures > 0 {
            log::warn!(
                "sensor init: {}/{} register writes failed",
                i2c_failures,
                sensor.init_table.len()
            );
        }

        // 5. Power MIPI CSI-2 PHY
        let mut ldo_cfg: esp_ldo_channel_config_t = core::mem::zeroed();
        ldo_cfg.chan_id = interface.ldo_chan_id;
        ldo_cfg.voltage_mv = interface.ldo_voltage_mv;
        let mut ldo_chan: esp_ldo_channel_handle_t = core::ptr::null_mut();
        esp!(esp_ldo_acquire_channel(&ldo_cfg, &mut ldo_chan))?;

        // 6. Allocate frame buffers in PSRAM (64-byte aligned for L2 cache)
        let cap_row_bytes = sensor.capture_width * 10 / 8;
        let capture_fb_bytes = cap_row_bytes * sensor.capture_height;
        let out_bytes = interface.output_width * interface.output_height * 3;

        let cap_buf =
            heap_caps_aligned_calloc(64, 1, capture_fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        assert!(!cap_buf.is_null(), "capture buffer allocation failed");
        let capture = Box::new(CaptureBuffer { buf: cap_buf, len: capture_fb_bytes });

        let out_buf = heap_caps_malloc(out_bytes, MALLOC_CAP_SPIRAM);
        assert!(!out_buf.is_null(), "output buffer allocation failed");

        // 7. CSI controller: RAW10 pass-through
        let mut csi_cfg: esp_cam_ctlr_csi_config_t = core::mem::zeroed();
        csi_cfg.ctlr_id = 0;
        csi_cfg.h_res = sensor.capture_width as u32;
        csi_cfg.v_res = sensor.capture_height as u32;
        csi_cfg.data_lane_num = sensor.data_lane_num as u8;
        csi_cfg.lane_bit_rate_mbps = sensor.lane_bit_rate_mbps;
        csi_cfg.input_data_color_type = cam_ctlr_color_t_CAM_CTLR_COLOR_RAW10;
        csi_cfg.output_data_color_type = cam_ctlr_color_t_CAM_CTLR_COLOR_RAW10;
        csi_cfg.queue_items = 1;
        csi_cfg.__bindgen_anon_1.set_bk_buffer_dis(1);
        let mut csi: esp_cam_ctlr_handle_t = core::ptr::null_mut();
        esp!(esp_cam_new_csi_ctlr(&csi_cfg, &mut csi))?;

        let cbs = esp_cam_ctlr_evt_cbs_t {
            on_get_new_trans: Some(on_get_new_trans),
            on_trans_finished: Some(on_trans_finished),
        };
        esp!(esp_cam_ctlr_register_event_callbacks(
            csi,
            &cbs,
            &*capture as *const CaptureBuffer as *mut c_void,
        ))?;
        esp!(esp_cam_ctlr_enable(csi))?;
        esp!(esp_cam_ctlr_start(csi))?;

        // 8. ISP processor: create with placeholder colors then patch for RAW10 bypass
        let mut isp_cfg: esp_isp_processor_cfg_t = core::mem::zeroed();
        isp_cfg.clk_hz = 80_000_000;
        isp_cfg.input_data_source = isp_input_data_source_t_ISP_INPUT_DATA_SOURCE_CSI;
        isp_cfg.input_data_color_type = isp_color_t_ISP_COLOR_RAW8;
        isp_cfg.output_data_color_type = isp_color_t_ISP_COLOR_RGB565;
        isp_cfg.h_res = sensor.capture_width as u32;
        isp_cfg.v_res = sensor.capture_height as u32;
        isp_cfg.bayer_order = color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_RGGB;
        let mut isp: isp_proc_handle_t = core::ptr::null_mut();
        esp!(esp_isp_new_processor(&isp_cfg, &mut isp))?;
        isp_bypass_raw10_patch(sensor.capture_width as u32, sensor.capture_height as u32);

        // 9. Seed first DMA receive transaction
        let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
        trans.buffer = capture.buf;
        trans.buflen = capture_fb_bytes;
        esp!(esp_cam_ctlr_receive(csi, &mut trans, 100))?;

        // 10. Enable sensor streaming; wait 200 ms for MIPI lanes to stabilize
        sensor_write(i2c_dev, 0x302c, 0x00);
        sensor_write(i2c_dev, 0x0100, 0x01);
        std::thread::sleep(Duration::from_millis(200));

        // 11. One-time ISP state initialization
        crate::image::init(sensor.wb_r_seed, sensor.wb_b_seed);

        log::info!(
            "camera streaming — capturing {}×{} → {}×{} RGB888",
            sensor.capture_width,
            sensor.capture_height,
            interface.output_width,
            interface.output_height,
        );

        Ok(Self {
            _csi: csi,
            _isp: isp,
            _i2c_dev: i2c_dev,
            _i2c_bus: i2c_bus,
            _ldo_chan: ldo_chan,
            capture,
            output_buf: out_buf as *mut u8,
            out_w: interface.output_width,
            out_h: interface.output_height,
            cap_w: sensor.capture_width,
            cap_h: sensor.capture_height,
            cap_row_bytes,
            black_level: sensor.black_level,
        })
    }

    /// Blocks until the next frame is ready, runs the software ISP pipeline, and returns
    /// the resulting downscaled RGB888 image as a flat byte slice (R, G, B per pixel, row-major).
    pub unsafe fn capture_rgb888(&mut self) -> &[u8] {
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
        }
        crate::image::process_frame(
            self.capture.buf as *const u8,
            self.output_buf,
            self.out_w,
            self.out_h,
            self.cap_w,
            self.cap_h,
            self.cap_row_bytes,
            self.black_level,
        );
        core::slice::from_raw_parts(self.output_buf, self.out_w * self.out_h * 3)
    }

    /// Returns the output image dimensions as `(width, height)` in pixels.
    pub fn output_size(&self) -> (usize, usize) {
        (self.out_w, self.out_h)
    }
}

impl Drop for OldCameraChannel {
    fn drop(&mut self) {
        unsafe {
            heap_caps_free(self.output_buf as *mut c_void);
            heap_caps_free(self.capture.buf);
        }
    }
}
