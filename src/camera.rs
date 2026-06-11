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

pub struct CameraSensor {
    /// 7-bit I2C address of the sensor on the control bus.
    pub i2c_address: u8,
    /// Width, height of the captured image in pixels.
    pub resolution: (usize, usize),
    /// The sensor's black-level pedestal in 8-bit space.
    pub black_level: u8,
    /// Initial white-balance red gain seed (green = 1.0 reference).
    pub red_gain_seed: f32,
    /// Initial white-balance blue gain seed (green = 1.0 reference).
    pub blue_gain_seed: f32,
    /// Register init table: (address, value) pairs written over I2C at startup.
    pub init_table: &'static [(u16, u8)],
    /// Register addresses that require a 20 ms delay after writing (e.g., PLL latching regs).
    pub delayed_registers: &'static [u16],
}

impl CameraSensor {
    unsafe fn init(&self, i2c_dev: i2c_master_dev_handle_t) {
        let mut failures: u32 = 0;
        for &(reg, val) in self.init_table {
            if !sensor_write(i2c_dev, reg, val) {
                failures += 1;
            }
            if self.delayed_registers.contains(&reg) {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        if failures > 0 {
            log::warn!(
                "sensor init: {}/{} register writes failed",
                failures,
                self.init_table.len()
            );
        }
    }
}

pub struct CameraInterface {
    /// Number of MIPI CSI-2 data lanes.
    pub data_lane_num: u8,
    /// Per-lane MIPI bit rate in Mbps.
    pub lane_bit_rate: u16,
    /// GPIO pin for I2C SDA.
    pub sda_pin: u8,
    /// GPIO pin for I2C SCL.
    pub scl_pin: u8,
    /// GPIO pin for sensor reset (active-low XSHUTDN).
    pub xshutdn_pin: u8,
    /// LDO channel ID used to power the MIPI CSI-2 PHY.
    pub ldo_channel: i32,
    /// LDO output voltage in millivolts for the MIPI PHY supply.
    pub ldo_voltage_mv: i32,
}

impl CameraInterface {
    unsafe fn init(
        &self,
        sensor: &CameraSensor,
        capture: &CaptureBuffer,
    ) -> Result<InnerCamera, EspError> {
        // 1. Hold sensor in reset (XSHUTDN active-low)
        let gpio_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << self.xshutdn_pin,
            mode: gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        esp!(gpio_config(&gpio_cfg))?;
        gpio_set_level(self.xshutdn_pin as gpio_num_t, 0);
        std::thread::sleep(Duration::from_millis(500)); // 500ms: ensures SDA is released after warm resets

        // 2. LP I2C bus (LP_I2C_NUM_0=2, LP_GPIO9 SCL / LP_GPIO11 SDA, 100 kHz)
        let mut i2c_bus_cfg: i2c_master_bus_config_t = core::mem::zeroed();
        i2c_bus_cfg.i2c_port = i2c_port_t_LP_I2C_NUM_0 as i32;
        i2c_bus_cfg.sda_io_num = self.sda_pin as gpio_num_t;
        i2c_bus_cfg.scl_io_num = self.scl_pin as gpio_num_t;
        i2c_bus_cfg.__bindgen_anon_1.lp_source_clk =
            soc_periph_lp_i2c_clk_src_t_LP_I2C_SCLK_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        let mut i2c_bus: i2c_master_bus_handle_t = core::ptr::null_mut();
        esp!(i2c_new_master_bus(&i2c_bus_cfg, &mut i2c_bus))?;

        let mut i2c_dev_cfg: i2c_device_config_t = core::mem::zeroed();
        i2c_dev_cfg.dev_addr_length = i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7;
        i2c_dev_cfg.device_address = sensor.i2c_address as u16;
        i2c_dev_cfg.scl_speed_hz = 100_000;
        i2c_dev_cfg.scl_wait_us = 50_000;
        let mut i2c_dev: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &mut i2c_dev))?;

        // 3. Release reset; give the sensor 300 ms to bring its I2C interface up,
        //    then retry bus recovery until the stuck SDA/SCL lines are clear.
        gpio_set_level(self.xshutdn_pin as gpio_num_t, 1);
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
        sensor.init(i2c_dev);

        // 5. Power MIPI CSI-2 PHY
        let mut ldo_cfg: esp_ldo_channel_config_t = core::mem::zeroed();
        ldo_cfg.chan_id = self.ldo_channel;
        ldo_cfg.voltage_mv = self.ldo_voltage_mv;
        let mut ldo_chan: esp_ldo_channel_handle_t = core::ptr::null_mut();
        esp!(esp_ldo_acquire_channel(&ldo_cfg, &mut ldo_chan))?;

        // 6. CSI controller: RAW10 pass-through
        let mut csi_cfg: esp_cam_ctlr_csi_config_t = core::mem::zeroed();
        csi_cfg.ctlr_id = 0;
        csi_cfg.h_res = sensor.resolution.0 as u32;
        csi_cfg.v_res = sensor.resolution.1 as u32;
        csi_cfg.data_lane_num = self.data_lane_num;
        csi_cfg.lane_bit_rate_mbps = self.lane_bit_rate as i32;
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
            capture as *const CaptureBuffer as *mut c_void,
        ))?;
        esp!(esp_cam_ctlr_enable(csi))?;
        esp!(esp_cam_ctlr_start(csi))?;

        // 7. ISP processor: create with placeholder colors then patch for RAW10 bypass
        let mut isp_cfg: esp_isp_processor_cfg_t = core::mem::zeroed();
        isp_cfg.clk_hz = 80_000_000;
        isp_cfg.input_data_source = isp_input_data_source_t_ISP_INPUT_DATA_SOURCE_CSI;
        isp_cfg.input_data_color_type = isp_color_t_ISP_COLOR_RAW8;
        isp_cfg.output_data_color_type = isp_color_t_ISP_COLOR_RGB565;
        isp_cfg.h_res = sensor.resolution.0 as u32;
        isp_cfg.v_res = sensor.resolution.1 as u32;
        isp_cfg.bayer_order = color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_RGGB;
        let mut isp: isp_proc_handle_t = core::ptr::null_mut();
        esp!(esp_isp_new_processor(&isp_cfg, &mut isp))?;
        isp_bypass_raw10_patch(sensor.resolution.0 as u32, sensor.resolution.1 as u32);

        Ok(InnerCamera { csi, isp, i2c_dev, i2c_bus, ldo_chan })
    }
}

pub struct Camera {
    sensor: CameraSensor,
    interface: CameraInterface,
    _inner: InnerCamera,
    capture_buf: Box<CaptureBuffer>,
    output_buf: *mut u8,
    output_resolution: (usize, usize),
    cap_row_bytes: usize,
}

impl Camera {
    pub unsafe fn new(
        sensor: CameraSensor,
        interface: CameraInterface,
    ) -> Result<Self, EspError> {
        // Allocate capture buffer in PSRAM (64-byte aligned for L2 cache)
        let cap_row_bytes = sensor.resolution.0 * 10 / 8;
        let capture_fb_bytes = cap_row_bytes * sensor.resolution.1;
        let cap_buf = heap_caps_aligned_calloc(
            64,
            1,
            capture_fb_bytes,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA,
        );
        assert!(!cap_buf.is_null(), "capture buffer allocation failed");
        let capture_buf = Box::new(CaptureBuffer { buf: cap_buf, len: capture_fb_bytes });

        let inner = interface.init(&sensor, &capture_buf)?;

        // Seed first DMA receive transaction
        let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
        trans.buffer = capture_buf.buf;
        trans.buflen = capture_fb_bytes;
        esp!(esp_cam_ctlr_receive(inner.csi, &mut trans, 100))?;

        // Enable sensor streaming; wait 200 ms for MIPI lanes to stabilize
        sensor_write(inner.i2c_dev, 0x302c, 0x00);
        sensor_write(inner.i2c_dev, 0x0100, 0x01);
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
            interface,
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
            assert!(!self.output_buf.is_null(), "output buffer allocation failed");
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

struct InnerCamera {
    csi: esp_cam_ctlr_handle_t,
    isp: isp_proc_handle_t,
    i2c_dev: i2c_master_dev_handle_t,
    i2c_bus: i2c_master_bus_handle_t,
    ldo_chan: esp_ldo_channel_handle_t,
}

pub const SC850SL: CameraSensor = CameraSensor {
    i2c_address: 0x30,
    resolution: (3840, 2160),
    black_level: 16,
    red_gain_seed: 1.5,
    blue_gain_seed: 1.5,
    init_table: SC850SL_INIT_TABLE,
    delayed_registers: &[0x36e9, 0x36f9],
};

pub const MIPI: CameraInterface = CameraInterface {
    data_lane_num: 2,
    lane_bit_rate: 1080,
    sda_pin: 11,
    scl_pin: 9,
    xshutdn_pin: 54,
    ldo_channel: 3,
    ldo_voltage_mv: 2500,
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
