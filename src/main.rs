#![allow(unsafe_op_in_unsafe_fn)]

mod audio;

use esp_idf_sys::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

unsafe extern "C" {
    fn isp_bypass_raw10_patch(h_res: u32, v_res: u32);
}

// SC850SL MIPI-CSI camera GPIO assignments (from MOVE-IIIa-Imager sdkconfig.defaults)
const I2C_SDA_GPIO: gpio_num_t = 11; // LP_GPIO11
const I2C_SCL_GPIO: gpio_num_t = 9; // LP_GPIO9
const XSHUTDN_GPIO: gpio_num_t = 54; // active-low reset

const CAPTURE_W: usize = 3840;
const CAPTURE_H: usize = 2160;
// RAW10 unpacked: each 10-bit pixel stored in 2 bytes (lower 10 bits valid)
const CAPTURE_FB_BYTES: usize = CAPTURE_W * CAPTURE_H * 2;

static FRAME_READY: AtomicBool = AtomicBool::new(false);
// Shared between main task and ISR callbacks — only written once at startup
static mut CAPTURE_BUF: *mut core::ffi::c_void = core::ptr::null_mut();

unsafe extern "C" fn on_get_new_trans(
    _: esp_cam_ctlr_handle_t,
    trans: *mut esp_cam_ctlr_trans_t,
    _: *mut core::ffi::c_void,
) -> bool {
    (*trans).buffer = CAPTURE_BUF;
    (*trans).buflen = CAPTURE_FB_BYTES;
    false
}

unsafe extern "C" fn on_trans_finished(
    _: esp_cam_ctlr_handle_t,
    _: *mut esp_cam_ctlr_trans_t,
    _: *mut core::ffi::c_void,
) -> bool {
    FRAME_READY.store(true, Ordering::Release);
    false
}

// 16-bit register address + 8-bit value write for SC850SL; returns true on success.
// Retries up to 3 times with 5 ms delay — the camera's I2C slave can be briefly
// unresponsive during the first few ms after XSHUTDN release + bus_reset.
unsafe fn sc850sl_write(dev: i2c_master_dev_handle_t, reg: u16, val: u8) -> bool {
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

// Init table for SC850SL mode: 4K / 15 fps / 2-lane / RAW10 (table_2, 191 entries)
// Source: MOVE-IIIa-Imager/firmware/components/sc850sl/init_tables/sc850sl_table_2.c
#[rustfmt::skip]
const INIT_TABLE: &[(u16, u8)] = &[
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

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("MIPI-CSI brightness monitor starting");
    unsafe { camera_brightness_loop() }
}

unsafe fn camera_brightness_loop() -> ! {
    // 1. Hold SC850SL in reset (XSHUTDN active-low)
    let gpio_cfg = gpio_config_t {
        pin_bit_mask: 1u64 << XSHUTDN_GPIO,
        mode: gpio_mode_t_GPIO_MODE_OUTPUT,
        pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
        intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
        hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
    };
    esp!(gpio_config(&gpio_cfg)).unwrap();
    gpio_set_level(XSHUTDN_GPIO, 0);
    // Hold reset for 200 ms — camera was running since boot (GPIO 54 floated HIGH).
    // Empirically, 20 ms is not always enough for SDA to be released on warm resets.
    std::thread::sleep(Duration::from_millis(200));

    // 2. LP I2C bus (LP_I2C_NUM_0=2, LP_GPIO9 SCL / LP_GPIO11 SDA, 100 kHz)
    //    GPIO 9/11 are in the LP domain (1.8 V). Using HP I2C (port 0) routes
    //    through the HP GPIO matrix whose input threshold is ~2 V (3.3 V domain),
    //    so 1.8 V pull-ups register as LOW → bus_busy always 1. LP I2C uses
    //    rtc_gpio_init which routes through the LP IO mux at 1.8 V.
    let mut i2c_bus_cfg: i2c_master_bus_config_t = core::mem::zeroed();
    i2c_bus_cfg.i2c_port = i2c_port_t_LP_I2C_NUM_0 as i32;
    i2c_bus_cfg.sda_io_num = I2C_SDA_GPIO;
    i2c_bus_cfg.scl_io_num = I2C_SCL_GPIO;
    i2c_bus_cfg.__bindgen_anon_1.lp_source_clk =
        soc_periph_lp_i2c_clk_src_t_LP_I2C_SCLK_DEFAULT;
    i2c_bus_cfg.glitch_ignore_cnt = 7;
    let mut i2c_bus: i2c_master_bus_handle_t = core::ptr::null_mut();
    esp!(i2c_new_master_bus(&i2c_bus_cfg, &mut i2c_bus)).unwrap();

    let mut i2c_dev_cfg: i2c_device_config_t = core::mem::zeroed();
    i2c_dev_cfg.dev_addr_length = i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7;
    i2c_dev_cfg.device_address = 0x30;
    i2c_dev_cfg.scl_speed_hz = 100_000;
    i2c_dev_cfg.scl_wait_us = 50_000;
    let mut i2c_dev: i2c_master_dev_handle_t = core::ptr::null_mut();
    esp!(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &mut i2c_dev)).unwrap();

    // 3. Release reset; 150 ms for I2C to stabilize, then run bus recovery.
    //    bus_reset must be called AFTER XSHUTDN release so the camera's I2C
    //    slave is actually running — calling it while in reset does nothing useful
    //    (nothing is driving SDA, the CLR_BUS pulse sequence completes but any
    //    stuck condition from the sensor's startup hasn't happened yet).
    gpio_set_level(XSHUTDN_GPIO, 1);
    std::thread::sleep(Duration::from_millis(150));
    let _ = i2c_master_bus_reset(i2c_bus);

    // 4. Program SC850SL registers (4K15 2-lane RAW10)
    //    Writes to 0x36e9 / 0x36f9 (PLL dividers) need a 20 ms lock delay
    let mut i2c_failures: u32 = 0;
    for &(reg, val) in INIT_TABLE {
        if !sc850sl_write(i2c_dev, reg, val) {
            i2c_failures += 1;
        }
        if reg == 0x36e9 || reg == 0x36f9 {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    if i2c_failures > 0 {
        log::warn!("SC850SL init: {}/{} register writes failed — camera may not stream", i2c_failures, INIT_TABLE.len());
    }

    // 5. Power MIPI CSI-2 PHY at 2.5 V (LDO channel 3)
    let mut ldo_cfg: esp_ldo_channel_config_t = core::mem::zeroed();
    ldo_cfg.chan_id = 3;
    ldo_cfg.voltage_mv = 2500;
    let mut ldo_chan: esp_ldo_channel_handle_t = core::ptr::null_mut();
    esp!(esp_ldo_acquire_channel(&ldo_cfg, &mut ldo_chan)).unwrap();

    // 6. Allocate 16 MB DMA frame buffer in PSRAM (64-byte aligned for L2 cache)
    let buf = heap_caps_aligned_calloc(
        64,
        1,
        CAPTURE_FB_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA,
    );
    assert!(!buf.is_null(), "frame buffer allocation failed");
    CAPTURE_BUF = buf;

    // 7. CSI controller: 3840×2160, 2-lane, 1080 Mbps/lane, RAW10 pass-through
    let mut csi_cfg: esp_cam_ctlr_csi_config_t = core::mem::zeroed();
    csi_cfg.ctlr_id = 0;
    csi_cfg.h_res = CAPTURE_W as u32;
    csi_cfg.v_res = CAPTURE_H as u32;
    csi_cfg.data_lane_num = 2;
    csi_cfg.lane_bit_rate_mbps = 1080;
    csi_cfg.input_data_color_type = cam_ctlr_color_t_CAM_CTLR_COLOR_RAW10;
    csi_cfg.output_data_color_type = cam_ctlr_color_t_CAM_CTLR_COLOR_RAW10;
    csi_cfg.queue_items = 1;
    csi_cfg.__bindgen_anon_1.set_bk_buffer_dis(1);
    let mut csi: esp_cam_ctlr_handle_t = core::ptr::null_mut();
    esp!(esp_cam_new_csi_ctlr(&csi_cfg, &mut csi)).unwrap();

    let cbs = esp_cam_ctlr_evt_cbs_t {
        on_get_new_trans: Some(on_get_new_trans),
        on_trans_finished: Some(on_trans_finished),
    };
    esp!(esp_cam_ctlr_register_event_callbacks(csi, &cbs, core::ptr::null_mut())).unwrap();
    esp!(esp_cam_ctlr_enable(csi)).unwrap();
    esp!(esp_cam_ctlr_start(csi)).unwrap();

    // 8. ISP processor: create with placeholder colors then patch hardware registers
    //    directly for RAW10 bypass (matches reference project camera_isp_bypass_start).
    //    The driver API doesn't correctly configure frame_cfg for RAW10 passthrough —
    //    isp_bypass_raw10_patch() writes hadr_num/vadr_num/isp_en directly so the
    //    ISP hardware knows the frame dimensions and asserts frame-done to the DMA.
    let mut isp_cfg: esp_isp_processor_cfg_t = core::mem::zeroed();
    isp_cfg.clk_hz = 80_000_000;
    isp_cfg.input_data_source = isp_input_data_source_t_ISP_INPUT_DATA_SOURCE_CSI;
    isp_cfg.input_data_color_type = isp_color_t_ISP_COLOR_RAW8;   // placeholder
    isp_cfg.output_data_color_type = isp_color_t_ISP_COLOR_RGB565; // placeholder
    isp_cfg.h_res = CAPTURE_W as u32;
    isp_cfg.v_res = CAPTURE_H as u32;
    isp_cfg.bayer_order = color_raw_element_order_t_COLOR_RAW_ELEMENT_ORDER_BGGR;
    let mut isp: isp_proc_handle_t = core::ptr::null_mut();
    esp!(esp_isp_new_processor(&isp_cfg, &mut isp)).unwrap();
    isp_bypass_raw10_patch(CAPTURE_W as u32, CAPTURE_H as u32);
    // 9. Seed first DMA receive transaction
    let mut trans: esp_cam_ctlr_trans_t = core::mem::zeroed();
    trans.buffer = CAPTURE_BUF;
    trans.buflen = CAPTURE_FB_BYTES;
    esp!(esp_cam_ctlr_receive(csi, &mut trans, 100)).unwrap();

    // 10. Enable sensor streaming; wait 200 ms for MIPI lanes to stabilize (~3 frames)
    sc850sl_write(i2c_dev, 0x302c, 0x00); // exit deep-sleep auxiliary
    sc850sl_write(i2c_dev, 0x0100, 0x01); // stream on
    std::thread::sleep(Duration::from_millis(200));

    log::info!("SC850SL streaming — printing average RAW10 brightness every second");

    // 11. Brightness loop: one measurement per second
    loop {
        let deadline = std::time::Instant::now() + Duration::from_secs(1);

        // Wait for the ISR to signal a completed DMA frame
        while !FRAME_READY.swap(false, Ordering::AcqRel) {
            std::thread::sleep(Duration::from_millis(1));
        }

        // Average all RAW10 pixels (u16 words, value in bits [9:0])
        let pixels = core::slice::from_raw_parts(CAPTURE_BUF as *const u16, CAPTURE_W * CAPTURE_H);
        let sum: u64 = pixels.iter().map(|&p| (p & 0x3ff) as u64).sum();
        let avg = sum / (CAPTURE_W * CAPTURE_H) as u64;

        log::info!("Average brightness: {}/1023", avg);

        // on_get_new_trans re-arms DMA automatically each frame; no explicit receive needed.

        let now = std::time::Instant::now();
        if deadline > now {
            std::thread::sleep(deadline - now);
        }
    }
}
