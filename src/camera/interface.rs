use core::ffi::c_void;
use std::time::Duration;

use esp_idf_sys::*;

use super::CameraSensor;
use super::rgb::{CaptureBuffer, InnerCamera};

unsafe extern "C" {
    fn isp_bypass_raw10_patch(h_res: u32, v_res: u32);
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
    pub(super) unsafe fn init(
        &self,
        sensor: &CameraSensor,
        capture: &CaptureBuffer,
    ) -> Result<InnerCamera, EspError> {
        self.enable_reset()?;
        let (_i2c_bus, i2c_dev) = self.set_up_i2c(sensor)?;
        self.disable_reset()?;
        if !sensor.init(i2c_dev) {
            log::warn!("Sensor init failed, retrying after reset");
            self.enable_reset()?;
            self.disable_reset()?;
            if !sensor.init(i2c_dev) {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
            }
        }
        self.enable_power()?;
        let csi = self.set_up_controller(sensor, capture)?;
        self.set_up_signal_processor(sensor)?;

        Ok(InnerCamera { csi, i2c_dev })
    }

    unsafe fn enable_reset(&self) -> Result<(), EspError> {
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
        std::thread::sleep(Duration::from_millis(500));
        Ok(())
    }

    unsafe fn set_up_i2c(
        &self,
        sensor: &CameraSensor,
    ) -> Result<(i2c_master_bus_handle_t, i2c_master_dev_handle_t), EspError> {
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
        i2c_dev_cfg.scl_wait_us = 5_000;
        let mut i2c_dev: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(i2c_master_bus_add_device(
            i2c_bus,
            &i2c_dev_cfg,
            &mut i2c_dev
        ))?;

        Ok((i2c_bus, i2c_dev))
    }

    unsafe fn disable_reset(&self) -> Result<(), EspError> {
        gpio_set_level(self.xshutdn_pin as gpio_num_t, 1);
        std::thread::sleep(Duration::from_millis(300));
        Ok(())
    }

    unsafe fn enable_power(&self) -> Result<(), EspError> {
        let mut ldo_cfg: esp_ldo_channel_config_t = core::mem::zeroed();
        ldo_cfg.chan_id = self.ldo_channel;
        ldo_cfg.voltage_mv = self.ldo_voltage_mv;
        let mut ldo_chan: esp_ldo_channel_handle_t = core::ptr::null_mut();
        esp!(esp_ldo_acquire_channel(&ldo_cfg, &mut ldo_chan))?;

        Ok(())
    }

    unsafe fn set_up_controller(
        &self,
        sensor: &CameraSensor,
        capture: &CaptureBuffer,
    ) -> Result<esp_cam_ctlr_handle_t, EspError> {
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
            on_get_new_trans: Some(super::rgb::on_get_new_trans),
            on_trans_finished: Some(super::rgb::on_trans_finished),
        };
        esp!(esp_cam_ctlr_register_event_callbacks(
            csi,
            &cbs,
            capture as *const CaptureBuffer as *mut c_void,
        ))?;
        esp!(esp_cam_ctlr_enable(csi))?;
        esp!(esp_cam_ctlr_start(csi))?;

        Ok(csi)
    }

    unsafe fn set_up_signal_processor(&self, sensor: &CameraSensor) -> Result<(), EspError> {
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

        Ok(())
    }
}

pub const MIPI: CameraInterface = CameraInterface {
    data_lane_num: 2,
    lane_bit_rate: 1080,
    sda_pin: 11,
    scl_pin: 9,
    xshutdn_pin: 54,
    ldo_channel: 3,
    ldo_voltage_mv: 2500,
};
