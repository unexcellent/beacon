//! `embedded-hal` I2C over the esp-idf `i2c_master` driver.
//!
//! esp-idf-hal's `I2cDriver` wraps the legacy driver and exposes neither the
//! LP-I2C port nor per-device clock-stretch tolerance (`scl_wait_us`), both of
//! which the camera buses depend on — so this is a thin wrapper over the new
//! `i2c_master` API instead.

use core::fmt;

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation, SevenBitAddress};
use esp_idf_sys::*;

/// An esp-idf error code, printed by name (e.g. `ESP_ERR_TIMEOUT`) so logs and
/// downlinked reports stay readable.
#[derive(Clone, Copy)]
pub struct I2cError(pub esp_err_t);

impl fmt::Debug for I2cError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = unsafe { core::ffi::CStr::from_ptr(esp_err_to_name(self.0)) };
        f.write_str(name.to_str().unwrap_or("unknown"))
    }
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub(super) fn check(err: esp_err_t) -> Result<(), I2cError> {
    if err == ESP_OK { Ok(()) } else { Err(I2cError(err)) }
}

pub struct I2cConfig {
    /// esp-idf port number; `i2c_port_t_LP_I2C_NUM_0` selects the LP-I2C
    /// peripheral (with its LP clock source), anything else the HP driver.
    pub port: i32,
    pub sda_pin: i32,
    pub scl_pin: i32,
    /// Enable the weak internal pull-ups as a safety net. Boards should still
    /// have proper external pull-ups on SDA/SCL for reliable comms.
    pub internal_pullups: bool,
    /// Reset the bus right after creation, freeing a slave (or a prior
    /// half-finished transfer) that left SDA held low across a host reset.
    pub reset_on_init: bool,
    pub scl_speed_hz: u32,
    /// Clock-stretch tolerance: how long a slave may hold SCL low before the
    /// master aborts the transfer. Slow slaves (e.g. the MI48Dx while busy)
    /// need tens of ms here.
    pub scl_wait_us: u32,
    /// Per-transaction timeout passed to the esp-idf master calls.
    pub timeout_ms: i32,
}

/// One I2C master bus. Implements [`embedded_hal::i2c::I2c`], creating (and
/// caching) an esp-idf device handle per 7-bit address on first use.
pub struct EspI2c {
    bus: i2c_master_bus_handle_t,
    devices: Vec<(u8, i2c_master_dev_handle_t)>,
    scl_speed_hz: u32,
    scl_wait_us: u32,
    timeout_ms: i32,
}

impl EspI2c {
    pub fn new(config: I2cConfig) -> Result<Self, I2cError> {
        unsafe {
            let mut bus_cfg: i2c_master_bus_config_t = core::mem::zeroed();
            bus_cfg.i2c_port = config.port;
            bus_cfg.sda_io_num = config.sda_pin as gpio_num_t;
            bus_cfg.scl_io_num = config.scl_pin as gpio_num_t;
            if config.port == i2c_port_t_LP_I2C_NUM_0 as i32 {
                bus_cfg.__bindgen_anon_1.lp_source_clk =
                    soc_periph_lp_i2c_clk_src_t_LP_I2C_SCLK_DEFAULT;
            } else {
                bus_cfg.__bindgen_anon_1.clk_source =
                    soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT as _;
            }
            bus_cfg.glitch_ignore_cnt = 7;
            if config.internal_pullups {
                bus_cfg.flags.set_enable_internal_pullup(1);
            }
            let mut bus: i2c_master_bus_handle_t = core::ptr::null_mut();
            check(i2c_new_master_bus(&bus_cfg, &mut bus))?;
            if config.reset_on_init {
                check(i2c_master_bus_reset(bus))?;
            }
            Ok(Self {
                bus,
                devices: Vec::new(),
                scl_speed_hz: config.scl_speed_hz,
                scl_wait_us: config.scl_wait_us,
                timeout_ms: config.timeout_ms,
            })
        }
    }

    fn device(&mut self, address: u8) -> Result<i2c_master_dev_handle_t, I2cError> {
        if let Some(&(_, dev)) = self.devices.iter().find(|&&(a, _)| a == address) {
            return Ok(dev);
        }
        unsafe {
            let mut dev_cfg: i2c_device_config_t = core::mem::zeroed();
            dev_cfg.dev_addr_length = i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7;
            dev_cfg.device_address = address as u16;
            dev_cfg.scl_speed_hz = self.scl_speed_hz;
            dev_cfg.scl_wait_us = self.scl_wait_us;
            let mut dev: i2c_master_dev_handle_t = core::ptr::null_mut();
            check(i2c_master_bus_add_device(self.bus, &dev_cfg, &mut dev))?;
            self.devices.push((address, dev));
            Ok(dev)
        }
    }

    /// Whether a device ACKs at `address` (100 ms window, wide enough for a
    /// clock-stretching slave that is still booting).
    pub fn probe(&self, address: u8) -> Result<(), I2cError> {
        check(unsafe { i2c_master_probe(self.bus, address as u16, 100) })
    }

    /// Probe the whole 7-bit address space and return the addresses that ACK.
    pub fn scan(&self) -> Vec<u8> {
        (0x08u8..=0x77)
            .filter(|&addr| self.probe(addr).is_ok())
            .collect()
    }
}

impl ErrorType for EspI2c {
    type Error = I2cError;
}

impl I2c<SevenBitAddress> for EspI2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), I2cError> {
        let dev = self.device(address)?;
        let timeout = self.timeout_ms;
        // The esp-idf master API exposes exactly the three transaction shapes
        // drivers actually use; anything more exotic is rejected.
        match operations {
            [] => Ok(()),
            [Operation::Write(w)] => {
                check(unsafe { i2c_master_transmit(dev, w.as_ptr(), w.len(), timeout) })
            }
            [Operation::Read(r)] => {
                check(unsafe { i2c_master_receive(dev, r.as_mut_ptr(), r.len(), timeout) })
            }
            [Operation::Write(w), Operation::Read(r)] => check(unsafe {
                i2c_master_transmit_receive(
                    dev,
                    w.as_ptr(),
                    w.len(),
                    r.as_mut_ptr(),
                    r.len(),
                    timeout,
                )
            }),
            _ => Err(I2cError(ESP_ERR_NOT_SUPPORTED)),
        }
    }
}
