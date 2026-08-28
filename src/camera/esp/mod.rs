//! ESP32-P4 transport and bus implementations.
//!
//! Everything in here is platform code by design: the portable layers are the
//! sensor drivers (generic over `embedded-hal`) and the trait definitions.

mod csi;
mod i2c;
mod spi_frame;

pub use csi::{CsiConfig, CsiInterface};
pub use i2c::{EspI2c, I2cConfig, I2cError};
pub use spi_frame::{SpiFrameConfig, SpiFrameInterface};

use std::time::Duration;

use esp_idf_sys::*;

/// An active-low reset line (e.g. a sensor's XSHUTDN pin), driven as a plain
/// GPIO output so board bring-up code can sequence it around bus setup.
pub struct ResetPin {
    pin: i32,
}

impl ResetPin {
    pub fn new(pin: i32) -> Result<Self, I2cError> {
        let cfg = gpio_config_t {
            pin_bit_mask: 1u64 << pin,
            mode: gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
            hys_ctrl_mode: gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
        };
        i2c::check(unsafe { gpio_config(&cfg) })?;
        Ok(Self { pin })
    }

    /// Drive the line low (device in reset) and hold for `hold`.
    pub fn assert(&self, hold: Duration) {
        unsafe { gpio_set_level(self.pin as gpio_num_t, 0) };
        std::thread::sleep(hold);
    }

    /// Release the line high (device out of reset) and wait `settle`.
    pub fn release(&self, settle: Duration) {
        unsafe { gpio_set_level(self.pin as gpio_num_t, 1) };
        std::thread::sleep(settle);
    }
}
