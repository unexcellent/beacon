use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, config::Resolution, config::TimerConfig};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::FromValueType;
use std::thread;
use std::time::Duration;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let sda = peripherals.pins.gpio31;
    let scl = peripherals.pins.gpio32;
    let mclk_pin = peripherals.pins.gpio36;

    let timer_config = TimerConfig::new()
        .frequency(20.MHz().into())
        .resolution(Resolution::Bits2);
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap();
    let mut mclk = LedcDriver::new(peripherals.ledc.channel0, timer, mclk_pin).unwrap();
    mclk.set_duty(2).unwrap();

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let mut i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    log::info!("Applying M5Unified Tab5 I/O Expander Initialization...");

    // Tab5 Expander 0 (0x43) - Controls CAM_RST, TP_RST, LCD_RST, EXT5V, SPK, RF_PTH
    let init_0x43 = [
        (0x05, 0b01110000), // OUT_SET (Bit 6 is CAM_RST)
        (0x03, 0b01110011), // IO_DIR
        (0x07, 0b00001000), // OUT_H_IM
        (0x0D, 0b00000100), // PULL_SEL
        (0x0B, 0b00000100), // PULL_EN
    ];

    // Tab5 Expander 1 (0x44) - Controls Charge, USB5V, WLAN_PWR
    let init_0x44 = [
        (0x05, 0b10000001), // OUT_SET
        (0x03, 0b10110001), // IO_DIR
        (0x07, 0b00000110), // OUT_H_IM
        (0x0D, 0b00001000), // PULL_SEL
        (0x0B, 0b00001000), // PULL_EN
    ];

    for &(reg, val) in &init_0x43 {
        let _ = i2c.write(0x43, &[reg, val], BLOCK);
    }

    for &(reg, val) in &init_0x44 {
        let _ = i2c.write(0x44, &[reg, val], BLOCK);
    }

    log::info!("Waiting for camera to come out of reset...");
    thread::sleep(Duration::from_millis(250));

    let camera_addr = 0x36;

    loop {
        if i2c.write(camera_addr, &[], BLOCK).is_ok() {
            log::info!(
                "SUCCESS: SC2356 Camera detected at I2C address {:#04x}!",
                camera_addr
            );
        } else {
            log::warn!("Camera not responding at {:#04x}.", camera_addr);
        }
        thread::sleep(Duration::from_secs(5));
    }
}
