//! Board bring-up for the two MOVE-IIIa cameras: pin maps, bus construction,
//! reset sequencing and diagnostics. Everything chip- or transport-generic
//! lives in [`beacon::camera`](beacon::camera); this module is the wiring.

use std::time::Duration;

use beacon::camera::esp::{
    CsiConfig, CsiInterface, EspI2c, I2cConfig, ResetPin, SpiFrameConfig, SpiFrameInterface,
};
use beacon::camera::sensor::CameraSensor;
use beacon::camera::sensors::{Mi48, Sc850sl};
use beacon::camera::{RgbCamera, ThermalCamera};
use esp_idf_sys::*;

use crate::error::{Error, Result};

/// Robot36 output grid, shared by both cameras and the watermark overlay.
pub const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
pub const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;

pub type RgbCam = RgbCamera<Sc850sl<EspI2c>, CsiInterface>;
pub type ThermalCam = ThermalCamera<Mi48<EspI2c>, SpiFrameInterface>;

// ── RGB camera: SC850SL on MIPI CSI-2 ───────────────────────────────────────────

/// GPIO pin for the sensor control bus (LP I2C).
const RGB_SDA_PIN: i32 = 11;
const RGB_SCL_PIN: i32 = 9;
/// GPIO pin for sensor reset (active-low XSHUTDN).
const RGB_XSHUTDN_PIN: i32 = 54;

const RGB_CSI: CsiConfig = CsiConfig {
    data_lane_num: 2,
    lane_bit_rate_mbps: 1080,
    ldo_channel: 3,
    ldo_voltage_mv: 2500,
};

pub fn initialize_rgb_camera() -> Result<RgbCam> {
    log::info!("RGB camera: initializing...");

    let reset = ResetPin::new(RGB_XSHUTDN_PIN).map_err(|_| Error::RgbInit)?;
    reset.assert(Duration::from_millis(500));

    let i2c = EspI2c::new(I2cConfig {
        port: i2c_port_t_LP_I2C_NUM_0 as i32,
        sda_pin: RGB_SDA_PIN,
        scl_pin: RGB_SCL_PIN,
        internal_pullups: false,
        reset_on_init: false,
        scl_speed_hz: 100_000,
        scl_wait_us: 5_000,
        timeout_ms: 50,
    })
    .map_err(|_| Error::RgbInit)?;
    reset.release(Duration::from_millis(300));

    let mut sensor = Sc850sl::new(i2c, Sc850sl::<EspI2c>::DEFAULT_I2C_ADDRESS);
    if sensor.init().is_err() {
        log::warn!("Sensor init failed, retrying after reset");
        reset.assert(Duration::from_millis(500));
        reset.release(Duration::from_millis(300));
        sensor.init().map_err(|_| Error::RgbInit)?;
    }

    let interface = CsiInterface::new(RGB_CSI, &sensor.format()).map_err(|_| Error::RgbInit)?;
    RgbCamera::try_new(sensor, interface, (OUTPUT_WIDTH, OUTPUT_HEIGHT))
        .map_err(|_| Error::RgbInit)
}

// ── Thermal camera: MI1602 via MI48Dx (I2C control + SPI readout) ────────────────

// SPI2 data bus (MI48Dx is the SPI slave; ESP32 is master, Mode 0, MSB-first).
const THERMAL_CS_PIN: i32 = 31; // G31_RMII_MDC  → SPI2 CS  (MI48 SSN, active low)
const THERMAL_CLK_PIN: i32 = 28; // G28_RMII_RXDV → SPI2 CLK
const THERMAL_MOSI_PIN: i32 = 30; // G30_RMII_RXD1 → SPI2 MOSI (host clocks dummy 0x0000)
const THERMAL_MISO_PIN: i32 = 29; // G29_RMII_RXD0 → SPI2 MISO (thermal data)
// I²C control bus (MI48Dx register access), driven by the HP I²C peripheral via the
// GPIO matrix. On the ESP32-P4 GPIO0-15 are the LP-capable pads, so these equal the
// schematic's LP_GPIO12/15.
const THERMAL_SDA_PIN: i32 = 12; // GPIO12 (LP_GPIO12)
const THERMAL_SCL_PIN: i32 = 15; // GPIO15 (LP_GPIO15)

const THERMAL_SPI: SpiFrameConfig = SpiFrameConfig {
    host: spi_host_device_t_SPI2_HOST,
    cs_pin: THERMAL_CS_PIN,
    clk_pin: THERMAL_CLK_PIN,
    mosi_pin: THERMAL_MOSI_PIN,
    miso_pin: THERMAL_MISO_PIN,
    // 7.8 MHz: the reference driver's proven value (higher rates corrupted the frame CRC).
    clock_hz: 7_800_000,
    chunk_bytes: 16_384,
    cs_settle_us: 100,
};

/// Settle delay after the I²C bus is reset and before the first probe, letting the rails
/// and the MI48Dx come up. Matches the reference driver's 2 s post-reset wait.
const THERMAL_BOOT_SETTLE_MS: u64 = 2_000;

pub fn initialize_thermal_camera() -> Result<ThermalCam> {
    log::info!("Thermal camera: initializing (MI1602 via MI48Dx)...");
    // The MI48Dx RSTN (active-low reset) is not host-driven on this carrier; it must
    // NOT float. Either pull it up to 3V3 on the board, or wire it to a spare GPIO
    // and add a ResetPin pulse here (assert ≥50 µs, then ~50 ms to settle).
    log::warn!(
        "MI48: RSTN not host-driven. If the reset line floats the chip will not boot \
         reliably — pull RSTN up to 3V3, or wire it to a GPIO and reset it here."
    );

    let i2c = EspI2c::new(I2cConfig {
        port: i2c_port_t_I2C_NUM_0 as i32,
        sda_pin: THERMAL_SDA_PIN,
        scl_pin: THERMAL_SCL_PIN,
        // Weak internal pull-ups as a safety net; the board should still have proper
        // external 4.7 kΩ pull-ups on SDA/SCL for reliable comms.
        internal_pullups: true,
        // Free the bus in case the MI48Dx (or a prior half-finished transfer) left SDA
        // held low after the P4 reset — without this the master can find the whole bus
        // wedged and no device ACKs.
        reset_on_init: true,
        // 100 kHz, matching the reference driver. Clock-stretch tolerance (`scl_wait_us`),
        // not a slow clock, is what makes the link reliable here: the MI48Dx holds SCL
        // low for tens of ms while busy. 50 ms is the reference driver's value.
        scl_speed_hz: 100_000,
        scl_wait_us: 50_000,
        timeout_ms: 100,
    })
    .map_err(|_| Error::ThermalInit)?;

    // Let the rails and the MI48Dx settle after the bus reset before the first probe.
    std::thread::sleep(Duration::from_millis(THERMAL_BOOT_SETTLE_MS));
    scan_thermal_bus(&i2c);

    let mut sensor = Mi48::new(i2c, Mi48::<EspI2c>::DEFAULT_I2C_ADDRESS);
    let frame_bytes = sensor.format().bytes_per_frame();
    sensor.init().map_err(|_| Error::ThermalInit)?;

    let interface =
        SpiFrameInterface::new(THERMAL_SPI, frame_bytes).map_err(|_| Error::ThermalInit)?;
    // Mirror left↔right so the scene matches the RGB camera's orientation.
    ThermalCamera::try_new(sensor, interface, (OUTPUT_WIDTH, OUTPUT_HEIGHT), true)
        .map_err(|_| Error::ThermalInit)
}

/// Ground-truth diagnostics: who is actually on the I²C bus, and does the
/// configured device address respond? If nothing answers, report what probing
/// our own address returns so a broken bus (INVALID_STATE) is distinguishable
/// from an empty one (NOT_FOUND).
fn scan_thermal_bus(i2c: &EspI2c) {
    let addr = Mi48::<EspI2c>::DEFAULT_I2C_ADDRESS;
    let found = i2c.scan();
    match found.len() {
        0 => log::error!(
            "MI48: I²C scan found no devices (probe 0x{addr:02x} → {:?}). \
             Check wiring, address, and pull-ups.",
            i2c.probe(addr)
        ),
        1..=8 => {
            let list: Vec<String> = found.iter().map(|a| format!("0x{a:02x}")).collect();
            log::info!("MI48: I²C devices responding: {}", list.join(", "));
        }
        n => log::error!(
            "MI48: {n} addresses ACKed — this is a bus-integrity fault (noisy/stuck SDA), \
             not {n} real devices. Suspect slow rise time (pull-ups too weak, wiring too \
             long, or clock too fast) or a wiring problem on SDA/SCL."
        ),
    }
}
