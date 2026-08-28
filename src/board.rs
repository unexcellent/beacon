//! Board bring-up for the MOVE-IIIa carrier: pin maps, bus construction,
//! reset sequencing and diagnostics for every peripheral. Everything chip- or
//! transport-generic lives in the `beacon` library; this module is the wiring.

use std::time::Duration;

use beacon::audio::{AudioChannel, AudioInterface, PCM5102A};
use beacon::camera::esp::{
    CsiConfig, CsiInterface, EspI2c, I2cConfig, ResetPin, SpiFrameConfig, SpiFrameInterface,
};
use beacon::camera::sensor::CameraSensor;
use beacon::camera::sensors::{Mi48, Sc850sl};
use beacon::camera::{RgbCamera, ThermalCamera};
use beacon::csp::{CspLink, CspLinkConfig};
use esp_idf_hal::{
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};
use esp_idf_sys::*;

use crate::error::{Error, Result};
use crate::link::{NODE, PayloadLink};

pub const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
pub const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;

pub type RgbCam = RgbCamera<Sc850sl<EspI2c>, CsiInterface>;
pub type ThermalCam = ThermalCamera<Mi48<EspI2c>, SpiFrameInterface>;

/// Interface configuration for the Philips I2S standard at 16 kHz on the ESP32-P4.
///
/// MCLK = 256 × 16 000 Hz = 4.096 MHz. BCLK = MCLK / 8 = 512 kHz,
/// which exactly satisfies the 16 kHz × 2 channels × 16 bits = 512 kHz requirement.
pub const PHILLIPS_I2S: AudioInterface = AudioInterface {
    sample_rate: 16_000,
    clock_divider: 8,
    mclk_pin: 20,
    bclk_pin: 21,
    dout_pin: 22,
    ws_pin: 23,
    chunk_size: 512,
};

pub fn initialize_audio_channel() -> Result<AudioChannel> {
    Ok(AudioChannel::try_new(PCM5102A, PHILLIPS_I2S)?)
}

// ── Payload link: CSP over KISS over RS422 (UART1) ───────────────────────────────

pub const LINK_BAUD_RATE: u32 = 115_200;

/// Bring up the RS422 UART (TX=GPIO38, RX=GPIO37, driver-enable=GPIO39) and
/// the CSP node, and wrap them in the mission's [`PayloadLink`].
pub fn initialize_payload_link(peripherals: Peripherals) -> Result<PayloadLink> {
    let mut de = PinDriver::output(peripherals.pins.gpio39).map_err(|_| Error::Peripheral)?;
    de.set_high().map_err(|_| Error::Peripheral)?;

    let driver = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart::config::Config::new()
            .baudrate(Hertz(LINK_BAUD_RATE))
            .rx_fifo_size(8192),
    )
    .map_err(|_| Error::UartAllocation)?;
    let (uart_tx, uart_rx) = driver.into_split();

    let csp = CspLink::try_new(
        CspLinkConfig {
            address: NODE,
            hostname: "beacon",
            model: "esp32p4",
        },
        uart_tx,
    )
    .map_err(|_| Error::CspInit)?;

    PayloadLink::try_new(csp, uart_rx, de)
}

const RGB_SDA_PIN: i32 = 11;
const RGB_SCL_PIN: i32 = 9;
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
    RgbCamera::try_new(sensor, interface, (OUTPUT_WIDTH, OUTPUT_HEIGHT)).map_err(|_| Error::RgbInit)
}

const THERMAL_CS_PIN: i32 = 31;
const THERMAL_CLK_PIN: i32 = 28;
const THERMAL_MOSI_PIN: i32 = 30;
const THERMAL_MISO_PIN: i32 = 29;
const THERMAL_SDA_PIN: i32 = 12;
const THERMAL_SCL_PIN: i32 = 15;

const THERMAL_SPI: SpiFrameConfig = SpiFrameConfig {
    host: spi_host_device_t_SPI2_HOST,
    cs_pin: THERMAL_CS_PIN,
    clk_pin: THERMAL_CLK_PIN,
    mosi_pin: THERMAL_MOSI_PIN,
    miso_pin: THERMAL_MISO_PIN,
    clock_hz: 7_800_000,
    chunk_bytes: 16_384,
    cs_settle_us: 100,
};

/// Settle delay after the I²C bus is reset and before the first probe, letting the rails
/// and the MI48Dx come up. Matches the reference driver's 2 s post-reset wait.
const THERMAL_BOOT_SETTLE_MS: u64 = 2_000;

pub fn initialize_thermal_camera() -> Result<ThermalCam> {
    log::info!("Thermal camera: initializing (MI1602 via MI48Dx)...");

    let i2c = EspI2c::new(I2cConfig {
        port: i2c_port_t_I2C_NUM_0 as i32,
        sda_pin: THERMAL_SDA_PIN,
        scl_pin: THERMAL_SCL_PIN,
        internal_pullups: true,
        reset_on_init: true,
        scl_speed_hz: 100_000,
        scl_wait_us: 50_000,
        timeout_ms: 100,
    })
    .map_err(|_| Error::ThermalInit)?;

    std::thread::sleep(Duration::from_millis(THERMAL_BOOT_SETTLE_MS));
    scan_thermal_bus(&i2c);

    let mut sensor = Mi48::new(i2c, Mi48::<EspI2c>::DEFAULT_I2C_ADDRESS);
    let frame_bytes = sensor.format().bytes_per_frame();
    sensor.init().map_err(|_| Error::ThermalInit)?;

    let interface =
        SpiFrameInterface::new(THERMAL_SPI, frame_bytes).map_err(|_| Error::ThermalInit)?;
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
