//! Board bring-up for the MOVE-IIIa carrier: pin maps, bus construction,
//! reset sequencing and diagnostics for every peripheral. Everything chip- or
//! transport-generic lives in the `beacon` library; this module is the wiring.

use std::time::Duration;

use beacon::audio::{I2sConfig, I2sInterface, Pcm5102a};
use beacon::camera::esp::{
    CsiConfig, CsiInterface, EspI2c, I2cConfig, ResetPin, SpiFrameConfig, SpiFrameInterface,
};
use beacon::camera::sensors::{Mi48, Sc850sl};
use beacon::csp::{CspLink, CspLinkConfig};
use esp_idf_hal::{
    gpio::{AnyIOPin, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver, UartRxDriver},
    units::Hertz,
};
use esp_idf_sys::*;

use crate::error::{Error, Result};
use crate::link::{NODE, PayloadLink};

pub const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
pub const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;


/// Interface configuration for the Philips I2S standard at 16 kHz on the ESP32-P4.
///
/// MCLK = 256 × 16 000 Hz = 4.096 MHz. BCLK = MCLK / 8 = 512 kHz,
/// which exactly satisfies the 16 kHz × 2 channels × 16 bits = 512 kHz requirement.
pub const PHILLIPS_I2S: I2sConfig = I2sConfig {
    sample_rate: 16_000,
    clock_divider: 8,
    mclk_pin: 20,
    bclk_pin: 21,
    dout_pin: 22,
    ws_pin: 23,
    chunk_size: 512,
};

pub fn initialize_audio_channel() -> Result<Pcm5102a<I2sInterface>> {
    let interface = I2sInterface::new(&Pcm5102a::<I2sInterface>::ENCODER, &PHILLIPS_I2S)?;
    Ok(Pcm5102a::new(
        interface,
        PHILLIPS_I2S.sample_rate,
        PHILLIPS_I2S.chunk_size,
    ))
}

// ── Payload link: CSP over KISS over RS422 (UART1) ───────────────────────────────

pub const LINK_BAUD_RATE: u32 = 115_200;

/// The mission's payload link over the ESP32-P4 UART RX transport.
pub type Rs422Link = PayloadLink<UartRxDriver<'static>>;

/// Bring up the RS422 UART (TX=GPIO38, RX=GPIO37, driver-enable=GPIO39) and
/// the CSP node, and wrap them in the mission's [`PayloadLink`].
pub fn initialize_payload_link(peripherals: Peripherals) -> Result<Rs422Link> {
    let mut de = PinDriver::output(peripherals.pins.gpio39).map_err(|_| Error::Peripheral)?;
    de.set_high().map_err(|_| Error::Peripheral)?;
    // RS422 full-duplex: the TX driver-enable line is held high for the whole
    // run. The firmware never exits, so leak the pin rather than parking it in
    // the link — dropping the driver would reset the line and disable the
    // transmitter.
    core::mem::forget(de);

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
        uart_rx,
    )
    .map_err(|_| Error::CspInit)?;

    PayloadLink::try_new(csp)
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

pub fn initialize_rgb_camera() -> Result<Sc850sl<CsiInterface>> {
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

    let format = Sc850sl::<CsiInterface>::FORMAT;
    let interface = CsiInterface::new(i2c, RGB_CSI, &format).map_err(|_| Error::RgbInit)?;
    let mut camera = Sc850sl::new(
        interface,
        Sc850sl::<CsiInterface>::DEFAULT_I2C_ADDRESS,
        (OUTPUT_WIDTH, OUTPUT_HEIGHT),
    );
    if camera.init().is_err() {
        log::warn!("Sensor init failed, retrying after reset");
        reset.assert(Duration::from_millis(500));
        reset.release(Duration::from_millis(300));
        camera.init().map_err(|_| Error::RgbInit)?;
    }
    Ok(camera)
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

pub fn initialize_thermal_camera() -> Result<Mi48<SpiFrameInterface>> {
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

    let frame_bytes = Mi48::<SpiFrameInterface>::FORMAT.bytes_per_frame();
    let interface =
        SpiFrameInterface::new(i2c, THERMAL_SPI, frame_bytes).map_err(|_| Error::ThermalInit)?;
    let mut camera = Mi48::new(
        interface,
        Mi48::<SpiFrameInterface>::DEFAULT_I2C_ADDRESS,
        (OUTPUT_WIDTH, OUTPUT_HEIGHT),
        true,
    );
    camera.init().map_err(|_| Error::ThermalInit)?;
    Ok(camera)
}

/// Ground-truth diagnostics: who is actually on the I²C bus, and does the
/// configured device address respond? If nothing answers, report what probing
/// our own address returns so a broken bus (INVALID_STATE) is distinguishable
/// from an empty one (NOT_FOUND).
fn scan_thermal_bus(i2c: &EspI2c) {
    let addr = Mi48::<SpiFrameInterface>::DEFAULT_I2C_ADDRESS;
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
