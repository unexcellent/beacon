mod devices;
mod error;
mod ota;
mod transmit_sstv;

use devices::audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use devices::camera::{MIPI, RgbCamera, SC850SL, ThermalCamera, capture_both};
use devices::debug::DebugChannel;
use devices::link::{Command, Message, PayloadLink};
use error::{Error, ReportIfErr, Result};
use transmit_sstv::transmit_sstv;

use esp_idf_hal::{delay, peripherals::Peripherals};

/// Firmware identity for ground validation: app version + ELF SHA-256 (hex).
/// The SHA-256 is stamped into the image by the build, so it uniquely
/// identifies the exact firmware binary currently running.
fn firmware_id() -> String {
    let desc = unsafe { &*esp_idf_svc::sys::esp_app_get_description() };
    let version = unsafe { std::ffi::CStr::from_ptr(desc.version.as_ptr()) }.to_string_lossy();
    let sha: String = desc
        .app_elf_sha256
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    format!("FW: {version} sha256={sha}")
}

/// Debug path (USB-C trigger): capture both cameras exactly like the SSTV command,
/// then stream both frames over the USB-C serial link, each tagged with its camera
/// name, for local/capture_frame_via_usb_c.sh to save.
fn capture_and_dump_usb(
    debug: &DebugChannel,
    rgb: Option<&mut RgbCamera>,
    thermal: Option<&mut ThermalCamera>,
) {
    let (rgb, ir) = capture_both(rgb, thermal);

    match rgb {
        Some(rgb) => {
            log::info!("USB-C: sending RGB frame...");
            if let Err(e) = debug.send_image("rgb", rgb.width(), rgb.height(), rgb) {
                log::error!("USB-C RGB frame send failed: {e}");
            }
        }
        None => log::warn!("RGB camera unavailable, skipping RGB frame"),
    }
    match ir {
        Some(ir) => {
            log::info!("USB-C: sending thermal frame...");
            if let Err(e) = debug.send_image("thermal", ir.width(), ir.height(), ir) {
                log::error!("USB-C thermal frame send failed: {e}");
            }
        }
        None => log::warn!("Thermal camera unavailable, skipping thermal frame"),
    }
    log::info!("USB-C: frames sent");
}

fn initialize_esp32() -> Result<Peripherals> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    Peripherals::take().map_err(|_| Error::Peripheral)
}

fn initialize_payload_link(peripherals: Peripherals) -> Result<PayloadLink> {
    let link = PayloadLink::try_new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        peripherals.pins.gpio39,
    )?;

    link.send(Message::Booted(firmware_id()));

    Ok(link)
}

fn main() {
    let peripherals = initialize_esp32().unwrap();
    let mut link = initialize_payload_link(peripherals).unwrap();

    let mut rgb = RgbCamera::try_new(SC850SL, MIPI).report_if_err(&link);
    let mut thermal = ThermalCamera::try_new().report_if_err(&link);
    let mut audio = AudioChannel::try_new(PCM5102A, PHILLIPS_I2S).report_if_err(&link);

    let mut debug = DebugChannel::new();

    link.send(Message::Available);

    loop {
        if debug.poll_trigger() {
            log::info!("USB-C: capture trigger received");
            capture_and_dump_usb(&debug, rgb.as_mut().ok(), thermal.as_mut().ok());
        }

        for cmd in link.poll() {
            match cmd {
                Command::Sstv => {
                    link.send(Message::Busy);
                    let _ = transmit_sstv(
                        &mut link,
                        rgb.as_mut().ok(),
                        thermal.as_mut().ok(),
                        audio.as_mut().ok(),
                    )
                    .report_if_err(&link);
                    link.send(Message::Available);
                }
            }
        }

        delay::FreeRtos::delay_ms(1);
    }
}
