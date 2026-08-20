mod devices;
mod error;
mod transmit_sstv;
mod update;

use devices::audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use devices::camera::{MIPI, RgbCamera, SC850SL, ThermalCamera};
use devices::debug::{DebugChannel, transmit_usb};
use devices::link::{Command, Message, PayloadLink};
use error::{Error, ReportIfErr, Result};
use transmit_sstv::transmit_sstv;
use update::update;

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
        if debug.capture_has_been_triggered() {
            transmit_usb(&debug, rgb.as_mut().ok(), thermal.as_mut().ok());
        }

        for cmd in link.poll() {
            match cmd {
                Command::Sstv => {
                    link.send(Message::Busy);
                    let _ = transmit_sstv(
                        rgb.as_mut().ok(),
                        thermal.as_mut().ok(),
                        audio.as_mut().ok(),
                    )
                    .report_if_err(&link);
                    link.send(Message::Available);
                }
                Command::UpdateAnnounced(chunk_size) => {
                    let _ = update(chunk_size, &mut link).report_if_err(&link);
                }
                Command::UpdateBegin(_) | Command::UpdateData { .. } | Command::UpdateEnd => {
                    log::warn!("OTA: no update announced, ignoring");
                }
            }
        }

        delay::FreeRtos::delay_ms(1);
    }
}
