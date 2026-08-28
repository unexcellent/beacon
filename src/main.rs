mod cameras;
mod devices;
mod error;
mod transmit_sstv;
mod update;
mod watermark;

use cameras::{initialize_rgb_camera, initialize_thermal_camera};
use devices::audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use devices::link::{Command, Message, PayloadLink};
use error::{Error, ReportIfErr, Result};
use transmit_sstv::transmit_sstv;
use update::update;

use esp_idf_hal::peripherals::Peripherals;

fn main() {
    let peripherals = initialize_esp32().unwrap();
    let mut link = initialize_payload_link(peripherals).unwrap();
    let mut audio = initialize_audio_channel().report_if_err(&link).unwrap();
    let mut rgb = initialize_rgb_camera().report_if_err(&link);
    let mut thermal = initialize_thermal_camera().report_if_err(&link);

    link.send(Message::Available);

    loop {
        match link.receive().report_if_err(&link) {
            Ok(Some(Command::Sstv)) => {
                link.send(Message::Busy);
                let _ = transmit_sstv(rgb.as_mut().ok(), thermal.as_mut().ok(), &mut audio)
                    .report_if_err(&link);
                link.send(Message::Available);
            }
            Ok(Some(Command::UpdateAnnounced(chunk_size))) => {
                let _ = update(chunk_size, &mut link).report_if_err(&link);
            }
            _ => (),
        }
    }
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

    let description = unsafe { &*esp_idf_svc::sys::esp_app_get_description() };
    let version =
        unsafe { std::ffi::CStr::from_ptr(description.version.as_ptr()) }.to_string_lossy();
    link.send(Message::Booted(format!("{version}")));

    Ok(link)
}

fn initialize_audio_channel() -> Result<AudioChannel> {
    AudioChannel::try_new(PCM5102A, PHILLIPS_I2S)
}
