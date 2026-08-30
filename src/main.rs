mod board;

use beacon::camera::Camera;
use beacon::error::{Error, ReportIfErr, Result};
use beacon::idle::idle;
use beacon::link::{CommandLink, Message};
use board::{
    Rs422Link, initialize_audio_channel, initialize_rgb_camera, initialize_thermal_camera,
};

use esp_idf_hal::peripherals::Peripherals;

fn main() {
    let peripherals = initialize_esp32().unwrap();
    let mut link = initialize_payload_link(peripherals).unwrap();
    let mut audio = initialize_audio_channel().report_if_err(&link).unwrap();

    let cameras = vec![
        initialize_rgb_camera().report_if_err(&link).ok().map(boxed),
        initialize_thermal_camera().report_if_err(&link).ok().map(boxed),
    ];

    link.send(Message::Available);

    idle(&mut link, cameras, &mut audio);
}

/// Erase a camera's concrete type so different cameras share one collection.
fn boxed<C: Camera + 'static>(camera: C) -> Box<dyn Camera> {
    Box::new(camera)
}

fn initialize_esp32() -> Result<Peripherals> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    Peripherals::take().map_err(|_| Error::Peripheral)
}

fn initialize_payload_link(peripherals: Peripherals) -> Result<Rs422Link> {
    let link = board::initialize_payload_link(peripherals)?;

    let description = unsafe { &*esp_idf_svc::sys::esp_app_get_description() };
    let version =
        unsafe { std::ffi::CStr::from_ptr(description.version.as_ptr()) }.to_string_lossy();
    link.send(Message::Booted(format!("{version}")));

    Ok(link)
}
