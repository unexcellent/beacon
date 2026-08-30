use beacon::audio::move_iiia::initialize_audio_channel;
use beacon::camera::Camera;
use beacon::camera::move_iiia::{initialize_rgb_camera, initialize_thermal_camera};
use beacon::error::{Error, ReportIfErr, Result};
use beacon::idle::idle;
use beacon::link::move_iiia::initialize_payload_link as bring_up_payload_link;
use beacon::link::{CommandLink, Message};

use esp_idf_hal::peripherals::Peripherals;

fn main() {
    initialize_esp32();
    let mut link = initialize_payload_link().unwrap();
    let mut audio = initialize_audio_channel().report_if_err(&link).unwrap();

    let cameras = vec![
        initialize_rgb_camera().report_if_err(&link).ok().map(boxed),
        initialize_thermal_camera()
            .report_if_err(&link)
            .ok()
            .map(boxed),
    ];

    link.send(Message::Available);
    idle(&mut link, cameras, &mut audio);
}

fn initialize_esp32() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
}

fn initialize_payload_link() -> Result<impl CommandLink> {
    let link = bring_up_payload_link(Peripherals::take().map_err(|_| Error::Peripheral)?)?;

    let description = unsafe { &*esp_idf_svc::sys::esp_app_get_description() };
    let version =
        unsafe { std::ffi::CStr::from_ptr(description.version.as_ptr()) }.to_string_lossy();
    link.send(Message::Booted(format!("{version}")));

    Ok(link)
}

/// Erase a camera's concrete type so different cameras share one collection.
fn boxed<C: Camera + 'static>(camera: C) -> Box<dyn Camera> {
    Box::new(camera)
}
