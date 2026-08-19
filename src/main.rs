mod audio;
mod camera;
mod csp_arch;
mod debug;
mod error;
mod kiss;
mod link;
mod ota;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use camera::{MIPI, RgbCamera, SC850SL, ThermalCamera, capture_both};
use debug::DebugChannel;
use error::{Error, ReportIfErr, Result};
use link::{Command, Message, PayloadLink};
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};

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

// ── SSTV helpers ──────────────────────────────────────────────────────────────

/// Encode any RGB pixel stream (RGB camera or false-coloured thermal) as Robot36
/// SSTV and play it out over the audio channel.
fn transmit_sstv<I>(image: I, audio: &mut AudioChannel) -> Result<()>
where
    I: Iterator<Item = RgbPixel> + 'static,
{
    log::info!("SSTV: encoding and transmitting...");
    let encoder = Encoder::new(Mode::Robot36, image).unwrap();
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        audio.transmit(sample).map_err(|_| Error::Peripheral)?;
    }
    audio.flush().map_err(|_| Error::Peripheral)?;
    log::info!("SSTV: transmission complete");
    Ok(())
}

/// Handle one SSTV command: capture both cameras, then transmit the RGB image,
/// wait 5 s, and transmit the infrared image. Each transmission is framed by its own
/// BUSY→AVAILABLE status, so the gap between RGB and IR reports AVAILABLE.
fn capture_and_transmit_both(
    link: &PayloadLink,
    rgb: Option<&mut RgbCamera>,
    thermal: Option<&mut ThermalCamera>,
    audio: Option<&mut AudioChannel>,
) {
    link.send(Message::Busy);

    // Without the audio channel nothing can be transmitted, so skip the captures too.
    let Some(audio) = audio else {
        log::warn!("Audio channel unavailable, skipping SSTV transmission");
        link.send(Message::Available);
        return;
    };

    let (rgb, ir) = capture_both(rgb, thermal);
    let have_both = rgb.is_some() && ir.is_some();

    match rgb {
        Some(rgb) => {
            log::info!("SSTV: transmitting RGB image...");
            if let Err(e) = transmit_sstv(rgb, audio) {
                log::error!("RGB SSTV transmission failed: {e}");
            }

            if have_both {
                // Signal AVAILABLE during the gap between the two transmissions, then BUSY
                // again before the infrared one so the status tracks each transmission.
                link.send(Message::Available);
                log::info!("Waiting 5 s before the infrared transmission...");
                delay::FreeRtos::delay_ms(5_000);
                link.send(Message::Busy);
            }
        }
        None => log::warn!("RGB camera unavailable, skipping RGB transmission"),
    }

    match ir {
        Some(ir) => {
            log::info!("SSTV: transmitting infrared image...");
            if let Err(e) = transmit_sstv(ir, audio) {
                log::error!("IR SSTV transmission failed: {e}");
            }
        }
        None => log::warn!("Thermal camera unavailable, skipping infrared transmission"),
    }

    link.send(Message::Available);
    log::info!("Transmission finished");
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
                Command::Sstv => capture_and_transmit_both(
                    &link,
                    rgb.as_mut().ok(),
                    thermal.as_mut().ok(),
                    audio.as_mut().ok(),
                ),
            }
        }

        delay::FreeRtos::delay_ms(1);
    }
}
