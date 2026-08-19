mod audio;
mod camera;
mod csp;
mod csp_arch;
mod debug;
mod kiss;
mod ota;
mod uart;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use camera::{Camera, Image, MIPI, RgbCamera, SC850SL, ThermalCamera};
use csp::Csp;
use debug::DebugChannel;
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};
use uart::Uart;

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
fn transmit_sstv<I>(image: I, audio: &mut AudioChannel) -> Result<(), Box<dyn std::error::Error>>
where
    I: Iterator<Item = RgbPixel> + 'static,
{
    log::info!("SSTV: encoding and transmitting...");
    let encoder = Encoder::new(Mode::Robot36, image)?;
    for sample in Synthesizer::new(encoder, PHILLIPS_I2S.sample_rate) {
        audio.transmit(sample)?;
    }
    audio.flush()?;
    log::info!("SSTV: transmission complete");
    Ok(())
}

/// Frames each camera discards to settle before the kept capture. The RGB sensor just
/// needs its stream to stabilise after power-on; the thermal sensor needs enough
/// single-shot frames for its on-chip temporal/median filters to converge.
const RGB_WARMUP_FRAMES: u32 = 2;
const THERMAL_WARMUP_FRAMES: u32 = 5;

/// Power a camera on, let it settle for `warmup` frames, capture one frame, then power it
/// back off. The shared [`Camera`] trait lets both cameras run the exact same sequence.
fn shoot(camera: &mut dyn Camera, warmup: u32) -> Image {
    camera.power_on();
    camera.calibrate(warmup);
    let image = camera.capture();
    camera.power_off();
    image
}

/// Capture one frame with each camera as close together in time as the single-threaded
/// flow allows. Returns (RGB, infrared).
fn capture_both(camera: &mut RgbCamera, thermal: &mut ThermalCamera) -> (Image, Image) {
    log::info!("RGB camera: activating + calibrating...");
    let rgb = shoot(camera, RGB_WARMUP_FRAMES);
    log::info!("Thermal camera: capturing (averaged)...");
    let ir = shoot(thermal, THERMAL_WARMUP_FRAMES);
    log::info!("Both frames captured");
    (rgb, ir)
}

/// Handle one SSTV command: capture both cameras, then transmit the RGB image,
/// wait 5 s, and transmit the infrared image. Each transmission is framed by its own
/// BUSY→AVAILABLE status, so the gap between RGB and IR reports AVAILABLE.
fn capture_and_transmit_both(
    uart: &Uart,
    csp: &Csp,
    camera: &mut RgbCamera,
    thermal: &mut ThermalCamera,
    audio: &mut AudioChannel,
) {
    csp.send_status(uart, b"BUSY");

    let (rgb, ir) = capture_both(camera, thermal);

    log::info!("SSTV: transmitting RGB image...");
    if let Err(e) = transmit_sstv(rgb, audio) {
        log::error!("RGB SSTV transmission failed: {e}");
    }

    // Signal AVAILABLE during the gap between the two transmissions, then BUSY again
    // before the infrared one so the status tracks each individual transmission.
    csp.send_status(uart, b"AVAILABLE");
    log::info!("Waiting 5 s before the infrared transmission...");
    delay::FreeRtos::delay_ms(5_000);
    csp.send_status(uart, b"BUSY");

    log::info!("SSTV: transmitting infrared image...");
    if let Err(e) = transmit_sstv(ir, audio) {
        log::error!("IR SSTV transmission failed: {e}");
    }

    csp.send_status(uart, b"AVAILABLE");
    log::info!("Both images sent, waiting for commands");
}

/// Debug path (USB-C trigger): capture both cameras exactly like the SSTV command,
/// then stream both frames over the USB-C serial link, each tagged with its camera
/// name, for local/capture_frame_via_usb_c.sh to save.
fn capture_and_dump_usb(debug: &DebugChannel, camera: &mut RgbCamera, thermal: &mut ThermalCamera) {
    let (rgb, ir) = capture_both(camera, thermal);

    log::info!("USB-C: sending RGB frame...");
    if let Err(e) = debug.send_image("rgb", rgb.width(), rgb.height(), rgb) {
        log::error!("USB-C RGB frame send failed: {e}");
    }
    log::info!("USB-C: sending thermal frame...");
    if let Err(e) = debug.send_image("thermal", ir.width(), ir.height(), ir) {
        log::error!("USB-C thermal frame send failed: {e}");
    }
    log::info!("USB-C: both frames sent");
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let uart = Uart::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        peripherals.pins.gpio39,
    );

    let csp = Csp::init();

    log::info!(
        "CSP node {} ready at {} baud (RX=G37 TX=G38 DE=G39)",
        csp::NODE,
        uart::BAUD_RATE
    );

    // Announce the boot to the OBC and report which firmware is running, so the
    // ground can confirm the reboot and validate the deployed image (e.g. after OTA).
    let fw = firmware_id();
    log::info!("Boot: reporting to node {} ({fw})", csp::OBC_NODE);
    csp.send_msg(&uart, csp::OBC_NODE, csp::OBC_PORT, b"STATUS: BOOTED");
    csp.send_msg(&uart, csp::OBC_NODE, csp::OBC_PORT, fw.as_bytes());

    log::info!("RGB camera: initializing...");
    let mut camera = RgbCamera::new(SC850SL, MIPI).expect("camera init");
    log::info!("RGB camera: ready (standby)");

    log::info!("Thermal camera: initializing (MI1602 via MI48Dx)...");
    let mut thermal = ThermalCamera::new().expect("thermal camera init");
    log::info!("Thermal camera: ready");

    let mut audio = AudioChannel::new(PCM5102A, PHILLIPS_I2S).expect("audio init");

    // USB-C debug link: lets a host trigger a two-camera capture and receive both
    // frames over the serial console (see local/capture_frame_via_usb_c.sh).
    let mut debug = DebugChannel::new();

    csp.send_status(&uart, b"AVAILABLE");
    log::info!("Sent AVAILABLE, waiting for commands");

    let mut decoder = kiss::KissDecoder::new();
    let mut ota = ota::OtaState::new();
    let mut buf = [0u8; 512];
    // Poll the RS485 read with a short timeout (rather than blocking forever) so the
    // loop also gets to service the USB-C debug trigger each iteration.
    let read_timeout = delay::TickType::new_millis(100).ticks();

    loop {
        if debug.poll_trigger() {
            log::info!("USB-C: capture trigger received");
            capture_and_dump_usb(&debug, &mut camera, &mut thermal);
        }

        if let Ok(n) = uart.read(&mut buf, read_timeout) {
            for &b in &buf[..n] {
                if let Some(frame) = decoder.push(b) {
                    if let Some(pkt) = kiss::frame_to_packet(&frame) {
                        if !matches!(ota, ota::OtaState::Writing(_)) {
                            let id = pkt.id();
                            let (src, sport, dst, dport, flags) =
                                (id.src, id.sport, id.dst, id.dport, id.flags);
                            log::info!(
                                "[UART RX] from {}:{} to {}:{} is {} (flags 0x{:02x}, len={})",
                                src,
                                sport,
                                dst,
                                dport,
                                kiss::fmt_payload(pkt.data()),
                                flags,
                                pkt.data().len()
                            );
                        }
                        csp.iface.rx(pkt);
                    }
                }
            }
        }

        let _ = csp.node.route_work();

        while let Some(conn) = csp.ping_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                conn.handle_service(pkt);
            }
        }

        while let Some(conn) = csp.ota_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                ota.handle(pkt.data());
            }
        }

        while let Some(conn) = csp.cmd_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if pkt.data().starts_with(b"SSTV") {
                    log::info!("CMD: SSTV requested");
                    capture_and_transmit_both(&uart, &csp, &mut camera, &mut thermal, &mut audio);
                } else {
                    log::warn!("CMD: unknown payload: {}", kiss::fmt_payload(pkt.data()));
                }
            }
        }

        let to_send = kiss::take_tx();
        if !to_send.is_empty() {
            uart.send(&to_send);
        }

        delay::FreeRtos::delay_ms(1);
    }
}
