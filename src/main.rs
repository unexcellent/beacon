mod audio;
#[allow(dead_code, unused_imports)] // retained for the RGB camera path; not used in the IR workflow
mod camera;
mod csp_arch;
mod ota;
mod thermal;

use audio::{AudioChannel, PCM5102A, PHILLIPS_I2S};
use sstv::{Encoder, Mode, RgbPixel, Synthesizer};
use thermal::ThermalCamera;

use esp_idf_hal::{
    delay,
    gpio::PinDriver,
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};
use std::sync::Mutex;

const BAUD_RATE: u32 = 115_200;
const NODE: u16 = 7;
const PING_PORT: u8 = 1;
const OTA_PORT: u8 = 10;
const CMD_PORT: u8 = 11;
const PAYLOAD_NODE: u16 = 14;
const PAYLOAD_PORT: u8 = 1;

// ── KISS ──────────────────────────────────────────────────────────────────────

const FEND: u8 = 0xC0;
const FESC: u8 = 0xDB;
const TFEND: u8 = 0xDC;
const TFESC: u8 = 0xDD;

fn kiss_encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 4);
    out.push(FEND);
    out.push(0x00);
    for &b in data {
        match b {
            FEND => {
                out.push(FESC);
                out.push(TFEND);
            }
            FESC => {
                out.push(FESC);
                out.push(TFESC);
            }
            _ => out.push(b),
        }
    }
    out.push(FEND);
    out
}

enum KissState {
    Idle,
    Command,
    Data,
    Escape,
}

struct KissDecoder {
    state: KissState,
    buf: Vec<u8>,
}

impl KissDecoder {
    fn new() -> Self {
        Self {
            state: KissState::Idle,
            buf: Vec::new(),
        }
    }

    fn push(&mut self, b: u8) -> Option<Vec<u8>> {
        match self.state {
            KissState::Idle => {
                if b == FEND {
                    self.state = KissState::Command;
                }
                None
            }
            KissState::Command => match b {
                FEND => None,
                0x00 => {
                    self.buf.clear();
                    self.state = KissState::Data;
                    None
                }
                _ => {
                    self.state = KissState::Idle;
                    None
                }
            },
            KissState::Data => match b {
                FEND => {
                    if self.buf.is_empty() {
                        return None;
                    }
                    let frame = self.buf.clone();
                    self.buf.clear();
                    self.state = KissState::Command;
                    Some(frame)
                }
                FESC => {
                    self.state = KissState::Escape;
                    None
                }
                _ => {
                    self.buf.push(b);
                    None
                }
            },
            KissState::Escape => {
                let decoded = match b {
                    TFEND => FEND,
                    TFESC => FESC,
                    _ => {
                        self.state = KissState::Idle;
                        return None;
                    }
                };
                self.buf.push(decoded);
                self.state = KissState::Data;
                None
            }
        }
    }
}

// ── CSP helpers ───────────────────────────────────────────────────────────────

pub fn fmt_payload(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return "(empty)".into();
    }
    if bytes.len() > 32 {
        return format!("({} bytes)", bytes.len());
    }
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

const CSP_FLAG_CRC32: u8 = 0x10;

// ── CSP / KISS interface ──────────────────────────────────────────────────────

/// Packets queued by nexthop() for transmission; drained by the main loop.
static TX_BUF: Mutex<Vec<u8>> = Mutex::new(Vec::new());

struct UartKissIface;

impl libcsp::CspInterface for UartKissIface {
    fn nexthop(&mut self, _via: u16, packet: libcsp::Packet, from_me: bool) {
        if !from_me {
            return;
        }
        let id = packet.id();
        let (src, sport, dst, dport, flags) = (id.src, id.sport, id.dst, id.dport, id.flags);
        log::info!(
            "[UART TX] from {}:{} to {}:{} is {} (flags 0x{:02x})",
            src,
            sport,
            dst,
            dport,
            fmt_payload(packet.data()),
            flags
        );
        let word: u32 = ((id.pri as u32) << 30)
            | ((id.src as u32) << 25)
            | ((id.dst as u32) << 20)
            | ((id.dport as u32) << 14)
            | ((id.sport as u32) << 8)
            | (id.flags as u32);
        let mut raw = word.to_be_bytes().to_vec();
        raw.extend_from_slice(packet.data());
        TX_BUF.lock().unwrap().extend_from_slice(&kiss_encode(&raw));
    }

    fn name(&self) -> &str {
        "KISS"
    }
}

/// Decode a KISS payload into a CSP Packet, or None if it is too short.
fn frame_to_packet(frame: &[u8]) -> Option<libcsp::Packet> {
    if frame.len() < 4 {
        return None;
    }
    let word = u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]);
    let id = libcsp::sys::csp_id_t {
        pri: ((word >> 30) & 0x3) as u8,
        flags: (word & 0xFF) as u8,
        src: ((word >> 25) & 0x1F) as u16,
        dst: ((word >> 20) & 0x1F) as u16,
        dport: ((word >> 14) & 0x3F) as u8,
        sport: ((word >> 8) & 0x3F) as u8,
    };
    let mut pkt = libcsp::Packet::get(0)?;
    pkt.set_id(id);
    if frame.len() > 4 {
        let data = &frame[4..];
        let data = if id.flags & CSP_FLAG_CRC32 != 0 && data.len() >= 4 {
            &data[..data.len() - 4]
        } else {
            data
        };
        pkt.write(data).ok()?;
    }
    Some(pkt)
}

// ── UART helper ───────────────────────────────────────────────────────────────

fn send_bytes(uart: &UartDriver, data: &[u8]) {
    let mut sent = 0;
    while sent < data.len() {
        sent += uart.write(&data[sent..]).unwrap();
    }
    uart.wait_tx_done(delay::BLOCK).unwrap();
}

fn send_status(uart: &UartDriver, node: &libcsp::CspNode, msg: &[u8]) {
    if let Some(mut pkt) = libcsp::Packet::get(0) {
        pkt.write(msg).ok();
        node.sendto(libcsp::Priority::Norm, PAYLOAD_NODE, PAYLOAD_PORT, NODE as u8, 0, pkt);
    }
    let to_send: Vec<u8> = std::mem::take(&mut *TX_BUF.lock().unwrap());
    if !to_send.is_empty() {
        send_bytes(uart, &to_send);
    }
}

// ── SSTV helpers ──────────────────────────────────────────────────────────────

fn transmit_sstv<I>(
    image: I,
    audio: &mut AudioChannel,
) -> Result<(), Box<dyn std::error::Error>>
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

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut de = PinDriver::output(peripherals.pins.gpio39).unwrap();
    de.set_high().unwrap(); // RS422 full-duplex: TX driver always enabled
    let _de = de;

    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        &uart::config::Config::new()
            .baudrate(Hertz(BAUD_RATE))
            .rx_fifo_size(8192),
    )
    .unwrap();

    let node = libcsp::CspConfig::new()
        .address(NODE)
        .hostname("beacon")
        .model("esp32p4")
        .init()
        .expect("csp init");

    let iface = libcsp::interface::register(UartKissIface);
    unsafe { libcsp::route::set_default(iface.c_iface_ptr()).expect("default route") };

    let mut ping_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
    ping_sock.bind(PING_PORT).expect("bind ping");

    let mut ota_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
    ota_sock.bind(OTA_PORT).expect("bind ota");

    let mut cmd_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
    cmd_sock.bind(CMD_PORT).expect("bind cmd");

    log::info!(
        "CSP node {} ready at {} baud (RX=G37 TX=G38 DE=G39)",
        NODE,
        BAUD_RATE
    );

    log::info!("Thermal camera: initializing (MI1602 via MI48Dx)...");
    let mut camera = ThermalCamera::new().expect("thermal camera init");
    log::info!("Thermal camera: ready");

    let mut audio = AudioChannel::new(PCM5102A, PHILLIPS_I2S).expect("audio init");

    send_status(&uart, &node, b"AVAILABLE");
    log::info!("Sent AVAILABLE, waiting for commands");

    let mut kiss = KissDecoder::new();
    let mut ota = ota::OtaState::new();
    let mut buf = [0u8; 512];

    loop {
        if let Ok(n) = uart.read(&mut buf, delay::BLOCK) {
            for &b in &buf[..n] {
                if let Some(frame) = kiss.push(b) {
                    if let Some(pkt) = frame_to_packet(&frame) {
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
                                fmt_payload(pkt.data()),
                                flags,
                                pkt.data().len()
                            );
                        }
                        iface.rx(pkt);
                    }
                }
            }
        }

        let _ = node.route_work();

        while let Some(conn) = ping_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                conn.handle_service(pkt);
            }
        }

        while let Some(conn) = ota_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                ota.handle(pkt.data());
            }
        }

        while let Some(conn) = cmd_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if pkt.data().starts_with(b"SSTV") {
                    log::info!("CMD: SSTV requested");
                    send_status(&uart, &node, b"BUSY");

                    log::info!("Thermal camera: capturing...");
                    let image = camera.capture();
                    log::info!("Thermal camera: frame captured");

                    if let Err(e) = transmit_sstv(image, &mut audio) {
                        log::error!("SSTV transmission failed: {e}");
                    }

                    send_status(&uart, &node, b"AVAILABLE");
                    log::info!("Sent AVAILABLE, waiting for commands");
                } else {
                    log::warn!("CMD: unknown payload: {}", fmt_payload(pkt.data()));
                }
            }
        }

        let to_send: Vec<u8> = {
            let mut g = TX_BUF.lock().unwrap();
            std::mem::take(&mut *g)
        };
        if !to_send.is_empty() {
            send_bytes(&uart, &to_send);
        }

        delay::FreeRtos::delay_ms(1);
    }
}
