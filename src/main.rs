mod ota;

use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, Output, PinDriver},
    peripherals::Peripherals,
    uart::{self, UartDriver},
    units::Hertz,
};

const BAUD_RATE: u32 = 115_200;
const NODE: u16 = 7;
const PING_PORT: u8 = 1;
const OTA_PORT:  u8 = 10;

// ─── KISS ────────────────────────────────────────────────────────────────────

const FEND:  u8 = 0xC0;
const FESC:  u8 = 0xDB;
const TFEND: u8 = 0xDC;
const TFESC: u8 = 0xDD;

fn kiss_encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 4);
    out.push(FEND);
    out.push(0x00);
    for &b in data {
        match b {
            FEND => { out.push(FESC); out.push(TFEND); }
            FESC => { out.push(FESC); out.push(TFESC); }
            _    => out.push(b),
        }
    }
    out.push(FEND);
    out
}

enum KissState { Idle, Command, Data, Escape }

struct KissDecoder { state: KissState, buf: Vec<u8> }

impl KissDecoder {
    fn new() -> Self { Self { state: KissState::Idle, buf: Vec::new() } }

    fn push(&mut self, b: u8) -> Option<Vec<u8>> {
        match self.state {
            KissState::Idle => {
                if b == FEND { self.state = KissState::Command; }
                None
            }
            KissState::Command => match b {
                FEND => None,
                0x00 => { self.buf.clear(); self.state = KissState::Data; None }
                _    => { self.state = KissState::Idle; None }
            },
            KissState::Data => match b {
                FEND => {
                    if self.buf.is_empty() { return None; }
                    let frame = self.buf.clone();
                    self.buf.clear();
                    self.state = KissState::Command;
                    Some(frame)
                }
                FESC => { self.state = KissState::Escape; None }
                _    => { self.buf.push(b); None }
            },
            KissState::Escape => {
                let decoded = match b {
                    TFEND => FEND,
                    TFESC => FESC,
                    _     => { self.state = KissState::Idle; return None; }
                };
                self.buf.push(decoded);
                self.state = KissState::Data;
                None
            }
        }
    }
}

// ─── CSP helpers ─────────────────────────────────────────────────────────────

fn fmt_payload(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return "(empty)".into();
    }
    if bytes.len() > 32 {
        return format!("({} bytes)", bytes.len());
    }
    if let Ok(s) = core::str::from_utf8(bytes) {
        if s.bytes().all(|b| b >= 0x20 && b < 0x7f) {
            return format!("\"{}\"", s);
        }
    }
    bytes.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(" ")
}

fn send_frame(uart: &UartDriver, de: &mut PinDriver<'_, Output>, frame: &[u8]) {
    de.set_high().unwrap();
    let mut sent = 0;
    while sent < frame.len() {
        sent += uart.write(&frame[sent..]).unwrap();
    }
    uart.flush_write().unwrap();
    delay::FreeRtos::delay_ms(1);
    de.set_low().unwrap();
}

fn handle_csp(
    frame: &[u8],
    uart: &UartDriver,
    de: &mut PinDriver<'_, Output>,
    ota: &mut ota::OtaState,
) {
    if frame.len() < 4 {
        log::warn!("CSP: frame too short ({} bytes)", frame.len());
        return;
    }

    let word  = u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]);
    let src   = ((word >> 25) & 0x1F) as u16;
    let dst   = ((word >> 20) & 0x1F) as u16;
    let dport = ((word >> 14) & 0x3F) as u8;
    let sport = ((word >>  8) & 0x3F) as u8;
    let flags = (word & 0xFF) as u8;
    let payload = &frame[4..];

    log::info!("[RX] from {}:{} to {}:{} is {} (flags 0x{:02x})",
        src, sport, dst, dport, fmt_payload(payload), flags);

    if dst != NODE { return; }

    if dport == PING_PORT {
        let pong_word: u32 = (2u32 << 30)
            | ((NODE  as u32) << 25)
            | ((src   as u32) << 20)
            | ((sport as u32) << 14)
            | ((PING_PORT as u32) << 8);
        let mut pong = pong_word.to_be_bytes().to_vec();
        pong.extend_from_slice(payload);
        send_frame(uart, de, &kiss_encode(&pong));
        log::info!("[TX] pong to {}:{}", src, sport);
    } else if dport == OTA_PORT {
        ota.handle(payload);
    }
}

// ─── Entry point ─────────────────────────────────────────────────────────────

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mut de = PinDriver::output(peripherals.pins.gpio39).unwrap();
    de.set_low().unwrap();

    let config = uart::config::Config::new().baudrate(Hertz(BAUD_RATE));

    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio38,
        peripherals.pins.gpio37,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    )
    .unwrap();

    log::info!("CSP node {} at {} baud (RX=G37, TX=G38, DE=G39)", NODE, BAUD_RATE);

    let mut kiss = KissDecoder::new();
    let mut ota  = ota::OtaState::new();
    let mut buf  = [0u8; 256];

    loop {
        if let Ok(n) = uart.read(&mut buf, delay::BLOCK) {
            for &b in &buf[..n] {
                if let Some(frame) = kiss.push(b) {
                    handle_csp(&frame, &uart, &mut de, &mut ota);
                }
            }
        }
    }
}
