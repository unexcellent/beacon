//! KISS framing and the CSP-over-KISS interface that rides the UART link.

use std::sync::Mutex;

const FEND: u8 = 0xC0;
const FESC: u8 = 0xDB;
const TFEND: u8 = 0xDC;
const TFESC: u8 = 0xDD;

const CSP_FLAG_CRC32: u8 = 0x10;

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

pub struct KissDecoder {
    state: KissState,
    buf: Vec<u8>,
}

impl KissDecoder {
    pub fn new() -> Self {
        Self {
            state: KissState::Idle,
            buf: Vec::new(),
        }
    }

    pub fn push(&mut self, b: u8) -> Option<Vec<u8>> {
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

/// Packets queued by nexthop() for transmission; drained via [`take_tx`].
static TX_BUF: Mutex<Vec<u8>> = Mutex::new(Vec::new());

/// Drain the KISS TX bytes queued by [`UartKissIface::nexthop`].
pub fn take_tx() -> Vec<u8> {
    std::mem::take(&mut *TX_BUF.lock().unwrap())
}

pub struct UartKissIface;

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
pub fn frame_to_packet(frame: &[u8]) -> Option<libcsp::Packet> {
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
