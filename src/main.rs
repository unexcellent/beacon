use std::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use esp_idf_sys::{
    esp,
    esp_timer_get_time,
    gpio_config,
    gpio_config_t,
    gpio_get_level,
    gpio_install_isr_service,
    gpio_int_type_t_GPIO_INTR_ANYEDGE as GPIO_INTR_ANYEDGE,
    gpio_isr_handler_add,
    gpio_mode_t_GPIO_MODE_INPUT as GPIO_INPUT,
    gpio_mode_t_GPIO_MODE_OUTPUT as GPIO_OUTPUT,
    gpio_set_level,
    vTaskDelay,
};

const RX_PIN: i32 = 37;
const DE_PIN: i32 = 38;
const BAUD_RATE: u32 = 9600;
const BIT_PERIOD_US: u32 = 1_000_000 / BAUD_RATE; // 104 µs

const RING_SIZE: usize = 128; // must be power of 2
const RING_MASK: usize = RING_SIZE - 1;

static RING_TS: [AtomicU32; RING_SIZE] = {
    const Z: AtomicU32 = AtomicU32::new(0);
    [Z; RING_SIZE]
};
static RING_LVL: [AtomicU8; RING_SIZE] = {
    const Z: AtomicU8 = AtomicU8::new(0);
    [Z; RING_SIZE]
};
static RING_WR: AtomicUsize = AtomicUsize::new(0);
static RING_RD: AtomicUsize = AtomicUsize::new(0);

unsafe extern "C" fn on_edge(_arg: *mut core::ffi::c_void) {
    let now = unsafe { esp_timer_get_time() } as u32;
    let level = unsafe { gpio_get_level(RX_PIN) } as u8;
    let wr = RING_WR.load(Ordering::Relaxed);
    let next_wr = (wr + 1) & RING_MASK;
    // Drop edge if buffer is full
    if next_wr != RING_RD.load(Ordering::Relaxed) {
        RING_TS[wr].store(now, Ordering::Relaxed);
        RING_LVL[wr].store(level, Ordering::Relaxed);
        RING_WR.store(next_wr, Ordering::Release);
    }
}

/// Software UART decoder (8N1, idle-high).
///
/// Reconstructs bytes from edge timestamps by counting how many bit periods
/// elapsed at each level between successive edges.
struct Decoder {
    framing: bool,
    start_us: u32,
    last_edge_us: u32,
    last_level: u8,
    bit_pos: u8,
    data: u8,
}

impl Decoder {
    const fn new() -> Self {
        Self {
            framing: false,
            start_us: 0,
            last_edge_us: 0,
            last_level: 1,
            bit_pos: 0,
            data: 0,
        }
    }

    fn on_edge(&mut self, ts: u32, level: u8) -> Option<u8> {
        if !self.framing {
            if level == 0 {
                self.begin_frame(ts);
            }
            return None;
        }
        let delta = ts.wrapping_sub(self.last_edge_us);
        let n = ((delta + BIT_PERIOD_US / 2) / BIT_PERIOD_US).max(1) as u8;
        let decoded = self.consume_bits(self.last_level, n);
        if !self.framing && level == 0 {
            // Stop bit ended and next start bit arrived simultaneously
            self.begin_frame(ts);
        } else if self.framing {
            self.last_edge_us = ts;
            self.last_level = level;
        }
        decoded
    }

    /// Force-flush a partial frame if no edge has arrived for >12 bit periods.
    fn check_timeout(&mut self, now: u32) -> Option<u8> {
        if !self.framing {
            return None;
        }
        if now.wrapping_sub(self.start_us) > 12 * BIT_PERIOD_US {
            let delta = now.wrapping_sub(self.last_edge_us);
            let n = ((delta + BIT_PERIOD_US / 2) / BIT_PERIOD_US).max(1) as u8;
            let result = self.consume_bits(self.last_level, n);
            self.framing = false;
            result
        } else {
            None
        }
    }

    fn begin_frame(&mut self, ts: u32) {
        self.framing = true;
        self.start_us = ts;
        self.last_edge_us = ts;
        self.last_level = 0;
        self.bit_pos = 0;
        self.data = 0;
    }

    /// Push `n` bits at `level` into the frame, starting at `bit_pos`.
    ///
    /// Bit layout: 0=start, 1–8=data LSB-first, 9=stop.
    fn consume_bits(&mut self, level: u8, n: u8) -> Option<u8> {
        for _ in 0..n {
            match self.bit_pos {
                0 => {
                    if level != 0 {
                        self.framing = false;
                        return None;
                    }
                }
                b @ 1..=8 => {
                    if level == 1 {
                        self.data |= 1 << (b - 1);
                    }
                }
                9 => {
                    self.framing = false;
                    if level == 1 {
                        return Some(self.data);
                    }
                    return None; // framing error: missing stop bit
                }
                _ => {
                    self.framing = false;
                    return None;
                }
            }
            self.bit_pos += 1;
        }
        None
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    unsafe {
        let out_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << DE_PIN,
            mode: GPIO_OUTPUT,
            pull_up_en: 0,
            pull_down_en: 0,
            intr_type: 0,
            ..Default::default()
        };
        esp!(gpio_config(&out_cfg)).unwrap();
        esp!(gpio_set_level(DE_PIN, 0)).unwrap();

        let in_cfg = gpio_config_t {
            pin_bit_mask: 1u64 << RX_PIN,
            mode: GPIO_INPUT,
            pull_up_en: 0,
            pull_down_en: 0,
            intr_type: GPIO_INTR_ANYEDGE,
            ..Default::default()
        };
        esp!(gpio_config(&in_cfg)).unwrap();
        esp!(gpio_install_isr_service(0)).unwrap();
        esp!(gpio_isr_handler_add(
            RX_PIN,
            Some(on_edge),
            core::ptr::null_mut()
        ))
        .unwrap();
    }

    log::info!(
        "Listening for UART at {} baud on GPIO {} (DE on GPIO {} held LOW)",
        BAUD_RATE,
        RX_PIN,
        DE_PIN,
    );

    let mut dec = Decoder::new();
    let mut buf: Vec<u8> = Vec::new();

    loop {
        // Drain the ring buffer
        loop {
            let rd = RING_RD.load(Ordering::Relaxed);
            let wr = RING_WR.load(Ordering::Acquire);
            if rd == wr {
                break;
            }
            let ts = RING_TS[rd].load(Ordering::Relaxed);
            let lvl = RING_LVL[rd].load(Ordering::Relaxed);
            RING_RD.store((rd + 1) & RING_MASK, Ordering::Relaxed);
            if let Some(b) = dec.on_edge(ts, lvl) {
                buf.push(b);
            }
        }

        // Flush stop bit when no trailing edge arrives
        let now = unsafe { esp_timer_get_time() as u32 };
        if let Some(b) = dec.check_timeout(now) {
            buf.push(b);
        }

        // Print once the line goes idle
        if !dec.framing && !buf.is_empty() {
            match core::str::from_utf8(&buf) {
                Ok(s) => log::info!("RX: {:?}", s),
                Err(_) => log::info!("RX: {:02x?}", buf),
            }
            buf.clear();
        }

        unsafe { vTaskDelay(1) };
    }
}
