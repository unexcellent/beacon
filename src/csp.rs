use core::ffi::c_void;

use esp_idf_sys as esys;
use libcsp::arch::CspArch;
use libcsp::{export_arch, interface, socket_opts, CspConfig, CspInterface, CspNode, Packet, Socket};
use libcsp::sys as csp_sys;

pub const OTA_PORT: u8 = 10;
pub const BEACON_PORT: u8 = 9;

// ── CspArch backed by FreeRTOS (via esp_idf_sys) ────────────────────────────

struct EspArch;

unsafe impl CspArch for EspArch {
    fn get_ms(&self) -> u32 {
        (unsafe { esys::esp_timer_get_time() } / 1_000) as u32
    }

    fn get_s(&self) -> u32 {
        (unsafe { esys::esp_timer_get_time() } / 1_000_000) as u32
    }

    // Binary semaphore: counting semaphore with max=1, initial=0.
    fn bin_sem_create(&self) -> *mut c_void {
        unsafe { esys::xQueueCreateCountingSemaphore(1, 0) as *mut c_void }
    }
    fn bin_sem_remove(&self, sem: *mut c_void) {
        unsafe { esys::vQueueDelete(sem as esys::QueueHandle_t) }
    }
    fn bin_sem_wait(&self, sem: *mut c_void, timeout_ms: u32) -> bool {
        unsafe {
            esys::xQueueReceive(sem as esys::QueueHandle_t, core::ptr::null_mut(), timeout_ms as esys::TickType_t) != 0
        }
    }
    fn bin_sem_post(&self, sem: *mut c_void) -> bool {
        unsafe {
            esys::xQueueGenericSend(sem as esys::QueueHandle_t, core::ptr::null(), 0, 0) != 0
        }
    }

    // Mutex (queueQUEUE_TYPE_MUTEX = 1).
    fn mutex_create(&self) -> *mut c_void {
        unsafe { esys::xQueueCreateMutex(1) as *mut c_void }
    }
    fn mutex_remove(&self, mutex: *mut c_void) {
        unsafe { esys::vQueueDelete(mutex as esys::QueueHandle_t) }
    }
    fn mutex_lock(&self, mutex: *mut c_void, timeout_ms: u32) -> bool {
        unsafe {
            esys::xQueueReceive(mutex as esys::QueueHandle_t, core::ptr::null_mut(), timeout_ms as esys::TickType_t) != 0
        }
    }
    fn mutex_unlock(&self, mutex: *mut c_void) -> bool {
        unsafe {
            esys::xQueueGenericSend(mutex as esys::QueueHandle_t, core::ptr::null(), 0, 0) != 0
        }
    }

    // Queue (queueQUEUE_TYPE_BASE = 0).
    fn queue_create(&self, length: usize, item_size: usize) -> *mut c_void {
        unsafe {
            esys::xQueueGenericCreate(
                length as esys::UBaseType_t,
                item_size as esys::UBaseType_t,
                0,
            ) as *mut c_void
        }
    }
    fn queue_remove(&self, queue: *mut c_void) {
        unsafe { esys::vQueueDelete(queue as esys::QueueHandle_t) }
    }
    fn queue_enqueue(&self, queue: *mut c_void, item: *const c_void, timeout_ms: u32) -> bool {
        unsafe {
            esys::xQueueGenericSend(
                queue as esys::QueueHandle_t,
                item,
                timeout_ms as esys::TickType_t,
                0, // queueSEND_TO_BACK
            ) != 0
        }
    }
    fn queue_dequeue(&self, queue: *mut c_void, item: *mut c_void, timeout_ms: u32) -> bool {
        unsafe {
            esys::xQueueReceive(
                queue as esys::QueueHandle_t,
                item,
                timeout_ms as esys::TickType_t,
            ) != 0
        }
    }
    fn queue_size(&self, queue: *mut c_void) -> usize {
        unsafe { esys::uxQueueMessagesWaiting(queue as esys::QueueHandle_t) as usize }
    }
}

static ARCH: EspArch = EspArch;
export_arch!(EspArch, ARCH);

// ── Custom CSP interface over UART+KISS ──────────────────────────────────────

struct UartIface;

impl CspInterface for UartIface {
    fn nexthop(&mut self, _via: u16, _packet: Packet, _from_me: bool) {}
    fn name(&self) -> &str {
        "UART"
    }
}

// ── Public CSP stack ─────────────────────────────────────────────────────────

pub struct CspStack {
    node: CspNode,
    handle: interface::InterfaceHandle,
    socket: Socket,
}

impl CspStack {
    pub fn new() -> Self {
        let node = CspConfig::new()
            .address(7)
            .init()
            .expect("CSP init failed");

        let handle = interface::register(UartIface);

        let mut socket = Socket::new(socket_opts::CONN_LESS);
        socket.bind(OTA_PORT).expect("CSP socket bind failed");

        Self { node, handle, socket }
    }

    /// Parse a raw KISS frame (CSP v1 header + payload) and hand it to the router.
    pub fn inject(&self, frame: &[u8]) {
        if frame.len() < 4 {
            log::warn!("CSP: frame too short ({} bytes)", frame.len());
            return;
        }

        let word = u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]);
        let payload = &frame[4..];

        let Some(mut pkt) = Packet::get(payload.len().max(1)) else {
            log::warn!("CSP: buffer pool exhausted (payload {} bytes)", payload.len());
            return;
        };

        if !payload.is_empty() {
            if pkt.write(payload).is_err() {
                log::warn!("CSP: payload ({} bytes) exceeds buffer; drop", payload.len());
                return;
            }
        }

        // Build csp_id_t from the CSP v1 32-bit big-endian header.
        // Layout: priority[31:30] src[29:25] dst[24:20] dport[19:14] sport[13:8] flags[7:0]
        // csp_id_t layout: pri:u8, flags:u8, src:u16, dst:u16, dport:u8, sport:u8
        let mut id: csp_sys::csp_id_t = unsafe { core::mem::zeroed() };
        id.pri   = ((word >> 30) & 0x03) as u8;
        id.src   = ((word >> 25) & 0x1F) as u16;
        id.dst   = ((word >> 20) & 0x1F) as u16;
        id.dport = ((word >> 14) & 0x3F) as u8;
        id.sport = ((word >> 8)  & 0x3F) as u8;
        id.flags = (word & 0xFF)          as u8;
        pkt.set_id(id);

        // Read packed fields via copies to avoid unaligned-reference UB.
        let (src, dst) = unsafe {
            (
                core::ptr::addr_of!(id.src).read_unaligned(),
                core::ptr::addr_of!(id.dst).read_unaligned(),
            )
        };
        log::info!(
            "CSP prio={} src={} dst={} dport={} sport={} flags=0x{:02x} payload({} bytes)",
            id.pri, src, dst, id.dport, id.sport, id.flags,
            payload.len(),
        );

        self.handle.rx(pkt);
    }

    /// Drive the CSP router for one iteration.
    pub fn route_work(&self) {
        self.node.route_work().ok();
    }

    /// Non-blocking poll for a packet on the OTA port; returns it if one arrived.
    pub fn recv_ota(&self) -> Option<Packet> {
        self.socket.recvfrom(0)
    }

    /// Build a KISS-framed CSP beacon packet (src=1 → dst=0, port BEACON_PORT).
    /// Returns the raw bytes ready to write to UART.
    pub fn beacon_frame() -> Vec<u8> {
        // CSP v1 32-bit header: pri=2, src=1, dst=0, dport=BEACON_PORT, sport=0, flags=0
        let word: u32 = (2u32 << 30)
            | (7u32 << 25)
            | (0u32 << 20)
            | ((BEACON_PORT as u32) << 14)
            | (0u32 << 8)
            | 0;
        let mut frame = word.to_be_bytes().to_vec();
        frame.extend_from_slice(b"AWAKE");
        crate::kiss::kiss_encode(&frame)
    }
}
