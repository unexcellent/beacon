//! A CSP node speaking KISS over a full-duplex serial link.
//!
//! [`CspLink`] owns both directions of the line: outgoing packets are
//! KISS-encoded and written from the interface's nexthop (TX), and
//! [`CspLink::poll`] reads the RX transport, KISS-decodes complete packets and
//! routes them (RX). The application drives it by binding service sockets and
//! calling `poll` in its loop.
//!
//! Linking this module makes it the program's libcsp runtime: the FreeRTOS /
//! ESP-IDF arch glue in [`arch`] is exported process-wide, and it assumes a
//! single-threaded dispatch loop (see its module docs).

mod arch;

use std::time::Duration;

use super::kiss;

/// CSP's standard ping/echo service port.
const PING_PORT: u8 = 1;

/// Consecutive RX read failures tolerated before [`CspLink::poll`] reports an
/// [`CspError::Rx`]. With the 100 ms retry pacing this means one report per
/// second of persistent failure.
const MAX_RX_ERRORS: u32 = 10;

/// Error raised by the CSP node.
#[derive(Clone, Copy, Debug)]
pub enum CspError {
    /// Bringing up the node, interface, or a socket failed.
    Init,
    /// The RX transport read failed persistently (past [`MAX_RX_ERRORS`]).
    Rx,
}

/// A blocking serial transmitter: writes the whole buffer and drains the
/// hardware FIFO before returning.
pub trait SerialWrite {
    fn write_all(&mut self, data: &[u8]);
}

impl SerialWrite for esp_idf_hal::uart::UartTxDriver<'_> {
    fn write_all(&mut self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            sent += self.write(&data[sent..]).unwrap();
        }
        self.wait_done(esp_idf_hal::delay::BLOCK).unwrap();
    }
}

/// A serial receiver: reads whatever bytes are available into `buf`, waiting up
/// to the transport's read timeout. Returns the count read, or `Err` when the
/// read itself fails (distinct from a timeout with no bytes, which is `Ok(0)`).
pub trait SerialRead {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()>;
}

impl SerialRead for esp_idf_hal::uart::UartRxDriver<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        let timeout = esp_idf_hal::delay::TickType::new_millis(100).ticks();
        // Disambiguate from this trait method of the same name.
        esp_idf_hal::uart::UartRxDriver::read(self, buf, timeout).map_err(|_| ())
    }
}

/// libcsp interface owning the serial TX half.
struct KissSerialInterface<W> {
    tx: W,
}

impl<W: SerialWrite + Send> libcsp::CspInterface for KissSerialInterface<W> {
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
            kiss::fmt_payload(packet.data()),
            flags
        );
        self.tx.write_all(&kiss::kiss_encode(&kiss::encode_packet(&packet)));
    }

    fn name(&self) -> &str {
        "KISS"
    }
}

pub struct CspLinkConfig {
    /// This node's CSP address, stamped on all outgoing packets.
    pub address: u16,
    pub hostname: &'static str,
    pub model: &'static str,
}

/// A CSP node with a KISS serial interface as its default route, answering
/// pings via libcsp's built-in service handler. Owns the RX transport `R` and
/// pumps it in [`poll`](Self::poll).
pub struct CspLink<R> {
    node: libcsp::CspNode,
    iface: libcsp::interface::InterfaceHandle,
    ping_sock: libcsp::Socket,
    rx: R,
    decoder: kiss::KissDecoder,
    /// Consecutive RX read failures, reset by every successful read.
    rx_errors: u32,
}

impl<R: SerialRead> CspLink<R> {
    pub fn try_new(
        config: CspLinkConfig,
        tx: impl SerialWrite + Send + 'static,
        rx: R,
    ) -> Result<Self, CspError> {
        let node = libcsp::CspConfig::new()
            .address(config.address)
            .hostname(config.hostname)
            .model(config.model)
            .init()
            .map_err(|_| CspError::Init)?;

        let iface = libcsp::interface::register(KissSerialInterface { tx });
        // Stamp our node address on packets leaving this interface. libcsp fills the
        // source from `snd_iface->addr` when it is 0, and `register()` zero-inits it,
        // so without this outgoing traffic is sent from address 0.
        unsafe { (*iface.c_iface_ptr()).addr = config.address };
        unsafe { libcsp::route::set_default(iface.c_iface_ptr()).map_err(|_| CspError::Init)? };

        let ping_sock = Self::bind_socket(PING_PORT)?;

        Ok(Self {
            node,
            iface,
            ping_sock,
            rx,
            decoder: kiss::KissDecoder::new(),
            rx_errors: 0,
        })
    }

    /// Bind a service socket the application will drain itself.
    pub fn bind(&self, port: u8) -> Result<libcsp::Socket, CspError> {
        Self::bind_socket(port)
    }

    fn bind_socket(port: u8) -> Result<libcsp::Socket, CspError> {
        let mut sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        sock.bind(port).map_err(|_| CspError::Init)?;
        Ok(sock)
    }

    /// Transmit one datagram. The interface's nexthop KISS-encodes it and
    /// writes it to the serial port before this returns. Dropped silently if
    /// no packet buffer is available.
    pub fn send(&self, node: u16, port: u8, priority: libcsp::Priority, payload: &[u8]) {
        if let Some(mut pkt) = libcsp::Packet::get(0) {
            pkt.write(payload).ok();
            self.node.sendto(priority, node, port, 0, 0, pkt);
        }
    }

    /// Pump the RX line once: read available bytes, KISS-decode every complete
    /// packet and route it (injecting into the node and answering pings). Blocks
    /// up to the transport's read timeout, so it is meant to be called in a
    /// loop. Fails with [`CspError::Rx`] only after [`MAX_RX_ERRORS`] consecutive
    /// read failures; a timeout with no bytes is a successful empty poll.
    pub fn poll(&mut self) -> Result<(), CspError> {
        let mut buf = [0u8; 512];

        let Ok(n) = self.rx.read(&mut buf) else {
            self.rx_errors += 1;

            std::thread::sleep(Duration::from_millis(100));
            if self.rx_errors >= MAX_RX_ERRORS {
                self.rx_errors = 0;
                return Err(CspError::Rx);
            }
            return Ok(());
        };
        self.rx_errors = 0;
        for &b in &buf[..n] {
            let Some(frame) = self.decoder.push(b) else {
                continue;
            };
            if let Some(pkt) = kiss::decode_packet(&frame) {
                log_rx_packet(&pkt);
                self.inject(pkt);
            }
        }
        Ok(())
    }

    /// Inject one received packet: route it and answer any pending pings.
    /// The arch glue relies on route_work running after every rx (see [`arch`]).
    fn inject(&mut self, packet: libcsp::Packet) {
        self.iface.rx(packet);
        let _ = self.node.route_work();
        self.service_ping();
    }

    fn service_ping(&mut self) {
        while let Some(conn) = self.ping_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                conn.handle_service(pkt);
            }
        }
    }
}

/// Log one received packet at debug level (the outgoing side logs at info from
/// the nexthop). Kept quiet by default so a firmware transfer's data chunks do
/// not flood the console.
fn log_rx_packet(pkt: &libcsp::Packet) {
    let id = pkt.id();
    // Copy out of the packed id struct before formatting (can't reference its fields).
    let (src, sport, dst, dport, flags) = (id.src, id.sport, id.dst, id.dport, id.flags);
    log::debug!(
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
