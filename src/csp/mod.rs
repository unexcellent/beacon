//! A CSP node speaking KISS over a serial link.
//!
//! [`CspLink`] owns the libcsp node and the TX side: outgoing packets are
//! KISS-encoded and written to the serial port from the interface's nexthop.
//! The application owns the RX byte loop — serial reads, timeouts, error
//! policy, logging — and hands complete packets back via [`CspLink::inject`].
//!
//! Linking this module makes it the program's libcsp runtime: the FreeRTOS /
//! ESP-IDF arch glue in [`arch`] is exported process-wide, and it assumes a
//! single-threaded dispatch loop (see its module docs).

mod arch;

use crate::kiss;

/// CSP's standard ping/echo service port.
const PING_PORT: u8 = 1;

/// Error raised while bringing up the CSP node.
#[derive(Clone, Copy, Debug)]
pub enum CspError {
    Init,
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
/// pings via libcsp's built-in service handler.
pub struct CspLink {
    node: libcsp::CspNode,
    iface: libcsp::interface::InterfaceHandle,
    ping_sock: libcsp::Socket,
}

impl CspLink {
    pub fn try_new(
        config: CspLinkConfig,
        tx: impl SerialWrite + Send + 'static,
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

    /// Inject one received packet: route it and answer any pending pings.
    /// The arch glue relies on route_work running after every rx (see [`arch`]).
    pub fn inject(&mut self, packet: libcsp::Packet) {
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
