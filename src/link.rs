//! The CSP-over-KISS-over-RS422 link to the payload board: UART setup, the
//! libcsp interface and service sockets, and the per-iteration RX pump.

use esp_idf_hal::{
    delay,
    gpio::{AnyIOPin, Gpio37, Gpio38, Gpio39, Output, PinDriver},
    uart::{self, UART1, UartDriver, UartRxDriver, UartTxDriver},
    units::Hertz,
};

use crate::kiss;
use crate::ota::OtaState;
use crate::{Error, Result};

pub const BAUD_RATE: u32 = 115_200;

pub const NODE: u16 = 7;
const PING_PORT: u8 = 1;
const OTA_PORT: u8 = 10;
const CMD_PORT: u8 = 11;
const PAYLOAD_NODE: u16 = 14;
const PAYLOAD_PORT: u8 = 1;
/// On-board computer: receives the boot status + firmware identity at startup.
pub const OBC_NODE: u16 = 1;
pub const OBC_PORT: u8 = 1;

/// A command received from the payload board, returned by [`PayloadLink::poll`]
/// for the application to execute.
pub enum Command {
    Sstv,
}

/// libcsp interface owning the UART TX half: outgoing packets are KISS-encoded
/// and written to the wire directly from nexthop().
struct KissUartIface {
    tx: UartTxDriver<'static>,
}

impl KissUartIface {
    /// Write all of `data` and block until the TX FIFO has drained.
    fn send(&mut self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            sent += self.tx.write(&data[sent..]).unwrap();
        }
        self.tx.wait_done(delay::BLOCK).unwrap();
    }
}

impl libcsp::CspInterface for KissUartIface {
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
        let word: u32 = ((id.pri as u32) << 30)
            | ((id.src as u32) << 25)
            | ((id.dst as u32) << 20)
            | ((id.dport as u32) << 14)
            | ((id.sport as u32) << 8)
            | (id.flags as u32);
        let mut raw = word.to_be_bytes().to_vec();
        raw.extend_from_slice(packet.data());
        self.send(&kiss::kiss_encode(&raw));
    }

    fn name(&self) -> &str {
        "KISS"
    }
}

pub struct PayloadLink {
    rx: UartRxDriver<'static>,
    node: libcsp::CspNode,
    iface: libcsp::interface::InterfaceHandle,
    ping_sock: libcsp::Socket,
    ota_sock: libcsp::Socket,
    cmd_sock: libcsp::Socket,
    decoder: kiss::KissDecoder,
    ota: OtaState,
    /// RS422 full-duplex: the TX driver-enable pin is held high for the lifetime
    /// of the link. Kept as a field so it is not dropped (which would reset the
    /// pin and disable the transmitter).
    _de: PinDriver<'static, Output>,
}

impl PayloadLink {
    /// Bring up the RS422 UART, the CSP node with the KISS interface as the
    /// default route, and the ping / OTA / command service sockets.
    pub fn try_new(
        uart: UART1<'static>,
        tx: Gpio38<'static>,
        rx: Gpio37<'static>,
        de: Gpio39<'static>,
    ) -> Result<Self> {
        let mut de = PinDriver::output(de).map_err(|_| Error::Peripheral)?;
        de.set_high().map_err(|_| Error::Peripheral)?;

        let driver = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &uart::config::Config::new()
                .baudrate(Hertz(BAUD_RATE))
                .rx_fifo_size(8192),
        )
        .map_err(|_| Error::UartAllocation)?;
        let (uart_tx, uart_rx) = driver.into_split();

        let node = libcsp::CspConfig::new()
            .address(NODE)
            .hostname("beacon")
            .model("esp32p4")
            .init()
            .map_err(|_| Error::CspInit)?;

        let iface = libcsp::interface::register(KissUartIface { tx: uart_tx });
        // Stamp our node address on packets leaving this interface. libcsp fills the
        // source from `snd_iface->addr` when it is 0, and `register()` zero-inits it,
        // so without this outgoing traffic (e.g. AVAILABLE) is sent from address 0.
        unsafe { (*iface.c_iface_ptr()).addr = NODE };
        unsafe { libcsp::route::set_default(iface.c_iface_ptr()).map_err(|_| Error::CspInit)? };

        let mut ping_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        ping_sock.bind(PING_PORT).map_err(|_| Error::CspInit)?;

        let mut ota_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        ota_sock.bind(OTA_PORT).map_err(|_| Error::CspInit)?;

        let mut cmd_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        cmd_sock.bind(CMD_PORT).map_err(|_| Error::CspInit)?;

        log::info!(
            "CSP node {} ready at {} baud (RX=G37 TX=G38 DE=G39)",
            NODE,
            BAUD_RATE
        );

        Ok(Self {
            rx: uart_rx,
            node,
            iface,
            ping_sock,
            ota_sock,
            cmd_sock,
            decoder: kiss::KissDecoder::new(),
            ota: OtaState::new(),
            _de: de,
        })
    }

    /// Send `msg` as a CSP packet to `dst`:`port`. The interface's nexthop
    /// KISS-encodes it and writes it to the UART before this returns.
    pub fn send_msg(&self, dst: u16, port: u8, msg: &[u8]) {
        if let Some(mut pkt) = libcsp::Packet::get(0) {
            pkt.write(msg).ok();
            self.node
                .sendto(libcsp::Priority::Norm, dst, port, 0, 0, pkt);
        }
    }

    /// Report the beacon status (BUSY/AVAILABLE) to the payload board.
    pub fn send_status(&self, msg: &[u8]) {
        self.send_msg(PAYLOAD_NODE, PAYLOAD_PORT, msg);
    }

    /// Pump the link once: feed received UART bytes through KISS into CSP, run
    /// the router, service the ping and OTA sockets internally, and return the
    /// application commands that arrived.
    pub fn poll(&mut self) -> Vec<Command> {
        let mut buf = [0u8; 512];
        // Poll the RS485 read with a short timeout (rather than blocking forever) so
        // the main loop also gets to service the USB-C debug trigger each iteration.
        let read_timeout = delay::TickType::new_millis(100).ticks();

        if let Ok(n) = self.rx.read(&mut buf, read_timeout) {
            for &b in &buf[..n] {
                if let Some(frame) = self.decoder.push(b) {
                    if let Some(pkt) = kiss::frame_to_packet(&frame) {
                        if !matches!(self.ota, OtaState::Writing(_)) {
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
                        self.iface.rx(pkt);
                    }
                }
            }
        }

        let _ = self.node.route_work();

        while let Some(conn) = self.ping_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                conn.handle_service(pkt);
            }
        }

        while let Some(conn) = self.ota_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                self.ota.handle(pkt.data());
            }
        }

        let mut commands = Vec::new();
        while let Some(conn) = self.cmd_sock.accept(0) {
            while let Some(pkt) = conn.read(0) {
                if pkt.data().starts_with(b"SSTV") {
                    log::info!("CMD: SSTV requested");
                    commands.push(Command::Sstv);
                } else {
                    log::warn!("CMD: unknown payload: {}", kiss::fmt_payload(pkt.data()));
                }
            }
        }
        commands
    }
}
