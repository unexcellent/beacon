//! CSP node setup and messaging: node identity, the KISS interface as default
//! route, the service sockets, and the send helpers.

use crate::kiss;
use crate::uart::Uart;

pub const NODE: u16 = 7;
const PING_PORT: u8 = 1;
const OTA_PORT: u8 = 10;
const CMD_PORT: u8 = 11;
const PAYLOAD_NODE: u16 = 14;
const PAYLOAD_PORT: u8 = 1;
/// On-board computer: receives the boot status + firmware identity at startup.
pub const OBC_NODE: u16 = 1;
pub const OBC_PORT: u8 = 1;

pub struct Csp {
    pub node: libcsp::CspNode,
    pub iface: libcsp::interface::InterfaceHandle,
    pub ping_sock: libcsp::Socket,
    pub ota_sock: libcsp::Socket,
    pub cmd_sock: libcsp::Socket,
}

impl Csp {
    /// Bring up the CSP node with the KISS-over-UART interface as the default
    /// route and bind the ping / OTA / command service sockets.
    pub fn init() -> Self {
        let node = libcsp::CspConfig::new()
            .address(NODE)
            .hostname("beacon")
            .model("esp32p4")
            .init()
            .expect("csp init");

        let iface = libcsp::interface::register(kiss::UartKissIface);
        // Stamp our node address on packets leaving this interface. libcsp fills the
        // source from `snd_iface->addr` when it is 0, and `register()` zero-inits it,
        // so without this outgoing traffic (e.g. AVAILABLE) is sent from address 0.
        unsafe { (*iface.c_iface_ptr()).addr = NODE };
        unsafe { libcsp::route::set_default(iface.c_iface_ptr()).expect("default route") };

        let mut ping_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        ping_sock.bind(PING_PORT).expect("bind ping");

        let mut ota_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        ota_sock.bind(OTA_PORT).expect("bind ota");

        let mut cmd_sock = libcsp::Socket::new(libcsp::socket_opts::NONE);
        cmd_sock.bind(CMD_PORT).expect("bind cmd");

        Self {
            node,
            iface,
            ping_sock,
            ota_sock,
            cmd_sock,
        }
    }

    /// Send `msg` as a CSP packet to `dst`:`port`, then flush the KISS TX buffer.
    pub fn send_msg(&self, uart: &Uart, dst: u16, port: u8, msg: &[u8]) {
        if let Some(mut pkt) = libcsp::Packet::get(0) {
            pkt.write(msg).ok();
            self.node.sendto(libcsp::Priority::Norm, dst, port, 0, 0, pkt);
        }
        let to_send = kiss::take_tx();
        if !to_send.is_empty() {
            uart.send(&to_send);
        }
    }

    /// Report the beacon status (BUSY/AVAILABLE) to the payload board.
    pub fn send_status(&self, uart: &Uart, msg: &[u8]) {
        self.send_msg(uart, PAYLOAD_NODE, PAYLOAD_PORT, msg);
    }
}
