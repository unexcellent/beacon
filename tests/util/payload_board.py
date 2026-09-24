#!/usr/bin/env python3
"""A mock of the MOVE-IIIa payload board's CSP node, seen from the ESP32 beacon.

The beacon (CSP node ``SSTV`` = 7) talks CSP-over-KISS over RS422/UART to the
payload board (CSP node ``PAYLOAD`` = 14). This module drives the *payload* end
of that link so an engineering-model ESP32 can be exercised on the bench: it
sends the payload board's commands to the ESP and interprets the status frames
the ESP sends back.

What the real payload board does toward the ESP (see ../move-iiia
apps/payload/src/workers/diagnostic.c and src/csp/sstv.c):

* Sends the 4-byte ASCII command ``b"SSTV"`` to node 7 port 11 to ask the ESP to
  capture and transmit a Robot36 image.
* Pings the ESP (CSP echo, port 1) to prove the link.
* Watches the link for the ESP's plain-ASCII status frames ``b"BUSY"`` (the ESP
  has started transmitting audio -> key the VHF PTT) and ``b"AVAILABLE"`` (done
  -> unkey). It reacts by byte-exact match on the CSP payload.

Wire format (matches src/link/kiss.rs and src/link/csp on the beacon side):

* KISS: FEND=0xC0, FESC=0xDB, TFEND=0xDC, TFESC=0xDD, TNC data command 0x00.
  A frame is ``FEND 0x00 <byte-stuffed CSP packet> FEND``.
* CSP v1 header: one big-endian u32,
  ``(pri<<30)|(src<<25)|(dst<<20)|(dport<<14)|(sport<<8)|flags`` (src/dst 5-bit,
  ports 6-bit, flags 8-bit), followed by the payload.
* CRC: the beacon's decode_packet only strips a trailing 4-byte CRC when the
  CRC32 flag (0x10) is set, and never verifies it, so this mock sends frames
  with no CRC (flags=0) by default. Set ``use_crc=True`` to append CRC-32C and
  flag it (needed only against a real libcsp peer).

Note on the network map: the constants below mirror the *beacon's* view
(src/link/message.rs, src/link/mod.rs) — PAYLOAD=14, SSTV/ESP=7, OBC=1,
UHF_GROUND=2. The move-iiia wire registry numbers some of these differently
(CDH/OBC=12, UHF=5, the payload's UART iface address is 8); the beacon is the
authority for what the ESP actually emits and accepts, so its numbers win here.

Run this file directly to log the link live, behaving as the payload board would
(periodically pinging + requesting SSTV, and keying/unkeying the VHF PTT on the
ESP's BUSY/AVAILABLE):

    ./.venv/bin/python tests/util/payload_board.py [PORT] [--interval SECONDS]
"""

from __future__ import annotations

import argparse
import glob
import logging
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial

log = logging.getLogger("payload_board")

# --- CSP network map, from the beacon's perspective (src/link) ---------------
NODE_SSTV = 7  # the ESP32 under test
NODE_PAYLOAD = 14  # this mock board (beacon PAYLOAD_NODE)
NODE_OBC = 1  # beacon OBC_NODE — target of BOOTED status
NODE_UHF_GROUND = 2  # beacon UHF_GROUND_NODE — target of error reports

PORT_PING = 1  # CSP built-in echo/ping service
PORT_UPDATE = 10  # beacon UPDATE_PORT (OTA; the real board never drives this)
PORT_CMD = 11  # beacon CMD_PORT — receives b"SSTV"

# The ESP's status payloads it sends back (src/link/message.rs).
STATUS_BUSY = b"BUSY"
STATUS_AVAILABLE = b"AVAILABLE"

# --- KISS constants (src/link/kiss.rs) ---------------------------------------
FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD
KISS_CMD_DATA = 0x00

CSP_FLAG_CRC32 = 0x10

# Update wire protocol, first payload byte selects the command (src/link/command.rs).
UPDATE_CMD_ANNOUNCE = 0x00
UPDATE_CMD_BEGIN = 0x01
UPDATE_CMD_DATA = 0x02
UPDATE_CMD_END = 0x03

# A fixed ephemeral source port for our outgoing packets. Its only role is to be
# the destination the ESP echoes ping replies back to; the beacon's command and
# update sockets ignore the source port entirely.
EPHEMERAL_PORT = 48

PRIO_NORM = 2


@dataclass
class CspPacket:
    """A decoded CSP v1 packet received from the ESP."""

    prio: int
    src: int
    dst: int
    dport: int
    sport: int
    flags: int
    payload: bytes

    def __str__(self) -> str:
        text = _printable(self.payload)
        return (
            f"{self.src}:{self.sport} -> {self.dst}:{self.dport} "
            f"flags=0x{self.flags:02x} [{len(self.payload)}] {text}"
        )


def _printable(data: bytes) -> str:
    if all(0x20 <= b < 0x7F for b in data) and data:
        return repr(data.decode("ascii"))
    return data.hex(" ")


# --- CSP v1 header + CRC-32C -------------------------------------------------


def pack_header(prio: int, src: int, dst: int, dport: int, sport: int, flags: int) -> bytes:
    """Pack a CSP v1 header word, big-endian, matching kiss.rs::encode_packet."""
    word = (
        (prio & 0x3) << 30
        | (src & 0x1F) << 25
        | (dst & 0x1F) << 20
        | (dport & 0x3F) << 14
        | (sport & 0x3F) << 8
        | (flags & 0xFF)
    )
    return struct.pack(">I", word)


def unpack_header(word_bytes: bytes) -> tuple[int, int, int, int, int, int]:
    """Inverse of pack_header: (prio, src, dst, dport, sport, flags)."""
    (word,) = struct.unpack(">I", word_bytes)
    return (
        (word >> 30) & 0x3,
        (word >> 25) & 0x1F,
        (word >> 20) & 0x1F,
        (word >> 14) & 0x3F,
        (word >> 8) & 0x3F,
        word & 0xFF,
    )


_CRC32C_POLY = 0x82F63B78  # reflected Castagnoli, as used by libcsp's KISS CRC
_CRC32C_TABLE = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ _CRC32C_POLY if _c & 1 else _c >> 1
    _CRC32C_TABLE.append(_c)


def crc32c(data: bytes) -> int:
    """CRC-32C (Castagnoli) over the payload, matching libcsp's CSP v1 KISS CRC."""
    crc = 0xFFFFFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC32C_TABLE[(crc ^ b) & 0xFF]
    return crc ^ 0xFFFFFFFF


# --- KISS framing ------------------------------------------------------------


def kiss_encode(packet: bytes) -> bytes:
    """Wrap a CSP packet in a KISS frame, byte-stuffing FEND/FESC (kiss.rs)."""
    out = bytearray([FEND, KISS_CMD_DATA])
    for b in packet:
        if b == FEND:
            out += bytes([FESC, TFEND])
        elif b == FESC:
            out += bytes([FESC, TFESC])
        else:
            out.append(b)
    out.append(FEND)
    return bytes(out)


class KissDecoder:
    """Streaming KISS decoder mirroring the beacon's kiss.rs state machine.

    Feed bytes with :meth:`push`; each completed frame (the unstuffed CSP packet,
    without the KISS command byte) is yielded. Empty frames are dropped, and a
    bad command or escape byte resets to idle, exactly as the beacon does.
    """

    IDLE, COMMAND, DATA, ESCAPE = range(4)

    def __init__(self) -> None:
        self._state = self.IDLE
        self._buf = bytearray()

    def push(self, byte: int) -> Optional[bytes]:
        if self._state == self.IDLE:
            if byte == FEND:
                self._state = self.COMMAND
        elif self._state == self.COMMAND:
            if byte == FEND:
                pass
            elif byte == KISS_CMD_DATA:
                self._buf.clear()
                self._state = self.DATA
            else:
                self._state = self.IDLE
        elif self._state == self.DATA:
            if byte == FEND:
                if self._buf:
                    frame = bytes(self._buf)
                    self._buf.clear()
                    self._state = self.COMMAND
                    return frame
            elif byte == FESC:
                self._state = self.ESCAPE
            else:
                self._buf.append(byte)
        elif self._state == self.ESCAPE:
            if byte == TFEND:
                self._buf.append(FEND)
                self._state = self.DATA
            elif byte == TFESC:
                self._buf.append(FESC)
                self._state = self.DATA
            else:
                self._state = self.IDLE
        return None


def decode_packet(frame: bytes) -> Optional[CspPacket]:
    """Decode an unstuffed KISS frame into a CspPacket, or None if too short.

    Mirrors kiss.rs::decode_packet: strips a trailing 4-byte CRC only when the
    CRC32 flag is set (and does not verify it).
    """
    if len(frame) < 4:
        return None
    prio, src, dst, dport, sport, flags = unpack_header(frame[:4])
    payload = frame[4:]
    if flags & CSP_FLAG_CRC32 and len(payload) >= 4:
        payload = payload[:-4]
    return CspPacket(prio, src, dst, dport, sport, flags, payload)


class MockPayloadBoard:
    """Drives the payload end of the RS422 CSP link toward an ESP32 beacon.

    Open it (as a context manager, or call :meth:`start`/:meth:`close`); a
    background thread reads the serial port, decodes incoming CSP packets, tracks
    the ESP's BUSY/AVAILABLE PTT signalling, and queues packets for tests to
    assert on. Outgoing helpers (:meth:`send_sstv`, :meth:`ping`, the
    ``update_*`` methods) build and KISS-frame packets the beacon accepts.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 115_200,
        address: int = NODE_PAYLOAD,
        esp_address: int = NODE_SSTV,
        use_crc: bool = False,
        on_message: Optional[Callable[[CspPacket], None]] = None,
    ) -> None:
        self.port = port or detect_port()
        self.baud = baud
        self.address = address
        self.esp_address = esp_address
        self.use_crc = use_crc
        self.on_message = on_message

        self._serial: Optional[serial.Serial] = None
        self._decoder = KissDecoder()
        self._rx_queue: "queue.Queue[CspPacket]" = queue.Queue()
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._transmitting = False

    # -- lifecycle ------------------------------------------------------------

    def start(self) -> "MockPayloadBoard":
        """Open the serial port and begin reading in the background."""
        self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="payload-rx", daemon=True)
        self._reader.start()
        log.info(
            "payload board %d up on %s @ %d 8N1, talking to ESP node %d",
            self.address,
            self.port,
            self.baud,
            self.esp_address,
        )
        return self

    def close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "MockPayloadBoard":
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def transmitting(self) -> bool:
        """Whether the ESP is currently in a transmission window (BUSY seen, no
        AVAILABLE yet) — i.e. whether the real board would be keying the VHF."""
        return self._transmitting

    # -- outgoing -------------------------------------------------------------

    def send_sstv(self) -> None:
        """Ask the ESP to capture and SSTV-transmit (b"SSTV" to port 11)."""
        self._send(self.esp_address, PORT_CMD, b"SSTV", label="SSTV request")

    def ping(self, data: bytes = bytes(range(8)), timeout: float = 1.0) -> bool:
        """Send a CSP echo request to the ESP and wait for the reply.

        Returns True if a matching echo came back within ``timeout``.
        """
        self._drain_queue()
        self._send(self.esp_address, PORT_PING, data, label="ping")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pkt = self.receive(timeout=deadline - time.monotonic())
            if pkt is not None and pkt.src == self.esp_address and pkt.payload == data:
                return True
        return False

    def update_announce(self, chunk_size: int) -> None:
        """Announce a firmware update with the given chunk size (port 10)."""
        self._send(
            self.esp_address,
            PORT_UPDATE,
            bytes([UPDATE_CMD_ANNOUNCE]) + struct.pack("<H", chunk_size),
            label="update ANNOUNCE",
        )

    def update_begin(self, total: int) -> None:
        """Begin a firmware update session of ``total`` bytes (port 10)."""
        self._send(
            self.esp_address,
            PORT_UPDATE,
            bytes([UPDATE_CMD_BEGIN]) + struct.pack("<I", total),
            label="update BEGIN",
        )

    def update_data(self, offset: int, data: bytes) -> None:
        """Send one firmware chunk starting at ``offset`` (port 10)."""
        self._send(
            self.esp_address,
            PORT_UPDATE,
            bytes([UPDATE_CMD_DATA]) + struct.pack("<I", offset) + data,
            label=f"update DATA @{offset} ({len(data)} b)",
        )

    def update_end(self) -> None:
        """Signal the firmware transfer is complete (port 10)."""
        self._send(self.esp_address, PORT_UPDATE, bytes([UPDATE_CMD_END]), label="update END")

    def send_raw(self, dst: int, dport: int, payload: bytes, sport: int = EPHEMERAL_PORT) -> None:
        """Send an arbitrary CSP payload — escape hatch for bespoke tests."""
        self._send(dst, dport, payload, sport=sport, label="raw")

    def _send(
        self,
        dst: int,
        dport: int,
        payload: bytes,
        sport: int = EPHEMERAL_PORT,
        label: str = "",
    ) -> None:
        if self._serial is None:
            raise RuntimeError("board not started")
        flags = CSP_FLAG_CRC32 if self.use_crc else 0
        body = payload + struct.pack(">I", crc32c(payload)) if self.use_crc else payload
        packet = pack_header(PRIO_NORM, self.address, dst, dport, sport, flags) + body
        with self._tx_lock:
            self._serial.write(kiss_encode(packet))
        log.info("TX -> %d:%d  %s  %s", dst, dport, _printable(payload), f"({label})" if label else "")

    # -- incoming -------------------------------------------------------------

    def receive(self, timeout: Optional[float] = None) -> Optional[CspPacket]:
        """Pop the next received packet, or None if none arrives within timeout."""
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for(
        self, predicate: Callable[[CspPacket], bool], timeout: float = 5.0
    ) -> Optional[CspPacket]:
        """Wait for a received packet satisfying ``predicate`` within ``timeout``."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pkt = self.receive(timeout=deadline - time.monotonic())
            if pkt is not None and predicate(pkt):
                return pkt
        return None

    def wait_for_text(self, text: bytes, timeout: float = 5.0) -> Optional[CspPacket]:
        """Wait for a packet whose payload equals ``text`` (e.g. b"BUSY")."""
        return self.wait_for(lambda p: p.payload == text, timeout)

    def _drain_queue(self) -> None:
        while True:
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                return

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop.is_set():
            try:
                chunk = self._serial.read(256)
            except serial.SerialException as exc:
                log.error("serial read failed: %s", exc)
                return
            for byte in chunk:
                frame = self._decoder.push(byte)
                if frame is None:
                    continue
                pkt = decode_packet(frame)
                if pkt is not None:
                    self._handle(pkt)

    def _handle(self, pkt: CspPacket) -> None:
        if pkt.payload == STATUS_BUSY and not self._transmitting:
            self._transmitting = True
            log.info("RX <- %s  -> keying VHF PTT", pkt)
        elif pkt.payload == STATUS_AVAILABLE and self._transmitting:
            self._transmitting = False
            log.info("RX <- %s  -> unkeying VHF PTT", pkt)
        else:
            log.info("RX <- %s", pkt)
        self._rx_queue.put(pkt)
        if self.on_message is not None:
            self.on_message(pkt)


def detect_port() -> str:
    """Return the first likely USB serial adapter (the RS422 dongle)."""
    for pattern in ("/dev/cu.usbserial-*", "/dev/cu.usbmodem*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    raise RuntimeError("no USB serial port found — pass one explicitly")


def main() -> None:
    parser = argparse.ArgumentParser(description="Log/drive the ESP link as the payload board.")
    parser.add_argument("port", nargs="?", help="serial device (default: auto-detect)")
    parser.add_argument("-b", "--baud", type=int, default=115_200)
    parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=120.0,
        help="seconds between SSTV requests (0 = passive: only log incoming)",
    )
    parser.add_argument("--no-ping", action="store_true", help="do not ping before each request")
    parser.add_argument("--crc", action="store_true", help="append CRC-32C to outgoing frames")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d %(message)s",
        datefmt="%H:%M:%S",
    )

    with MockPayloadBoard(args.port, baud=args.baud, use_crc=args.crc) as board:
        try:
            if args.interval <= 0:
                log.info("passive mode — logging incoming frames, Ctrl-C to stop")
                while True:
                    board.receive(timeout=1.0)
            else:
                while True:
                    if not args.no_ping:
                        log.info("ping %s", "ok" if board.ping() else "no reply")
                    board.send_sstv()
                    time.sleep(args.interval)
        except KeyboardInterrupt:
            print(file=sys.stderr)
            log.info("stopping")


if __name__ == "__main__":
    main()
