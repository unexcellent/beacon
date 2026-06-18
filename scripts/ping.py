#!/usr/bin/env python3
"""
Send a CSP ping (port 1) to node 7 and display the pong reply.
KISS-framed over RS485 via the DSD TECH SH-U11 USB adapter.

Usage:
    python3 ping.py [port] [--baud N] [--src N] [--dst N]
"""

import argparse
import glob
import struct
import sys
import time

import serial

FEND  = 0xC0
FESC  = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CSP_PING_PORT = 1
CSP_SRC_NODE  = 14
CSP_DST_NODE  = 7
CSP_SPORT     = 63
PING_PAYLOAD  = b"PING"


def kiss_encode(data: bytes) -> bytes:
    escaped = bytearray()
    for b in data:
        if b == FEND:
            escaped.extend([FESC, TFEND])
        elif b == FESC:
            escaped.extend([FESC, TFESC])
        else:
            escaped.append(b)
    return bytes([FEND, 0x00]) + bytes(escaped) + bytes([FEND])


def csp_header(src: int, dst: int, dport: int, sport: int, flags: int = 0, prio: int = 2) -> bytes:
    word = (
        ((prio  & 0x03) << 30) |
        ((src   & 0x1F) << 25) |
        ((dst   & 0x1F) << 20) |
        ((dport & 0x3F) << 14) |
        ((sport & 0x3F) <<  8) |
        (flags  & 0xFF)
    )
    return struct.pack(">I", word)


def parse_csp(data: bytes) -> dict | None:
    if len(data) < 4:
        return None
    word, = struct.unpack(">I", data[:4])
    return {
        "src":     (word >> 25) & 0x1F,
        "dst":     (word >> 20) & 0x1F,
        "dport":   (word >> 14) & 0x3F,
        "sport":   (word >>  8) & 0x3F,
        "flags":    word        & 0xFF,
        "payload": data[4:],
    }


def read_kiss_frames(ser: serial.Serial, timeout: float):
    """Yield unescaped KISS data-frame payloads within timeout seconds."""
    deadline = time.monotonic() + timeout
    buf = bytearray()
    in_frame = False
    esc = False

    while time.monotonic() < deadline:
        ser.timeout = max(0.01, deadline - time.monotonic())
        for b in ser.read(256):
            if esc:
                buf.append(FEND if b == TFEND else FESC if b == TFESC else b)
                esc = False
            elif b == FEND:
                if in_frame and len(buf) > 1 and buf[0] == 0x00:
                    yield bytes(buf[1:])
                buf.clear()
                in_frame = True
            elif in_frame:
                if b == FESC:
                    esc = True
                else:
                    buf.append(b)


def _detect_port() -> str | None:
    for pattern in ("/dev/tty.usbserial-*", "/dev/tty.wchusbserial-*", "/dev/ttyUSB*"):
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--src", type=int, default=CSP_SRC_NODE, help="Our CSP node ID (default 14)")
    parser.add_argument("--dst", type=int, default=CSP_DST_NODE, help="Target CSP node ID (default 7)")
    args = parser.parse_args()

    port = args.port or _detect_port()
    if port is None:
        print("ERROR: no serial port found — pass one explicitly")
        sys.exit(1)

    ser = serial.Serial(
        port=port, baudrate=args.baud,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=1,
    )
    time.sleep(0.1)

    frame = kiss_encode(csp_header(args.src, args.dst, CSP_PING_PORT, CSP_SPORT) + PING_PAYLOAD)
    ser.write(frame)
    ser.flush()
    print(f"→ PING  {args.src}:{CSP_SPORT} → {args.dst}:{CSP_PING_PORT}  payload={PING_PAYLOAD!r}")

    for raw in read_kiss_frames(ser, timeout=3.0):
        pkt = parse_csp(raw)
        if pkt and pkt["src"] == args.dst and pkt["dport"] == CSP_SPORT:
            print(f"← PONG  {pkt['src']}:{pkt['sport']} → {pkt['dst']}:{pkt['dport']}  payload={pkt['payload']!r}")
            break
    else:
        print("✗ no pong received (timeout 3 s)")

    ser.close()


if __name__ == "__main__":
    main()
