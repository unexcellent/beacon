#!/usr/bin/env python3
"""
Passively monitor RS422 / KISS-framed CSP traffic via the DSD TECH SH-U11
in full-duplex RS422 mode (Y/Z = TX pair, A/B = RX pair, no A<->Y B<->Z jumpers).

Never transmits anything.  Decodes CSP headers when possible.

Usage:
    python3 monitor.py [port] [--baud N]
"""

import argparse
import glob
import struct
import sys
import time

import serial

# KISS
FEND  = 0xC0
FESC  = 0xDB
TFEND = 0xDC
TFESC = 0xDD

# Known OTA commands
_OTA_PORT = 10
_OTA_CMDS = {0x00: "ANNOUNCE", 0x01: "BEGIN", 0x02: "DATA", 0x03: "END"}


def _detect_port() -> str | None:
    candidates = (
        glob.glob("/dev/tty.usbserial-*")
        + glob.glob("/dev/tty.wchusbserial-*")
        + glob.glob("/dev/ttyUSB*")
    )
    return candidates[0] if candidates else None


def kiss_unescape(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == FESC and i + 1 < len(data):
            nxt = data[i + 1]
            if nxt == TFEND:
                out.append(FEND)
            elif nxt == TFESC:
                out.append(FESC)
            else:
                out.append(b)
                out.append(nxt)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def _fmt_payload(data: bytes, dport: int) -> str:
    if not data:
        return "(empty)"
    if dport == _OTA_PORT:
        cmd = _OTA_CMDS.get(data[0])
        if cmd == "BEGIN" and len(data) >= 5:
            size = struct.unpack("<I", data[1:5])[0]
            return f"OTA:BEGIN size={size:,}"
        if cmd == "DATA" and len(data) >= 5:
            offset = struct.unpack("<I", data[1:5])[0]
            return f"OTA:DATA offset={offset:,} len={len(data) - 5}"
        if cmd:
            return f"OTA:{cmd}"
    try:
        s = data.decode("utf-8")
        if all(c.isprintable() for c in s):
            return f'"{s}"'
    except UnicodeDecodeError:
        pass
    if len(data) <= 32:
        return " ".join(f"{b:02x}" for b in data)
    return f"({len(data)} bytes)"


def decode_csp(raw: bytes) -> str | None:
    """Return a formatted CSP log line, or None if the frame is too short."""
    if len(raw) < 4:
        return None
    hdr   = struct.unpack(">I", raw[:4])[0]
    src   = (hdr >> 25) & 0x1F
    dst   = (hdr >> 20) & 0x1F
    dport = (hdr >> 14) & 0x3F
    sport = (hdr >>  8) & 0x3F
    flags = hdr & 0xFF
    pl    = _fmt_payload(raw[4:], dport)
    return f"[RX] from {src}:{sport} to {dst}:{dport} is {pl} (flags 0x{flags:02x})"


def iter_kiss_frames(port: serial.Serial):
    """Yield raw (unescaped) KISS frame payloads as they arrive."""
    buf = bytearray()
    in_frame = False

    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        for b in chunk:
            if b == FEND:
                if in_frame and len(buf) > 1:
                    # buf[0] is the KISS command byte (0x00 = data)
                    yield kiss_unescape(bytes(buf[1:]))
                buf.clear()
                in_frame = True
            elif in_frame:
                buf.append(b)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    port = args.port or _detect_port()
    if port is None:
        print("ERROR: no serial port found — pass one explicitly")
        sys.exit(1)

    print(f"Monitoring {port} @ {args.baud} baud  RS422 full-duplex  (read-only, Ctrl-C to stop)\n")

    ser = serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        rtscts=False,
        dsrdtr=False,
    )
    # In RS422 full-duplex mode the SH-U11's TX driver (Y/Z) is always enabled
    # by hardware; RTS must not be asserted or it may activate direction control
    # on adapters that support both RS485 and RS422 modes.
    ser.rts = False
    ser.dtr = False

    try:
        for raw in iter_kiss_frames(ser):
            ts = time.strftime("%H:%M:%S")
            csp = decode_csp(raw)
            if csp:
                print(f"[{ts}] {csp}")
            else:
                print(f"[{ts}] RAW {raw.hex()}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
