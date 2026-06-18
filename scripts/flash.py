#!/usr/bin/env python3
"""
Flash firmware to the ESP32 via CSP OTA (port 10) over RS485.

Protocol:
  ANNOUNCE (0x00): warn the node a flash is coming
  BEGIN    (0x01): 4-byte total size (LE)
  DATA     (0x02): 4-byte offset (LE) + up to CHUNK bytes
  END      (0x03): finalise OTA and reboot

Usage:
    python3 flash.py <firmware.bin|elf> [port] [--baud N] [--chunk N] [--delay S]
"""

import argparse
import glob
import os
import struct
import subprocess
import sys
import tempfile
import time

import serial

FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD

CSP_PRIO = 2
CSP_SRC = 14  # ground node
CSP_DST = 7  # ESP32 node
CSP_DPORT = 10  # OTA port
CSP_SPORT = 63

CMD_ANNOUNCE = 0x00
CMD_BEGIN = 0x01
CMD_DATA = 0x02
CMD_END = 0x03


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


def csp_packet(payload: bytes) -> bytes:
    word = (
        ((CSP_PRIO & 0x03) << 30)
        | ((CSP_SRC & 0x1F) << 25)
        | ((CSP_DST & 0x1F) << 20)
        | ((CSP_DPORT & 0x3F) << 14)
        | ((CSP_SPORT & 0x3F) << 8)
    )
    return kiss_encode(struct.pack(">I", word) + payload)


def _detect_port() -> str | None:
    for pattern in ("/dev/tty.usbserial-*", "/dev/tty.wchusbserial-*", "/dev/ttyUSB*"):
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def _elf_to_bin(elf_path: str) -> str:
    fd, out_path = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    result = subprocess.run(
        ["espflash", "save-image", "--chip", "esp32p4", elf_path, out_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print("espflash error:", result.stderr.strip())
        sys.exit(1)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("binary", help="Firmware .bin or ELF file")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=115_200)
    parser.add_argument(
        "--chunk", type=int, default=512, help="Bytes per DATA packet (default 512)"
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.02,
        help="Seconds between packets (default 0.02)",
    )
    args = parser.parse_args()

    bin_path = args.binary
    with open(bin_path, "rb") as f:
        if f.read(4) == b"\x7fELF":
            print("ELF detected — converting via espflash save-image...")
            bin_path = _elf_to_bin(bin_path)

    with open(bin_path, "rb") as f:
        firmware = f.read()

    port = args.port or _detect_port()
    if port is None:
        print("ERROR: no serial port found — pass one explicitly")
        sys.exit(1)

    size = len(firmware)
    n_chunks = (size + args.chunk - 1) // args.chunk
    eta_s = n_chunks * args.delay

    print(f"Port     : {port} @ {args.baud} baud")
    print(f"Firmware : {args.binary}  ({size:,} bytes)")
    print(f"Chunks   : {n_chunks} × {args.chunk} B  ({args.delay * 1000:.0f} ms gap)")
    print(f"ETA      : ~{eta_s:.0f} s  ({eta_s / 60:.1f} min)")
    print()

    ser = serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )
    time.sleep(0.1)

    def send(pkt: bytes) -> None:
        ser.write(pkt)
        ser.flush()
        time.sleep(args.delay)

    send(csp_packet(bytes([CMD_ANNOUNCE])))
    print("[ANNC ] announced")

    send(csp_packet(bytes([CMD_BEGIN]) + struct.pack("<I", size)))
    print(f"[BEGIN] {size:,} bytes")

    t0 = time.monotonic()
    offset = 0
    for _ in range(n_chunks):
        chunk = firmware[offset : offset + args.chunk]
        payload = bytes([CMD_DATA]) + struct.pack("<I", offset) + chunk
        send(csp_packet(payload))

        offset += len(chunk)
        elapsed = time.monotonic() - t0
        pct = offset / size * 100
        eta = (elapsed / (pct / 100) - elapsed) if pct > 0 else 0
        print(
            f"\r[DATA ] {offset:>7,}/{size:,}  {pct:5.1f}%  {elapsed:4.0f}s elapsed  eta {eta:4.0f}s",
            end="",
            flush=True,
        )

    print()

    send(csp_packet(bytes([CMD_END])))
    print(f"[END  ] sent — ESP32 will verify and reboot")

    ser.close()


if __name__ == "__main__":
    main()
