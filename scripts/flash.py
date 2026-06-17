#!/usr/bin/env python3
"""
Flash a firmware binary to the ESP32 via KISS-framed CSP OTA (dport=10).

Protocol:
  BEGIN (0x01): 4-byte total size (LE)
  DATA  (0x02): 4-byte offset (LE) + up to CHUNK_SIZE bytes
  END   (0x03): triggers esp_ota_end + reboot

Usage:
    python3 flash_ota.py <firmware.bin> [port] [--baud N] [--chunk N] [--delay S]
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

_ELF_MAGIC = b"\x7fELF"
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
_ESPTOOL = os.path.join(
    _PROJECT_ROOT,
    ".embuild/espressif/python_env/idf5.5_py3.13_env/bin/esptool.py",
)
_PYTHON = os.path.join(
    _PROJECT_ROOT,
    ".embuild/espressif/python_env/idf5.5_py3.13_env/bin/python3",
)


def _elf_to_bin(elf_path: str) -> str:
    """Convert an ELF to a flat .bin via esptool; return the .bin path."""
    out = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    out.close()
    cmd = [
        _PYTHON,
        _ESPTOOL,
        "--chip",
        "esp32p4",
        "elf2image",
        "--output",
        out.name,
        elf_path,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("esptool error:", result.stderr.strip())
        sys.exit(1)
    return out.name


# CSP addressing
_CSP_PRIO = 2
_CSP_SRC = 0
_CSP_DST = 7
_CSP_DPORT = 10  # OTA service port
_CSP_SPORT = 63

# OTA command bytes
CMD_ANNOUNCE = 0x00
CMD_BEGIN = 0x01
CMD_DATA = 0x02
CMD_END = 0x03

# KISS constants
FEND = 0xC0
FESC = 0xDB
TFEND = 0xDC
TFESC = 0xDD


def _kiss_frame(data: bytes) -> bytes:
    escaped = bytearray()
    for b in data:
        if b == FEND:
            escaped.extend([FESC, TFEND])
        elif b == FESC:
            escaped.extend([FESC, TFESC])
        else:
            escaped.append(b)
    return bytes([FEND, 0x00]) + bytes(escaped) + bytes([FEND])


def _csp_header() -> bytes:
    word = (
        ((_CSP_PRIO & 0x03) << 30)
        | ((_CSP_SRC & 0x1F) << 25)
        | ((_CSP_DST & 0x1F) << 20)
        | ((_CSP_DPORT & 0x3F) << 14)
        | ((_CSP_SPORT & 0x3F) << 8)
    )
    return struct.pack(">I", word)


def _packet(payload: bytes) -> bytes:
    return _kiss_frame(_csp_header() + payload)


def _detect_port() -> str | None:
    candidates = (
        glob.glob("/dev/tty.usbserial-*")
        + glob.glob("/dev/tty.wchusbserial-*")
        + glob.glob("/dev/ttyUSB*")
    )
    return candidates[0] if candidates else None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("binary", help="Firmware ELF or .bin file")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--chunk", type=int, default=128, help="Bytes per DATA packet (default 128)"
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.02,
        help="Seconds between packets (default 0.2); must exceed "
        "chunk_bytes/baud + flash write time",
    )
    args = parser.parse_args()

    port = args.port or _detect_port()
    if port is None:
        print("ERROR: no serial port found — pass one explicitly")
        sys.exit(1)

    bin_path = args.binary
    with open(bin_path, "rb") as f:
        header = f.read(4)
    if header == _ELF_MAGIC:
        print(f"ELF detected — converting via esptool...")
        bin_path = _elf_to_bin(bin_path)

    with open(bin_path, "rb") as f:
        firmware = f.read()

    size = len(firmware)
    n_chunks = (size + args.chunk - 1) // args.chunk
    tx_ms = args.chunk * 10 / args.baud * 1000
    total_s = n_chunks * (tx_ms / 1000 + args.delay)

    print(f"Port     : {port} @ {args.baud} baud")
    print(f"Firmware : {args.binary}  ({size:,} bytes)")
    print(f"Chunks   : {n_chunks} × {args.chunk} bytes  (~{tx_ms:.0f} ms tx each)")
    print(f"Delay    : {args.delay * 1000:.0f} ms between packets")
    print(f"ETA      : ~{total_s:.0f} s  ({total_s / 60:.1f} min)")
    print()

    ser = serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
        rtscts=False,
    )
    ser.rts = False

    def _send(data: bytes) -> None:
        ser.rts = True
        ser.write(data)
        ser.flush()  # tcdrain — blocks until bytes leave the wire
        ser.rts = False

    # Let the RS485 line settle before the first packet.
    time.sleep(0.5)
    t0 = time.monotonic()

    # Announce the incoming update so the node can log/prepare.
    _send(_packet(bytes([CMD_ANNOUNCE])))
    print("[ANNC ] Update announced")
    time.sleep(args.delay)

    # Send BEGIN twice so a single corrupted copy doesn't stall the session.
    begin_pkt = _packet(bytes([CMD_BEGIN]) + struct.pack("<I", size))
    for attempt in range(1, 3):
        _send(begin_pkt)
        print(f"[BEGIN] {size} bytes (attempt {attempt}/2)")
        time.sleep(args.delay)

    # DATA
    offset = 0
    for _ in range(n_chunks):
        chunk = firmware[offset : offset + args.chunk]
        payload = bytes([CMD_DATA]) + struct.pack("<I", offset) + chunk
        _send(_packet(payload))

        elapsed = time.monotonic() - t0
        pct = (offset + len(chunk)) / size * 100
        eta = elapsed / (pct / 100) - elapsed if pct > 0 else 0
        print(
            f"\r[DATA ] {offset + len(chunk):>7,}/{size:,} bytes  "
            f"{pct:5.1f}%  elapsed {elapsed:5.0f}s  eta {eta:5.0f}s",
            end="",
            flush=True,
        )

        offset += len(chunk)
        time.sleep(args.delay)

    print()

    # END
    _send(_packet(bytes([CMD_END])))
    elapsed = time.monotonic() - t0
    print(f"[END  ] sent after {elapsed:.1f}s — waiting for reboot...")

    ser.close()


if __name__ == "__main__":
    main()
