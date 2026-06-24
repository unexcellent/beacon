#!/usr/bin/env python3
"""
Flash firmware to the ESP32 (node 7 / SSTV) via the CAN bus.

The payload board (node 14) accepts CAN frames destined for node 7 and
forwards them over its RS485 link to the ESP32.

Protocol (CSP port 10):
  ANNOUNCE (0x00): [cmd][chunk_size: u16 LE]
  BEGIN    (0x01): [cmd][total_size: u32 LE]
  DATA     (0x02): [cmd][offset: u32 LE][data...]
  END      (0x03): [cmd]

All commands are fire-and-forget; the script does not wait for replies.

The CAN interface must be up before running:
    sudo ip link set can0 up type can bitrate 1000000

Usage:
    python3 update_via_can.py --image firmware.bin
    python3 update_via_can.py --can can0 --chunk 512 --image firmware.bin
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

# ── SocketCAN raw frame ───────────────────────────────────────────────────────

CAN_EFF_FLAG = 0x80000000
CAN_FRAME_FMT = "=IB3x8s"
CAN_DLC_MAX = 8

# ── CFP v1 field layout within the 29-bit extended CAN ID ────────────────────
#   bits 28-24: src    (5)
#   bits 23-19: dst    (5)
#   bit  18:    type   (0=BEGIN, 1=MORE)
#   bits 17-10: remain (8)
#   bits  9-0:  ident  (10)
#
# BEGIN frame data layout:
#   bytes 0-3: CSP 1.x header (big-endian u32)
#   bytes 4-5: payload length (big-endian u16)
#   bytes 6-7: first 2 bytes of payload

CFP_SRC_SHIFT = 24
CFP_DST_SHIFT = 19
CFP_TYPE_SHIFT = 18
CFP_REMAIN_SHIFT = 10
CFP_BEGIN = 0
CFP_MORE = 1

CFP1_DATA_OFFSET = 6
CFP1_BEGIN_DATA = 2

# ── CSP / OTA constants ───────────────────────────────────────────────────────

CSP_PRIO = 2
CSP_SRC_DEFAULT = 0
CSP_DST_DEFAULT = 7  # WIRE_NODE_SSTV / ESP32
CSP_SPORT = 63  # ephemeral source port

OTA_PORT = 10
CMD_ANNOUNCE = 0x00
CMD_BEGIN = 0x01
CMD_DATA = 0x02
CMD_END = 0x03


# ── CFP v1 framing ────────────────────────────────────────────────────────────


def _cfp_id(src: int, dst: int, ftype: int, remain: int, ident: int = 0) -> int:
    return (
        ((src & 0x1F) << CFP_SRC_SHIFT)
        | ((dst & 0x1F) << CFP_DST_SHIFT)
        | ((ftype & 0x01) << CFP_TYPE_SHIFT)
        | ((remain & 0xFF) << CFP_REMAIN_SHIFT)
        | (ident & 0x3FF)
    )


def _csp1_header(src: int, dst: int, dport: int, sport: int) -> bytes:
    word = (
        ((CSP_PRIO & 0x03) << 30)
        | ((src & 0x1F) << 25)
        | ((dst & 0x1F) << 20)
        | ((dport & 0x3F) << 14)
        | ((sport & 0x3F) << 8)
    )
    return struct.pack(">I", word)


def build_frames(
    src: int,
    dst: int,
    dport: int,
    sport: int,
    payload: bytes,
    ident: int = 0,
) -> list[tuple[int, bytes]]:
    """Return list of (cfp_can_id, data_bytes) for one CFP v1 CSP packet."""
    csp_hdr = _csp1_header(src, dst, dport, sport)
    length = len(payload)
    total_remain = (length + CFP1_DATA_OFFSET - 1) // CAN_DLC_MAX

    frames: list[tuple[int, bytes]] = []
    first_chunk = payload[:CFP1_BEGIN_DATA]
    begin_data = csp_hdr + struct.pack(">H", length) + first_chunk
    frames.append((_cfp_id(src, dst, CFP_BEGIN, total_remain, ident), begin_data))

    tx = len(first_chunk)
    while tx < length:
        chunk = payload[tx : tx + CAN_DLC_MAX]
        tx += len(chunk)
        remain = (length - tx + CAN_DLC_MAX - 1) // CAN_DLC_MAX
        frames.append((_cfp_id(src, dst, CFP_MORE, remain, ident), chunk))

    return frames


def send_csp(
    sock: socket.socket,
    src: int,
    dst: int,
    dport: int,
    sport: int,
    payload: bytes,
    ident: int = 0,
) -> None:
    for cfp_id, data in build_frames(src, dst, dport, sport, payload, ident):
        raw = struct.pack(
            CAN_FRAME_FMT,
            cfp_id | CAN_EFF_FLAG,
            len(data),
            data.ljust(CAN_DLC_MAX, b"\x00"),
        )
        sock.send(raw)


# ── ELF / binary helpers ──────────────────────────────────────────────────────


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


# ── Main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-i",
        "--image",
        required=True,
        metavar="FILE",
        help="Firmware .bin or ELF file",
    )
    parser.add_argument(
        "--can",
        default="can0",
        help="SocketCAN interface (default: can0)",
    )
    parser.add_argument(
        "--src",
        type=int,
        default=CSP_SRC_DEFAULT,
        help=f"Source CSP node ID (default {CSP_SRC_DEFAULT})",
    )
    parser.add_argument(
        "--dst",
        type=int,
        default=CSP_DST_DEFAULT,
        help=f"Target CSP node ID (default {CSP_DST_DEFAULT} = SSTV/ESP32)",
    )
    parser.add_argument(
        "--chunk",
        type=int,
        default=512,
        help="Bytes per DATA packet (default 512)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.05,
        help="Seconds between packets (default 0.05)",
    )
    args = parser.parse_args()

    bin_path = args.image
    with open(bin_path, "rb") as f:
        magic = f.read(4)
    if magic == b"\x7fELF":
        print("ELF detected — converting via espflash save-image...")
        bin_path = _elf_to_bin(bin_path)

    with open(bin_path, "rb") as f:
        firmware = f.read()

    size = len(firmware)
    n_chunks = (size + args.chunk - 1) // args.chunk

    print(f"Interface: {args.can}")
    print(f"Firmware : {args.image}  ({size:,} bytes)")
    print(f"Target   : CSP node {args.dst}, port {OTA_PORT}")
    print(f"Chunks   : {n_chunks} x {args.chunk} B")
    print()

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)  # type: ignore[attr-defined]
        sock.bind((args.can,))
    except OSError as e:
        print(f"ERROR: cannot open {args.can}: {e}")
        print(
            f"Make sure the interface is up: "
            f"sudo ip link set {args.can} up type can bitrate 1000000"
        )
        sys.exit(1)

    def send_ff(payload: bytes) -> None:
        """Send a fire-and-forget OTA command (no reply expected)."""
        send_csp(sock, args.src, args.dst, OTA_PORT, CSP_SPORT, payload)
        time.sleep(args.delay)

    # 1. ANNOUNCE: chunk size so the ESP32 can detect truncated packets
    send_ff(bytes([CMD_ANNOUNCE]) + struct.pack("<H", args.chunk))
    print(f"[ANNC ] announced (chunk_size={args.chunk})")
    time.sleep(0.2)

    # 2. BEGIN: total firmware size
    send_ff(bytes([CMD_BEGIN]) + struct.pack("<I", size))
    print(f"[BEGIN] {size:,} bytes")
    time.sleep(0.2)

    # 3. DATA: one copy per chunk
    t0 = time.monotonic()
    offset = 0

    for _ in range(n_chunks):
        chunk = firmware[offset : offset + args.chunk]
        payload = bytes([CMD_DATA]) + struct.pack("<I", offset) + chunk
        send_ff(payload)

        done = offset + len(chunk)
        elapsed = time.monotonic() - t0
        pct = done / size * 100
        if done > 0 and pct < 100:
            eta_s = elapsed / done * (size - done)
            h, rem = divmod(int(eta_s), 3600)
            m, s = divmod(rem, 60)
            eta = f"ETA {h:02d}:{m:02d}:{s:02d}"
        else:
            eta = "         "
        if len(payload) <= 32:
            payload_repr = " ".join(f"{b:02x}" for b in payload)
        else:
            payload_repr = f"{len(payload)} bytes"
        print(f"[DATA ] {pct:5.1f}%  {eta}  {payload_repr}")
        offset += len(chunk)

    # 4. END: trigger verify + reboot
    print("[END  ] Waiting for ESP32 flash write to complete...")
    time.sleep(5)

    end_payload = bytes([CMD_END])

    # Send it 4 times with a significant gap to prevent Zephyr from clustering them
    # and overflowing the ESP32's 128-byte RX FIFO.
    for _ in range(10):
        send_ff(end_payload)
        payload_repr = " ".join(f"{b:02x}" for b in end_payload)
        print(f"[END ] {payload_repr}")
        time.sleep(1)

    print("[END  ] sent — ESP32 will verify and reboot")

    time.sleep(3)
    sock.close()


if __name__ == "__main__":
    main()
