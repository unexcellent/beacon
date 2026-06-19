#!/usr/bin/env python3
"""
Send a CSP ping (port 1) to the payload board (node 14) via CAN/SocketCAN
and wait for the pong reply. Uses CFP v1 framing (libcsp version 1).

The CAN interface must be up before running:
    sudo ip link set can0 up type can bitrate 250000

Usage:
    python3 ping_payload.py [--iface IFACE] [--src N] [--dst N] [--timeout S]
"""

import argparse
import socket
import struct
import sys
import time

# SocketCAN raw frame: can_id(4) + can_dlc(1) + pad(3) + data(8)
CAN_EFF_FLAG   = 0x80000000
CAN_ERR_FLAG   = 0x20000000
CAN_RTR_FLAG   = 0x40000000
CAN_EFF_MASK   = 0x1FFFFFFF
CAN_FRAME_FMT  = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)  # 16
CAN_DLC_MAX    = 8

# CSP
CSP_PING_PORT   = 1
CSP_SPORT       = 63   # ephemeral source port
CSP_PRIO        = 2
CSP_SRC_DEFAULT = 0    # ground / Pi node
CSP_DST_DEFAULT = 14   # WIRE_NODE_PAYLOAD
PING_PAYLOAD    = b"PING"

# CFP v1 field layout within the 29-bit extended CAN ID
#   bits 28-24: src (5)
#   bits 23-19: dst (5)
#   bit  18:    type  (0=BEGIN, 1=MORE)
#   bits 17-10: remain (8)
#   bits  9-0:  ident (10)
CFP_SRC_SHIFT    = 24
CFP_DST_SHIFT    = 19
CFP_TYPE_SHIFT   = 18
CFP_REMAIN_SHIFT = 10
CFP_BEGIN        = 0
CFP_MORE         = 1

# CFP v1 data layout for a BEGIN frame
#   bytes 0-3: CSP 1.x header  (big-endian u32)
#   bytes 4-5: payload length  (big-endian u16)
#   bytes 6-7: first 2 bytes of payload
CFP1_HDR_SIZE    = 4
CFP1_LEN_SIZE    = 2
CFP1_DATA_OFFSET = 6   # header + length
CFP1_BEGIN_DATA  = 2   # max payload bytes in the BEGIN frame


# ---------------------------------------------------------------------------
# Framing helpers
# ---------------------------------------------------------------------------

def _cfp_id(src, dst, ftype, remain, ident=0):
    return (
        ((src    & 0x1F) << CFP_SRC_SHIFT)    |
        ((dst    & 0x1F) << CFP_DST_SHIFT)    |
        ((ftype  & 0x01) << CFP_TYPE_SHIFT)   |
        ((remain & 0xFF) << CFP_REMAIN_SHIFT) |
        (ident   & 0x3FF)
    )


def _csp1_header(src, dst, dport, sport):
    word = (
        ((CSP_PRIO & 0x03) << 30) |
        ((src      & 0x1F) << 25) |
        ((dst      & 0x1F) << 20) |
        ((dport    & 0x3F) << 14) |
        ((sport    & 0x3F) <<  8)
    )
    return struct.pack(">I", word)


def _parse_cfp(can_id):
    return {
        "src":    (can_id >> CFP_SRC_SHIFT)    & 0x1F,
        "dst":    (can_id >> CFP_DST_SHIFT)    & 0x1F,
        "type":   (can_id >> CFP_TYPE_SHIFT)   & 0x01,
        "remain": (can_id >> CFP_REMAIN_SHIFT) & 0xFF,
        "ident":   can_id                       & 0x3FF,
    }


def _parse_csp1(data):
    if len(data) < CFP1_HDR_SIZE:
        return None
    word, = struct.unpack(">I", data[:4])
    return {
        "src":   (word >> 25) & 0x1F,
        "dst":   (word >> 20) & 0x1F,
        "dport": (word >> 14) & 0x3F,
        "sport": (word >>  8) & 0x3F,
        "flags":  word        & 0xFF,
    }


def build_frames(src, dst, dport, sport, payload, ident=0):
    """Return list of (cfp_can_id, data_bytes) for one CFP v1 CSP packet."""
    csp_hdr = _csp1_header(src, dst, dport, sport)
    length  = len(payload)
    # Number of MORE frames that follow the BEGIN frame
    total_remain = (length + CFP1_DATA_OFFSET - 1) // CAN_DLC_MAX

    frames = []

    first_chunk = payload[:CFP1_BEGIN_DATA]
    begin_data  = csp_hdr + struct.pack(">H", length) + first_chunk
    frames.append((_cfp_id(src, dst, CFP_BEGIN, total_remain, ident), begin_data))

    tx = len(first_chunk)
    while tx < length:
        chunk   = payload[tx : tx + CAN_DLC_MAX]
        tx     += len(chunk)
        remain  = (length - tx + CAN_DLC_MAX - 1) // CAN_DLC_MAX
        frames.append((_cfp_id(src, dst, CFP_MORE, remain, ident), chunk))

    return frames


def send_frame(sock, cfp_id, data):
    raw = struct.pack(CAN_FRAME_FMT,
                      cfp_id | CAN_EFF_FLAG,
                      len(data),
                      data.ljust(CAN_DLC_MAX, b'\x00'))
    sock.send(raw)


# ---------------------------------------------------------------------------
# Receive / reassemble
# ---------------------------------------------------------------------------

def recv_pong(sock, our_node, peer_node, timeout):
    """
    Reassemble CFP v1 frames until a pong arrives from peer_node to our_node
    (dport == CSP_SPORT). Returns the pong payload bytes, or None on timeout.
    """
    # key: (src, dst, ident) → {"csp": ..., "length": N, "buf": bytearray, "rx": N}
    pending  = {}
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        sock.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            raw = sock.recv(CAN_FRAME_SIZE)
        except (socket.timeout, OSError):
            continue

        if len(raw) < CAN_FRAME_SIZE:
            continue

        raw_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)

        if not (raw_id & CAN_EFF_FLAG):
            continue
        if raw_id & (CAN_ERR_FLAG | CAN_RTR_FLAG):
            continue

        can_id = raw_id & CAN_EFF_MASK
        f      = _parse_cfp(can_id)
        chunk  = data[:dlc]

        # Only track frames coming from the peer destined for us
        if f["src"] != peer_node or f["dst"] != our_node:
            continue

        key = (f["src"], f["dst"], f["ident"])

        if f["type"] == CFP_BEGIN:
            if dlc < CFP1_DATA_OFFSET:
                continue
            csp    = _parse_csp1(chunk)
            length, = struct.unpack(">H", chunk[CFP1_HDR_SIZE:CFP1_DATA_OFFSET])
            first  = chunk[CFP1_DATA_OFFSET:dlc]
            pending[key] = {"csp": csp, "length": length, "buf": bytearray(first), "rx": len(first)}
        elif key in pending:
            pending[key]["buf"].extend(chunk)
            pending[key]["rx"] += len(chunk)

        entry = pending.get(key)
        if entry and entry["rx"] >= entry["length"]:
            pending.pop(key)
            csp = entry["csp"]
            if csp and csp["dport"] == CSP_SPORT:
                return bytes(entry["buf"][:entry["length"]])

    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--iface",   default="can0",
                        help="SocketCAN interface (default: can0)")
    parser.add_argument("--src",     type=int, default=CSP_SRC_DEFAULT,
                        help=f"Our CSP node ID (default {CSP_SRC_DEFAULT})")
    parser.add_argument("--dst",     type=int, default=CSP_DST_DEFAULT,
                        help=f"Target CSP node ID (default {CSP_DST_DEFAULT} = payload)")
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="Seconds to wait for pong (default 3)")
    args = parser.parse_args()

    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.iface,))
    except OSError as e:
        print(f"ERROR: cannot open {args.iface}: {e}")
        print("Make sure the interface is up:  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)

    frames = build_frames(args.src, args.dst, CSP_PING_PORT, CSP_SPORT, PING_PAYLOAD)

    t_send = time.monotonic()
    for cfp_id, data in frames:
        send_frame(sock, cfp_id, data)

    print(f"→ PING  {args.src}:{CSP_SPORT} → {args.dst}:{CSP_PING_PORT}  "
          f"payload={PING_PAYLOAD!r}  ({len(frames)} CAN frame(s))  [{args.iface}]")

    result = recv_pong(sock, args.src, args.dst, args.timeout)
    sock.close()

    if result is not None:
        rtt_ms = (time.monotonic() - t_send) * 1000
        print(f"← PONG  {args.dst} → {args.src}  payload={result!r}  rtt={rtt_ms:.1f} ms")
        sys.exit(0)
    else:
        print(f"✗ no pong received within {args.timeout:.1f} s")
        sys.exit(1)


if __name__ == "__main__":
    main()
