#!/usr/bin/env python3
"""
Send a test RS485 message via the DSD TECH SH-U11 USB adapter.

The SH-U11 handles direction switching automatically, so no RTS toggling
is needed — just open the port and write.

Usage:
    python3 ping.py [port] [--baud N] [--message TEXT]
"""

import argparse
import glob
import sys
import time

import serial


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
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--message", default="PING", help="Message to send (default: PING)")
    args = parser.parse_args()

    port = args.port or _detect_port()
    if port is None:
        print("ERROR: no serial port found — pass one explicitly")
        sys.exit(1)

    payload = args.message.encode()

    ser = serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )

    time.sleep(0.1)  # let the adapter settle after open

    ser.write(payload)
    ser.flush()

    print(f"Sent {len(payload)} byte(s) on {port} @ {args.baud} baud: {payload!r}")

    ser.close()


if __name__ == "__main__":
    main()
