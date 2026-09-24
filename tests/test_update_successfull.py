#!/usr/bin/env python3
"""Hardware-in-the-loop OTA test: push a firmware image over the payload link and
confirm the ESP32 reboots into it.

Drives the beacon's own update protocol (ANNOUNCE/BEGIN/DATA/END on CSP port 10,
see src/link/command.rs + src/update.rs) directly over RS422 — the same path the
real system uses, and the FTDI RS422 adapter survives the ESP reboot (the
USB-Serial-JTAG console does not), so no USB-C is needed.

The reboot itself is the confirmation: the ESP only reboots (sending a fresh
BOOTED status) after esp_ota_end validates the received image. A failed or short
transfer makes esp_ota_end fail and downlinks an `Update*` error WITHOUT
rebooting, so seeing BOOTED after END means the update took.

Run standalone:  ./.venv/bin/python tests/test_update_successfull.py
Run via pytest:  ./.venv/bin/pytest tests/test_update_successfull.py -s

Env overrides (skip/redirect the build):
    BEACON_OTA_IMAGE   path to a prebuilt app image (.bin) to send as-is
    BEACON_OTA_ELF     ELF to save-image instead of building
    BEACON_OTA_PROFILE cargo profile to build (default: release)
"""

from __future__ import annotations

import logging
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import drain, run_case  # noqa: E402
from tests.util.payload_board import MockPayloadBoard  # noqa: E402

log = logging.getLogger("ota_test")

REPO = Path(__file__).resolve().parent.parent

# DATA payload size — 128 B, matching the CAN reference sender. Larger chunks
# (256 B) proved unreliable over this RS422 link: a dropped/garbled byte in a
# long frame desyncs the ESP's KISS decoder and truncates the chunk.
CHUNK = 128
CHUNK_DELAY = 0.025  # seconds between DATA frames
BOOT_TIMEOUT = 20.0  # seconds to wait for the post-update reboot


def test_update_successful(board: MockPayloadBoard) -> None:
    image = build_firmware()

    send_update_announcement(board)
    send_update_begin(board, len(image))
    send_update_chunks(board, image)
    send_update_end(board)
    success = receive_boot_info(board)

    assert success, "ESP did not reboot into the new firmware"


def build_firmware() -> bytes:
    """Compile the firmware and return the ESP app image bytes.

    Uses BEACON_OTA_IMAGE (a prebuilt .bin) as-is if set, else `espflash
    save-image` of a freshly built (or BEACON_OTA_ELF) ELF.
    """
    prebuilt = os.environ.get("BEACON_OTA_IMAGE")
    if prebuilt:
        log.info("using prebuilt image %s", prebuilt)
        return Path(prebuilt).read_bytes()

    elf = os.environ.get("BEACON_OTA_ELF")
    if not elf:
        profile = os.environ.get("BEACON_OTA_PROFILE", "release")
        flag = [] if profile == "debug" else [f"--{profile}"]  # debug is `cargo build`
        log.info("building firmware (cargo build %s) ...", " ".join(flag))
        subprocess.run(["cargo", "build", *flag], cwd=REPO, check=True)
        elf = REPO / f"target/riscv32imafc-esp-espidf/{profile}/beacon"

    out = Path(tempfile.gettempdir()) / "beacon_ota_app.bin"
    log.info("converting %s -> app image via espflash save-image", elf)
    subprocess.run(
        [
            "espflash",
            "save-image",
            "--chip",
            "esp32p4",
            "-s",
            "16mb",
            str(elf),
            str(out),
        ],
        check=True,
        capture_output=True,
    )
    return out.read_bytes()


def send_update_announcement(board: MockPayloadBoard) -> None:
    drain(board)  # start the session clean: drop any stale frames (e.g. a startup BOOTED)
    board.update_announce(CHUNK)
    time.sleep(0.2)


def send_update_begin(board: MockPayloadBoard, total: int) -> None:
    board.update_begin(total)
    time.sleep(0.2)


def send_update_chunks(board: MockPayloadBoard, image: bytes) -> None:
    chunks = [image[i : i + CHUNK] for i in range(0, len(image), CHUNK)]
    log.info(
        "transmitting %d bytes in %d chunks of %d B", len(image), len(chunks), CHUNK
    )
    for idx, data in enumerate(chunks):
        board.update_data(idx * CHUNK, data)
        time.sleep(CHUNK_DELAY)
        if idx % 400 == 0 or idx == len(chunks) - 1:
            log.info(
                "  %d/%d chunks (%d%%)",
                idx + 1,
                len(chunks),
                (idx + 1) * 100 // len(chunks),
            )


def send_update_end(board: MockPayloadBoard) -> None:
    board.update_end()
    log.info("END sent; awaiting reboot")


def receive_boot_info(board: MockPayloadBoard) -> bool:
    """True once the ESP reboots (fresh BOOTED status). False if it downlinks an
    `Update*` error instead (a failed update never reboots) or nothing reboots."""
    deadline = time.monotonic() + BOOT_TIMEOUT
    while time.monotonic() < deadline:
        pkt = board.receive(timeout=deadline - time.monotonic())
        if pkt is None:
            continue
        if pkt.dst == 1 and pkt.payload.startswith(b"STATUS: BOOTED"):
            log.info("rebooted: %s", pkt.payload.decode("ascii", "replace"))
            return True
        if pkt.dst == 2 and pkt.payload.startswith(b"Update"):
            log.error("update error: %s", pkt.payload.decode("ascii", "replace"))
            return False
    log.error("no reboot within %.0fs after END", BOOT_TIMEOUT)
    return False


if __name__ == "__main__":
    sys.exit(run_case(test_update_successful))
