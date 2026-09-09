#!/usr/bin/env python3
"""Hardware-in-the-loop OTA test: push a firmware image over the payload link and
confirm the ESP32 reboots into it.

This drives the beacon's own update protocol (ANNOUNCE/BEGIN/DATA/END on CSP
port 10, see src/link/command.rs + src/update.rs) directly over RS422, instead
of the CAN ground path in local/update_via_can.sh. Flow:

  1. build the firmware and convert the ELF to an ESP app image (espflash save-image)
  2. confirm the ESP is reachable (CSP ping over the payload link)
  3. run the update twice (ANNOUNCE -> BEGIN -> DATA... -> END each time); the ESP
     writes the inactive OTA partition, validates it (esp_ota_end), switches the
     boot partition and reboots (esp_restart) on each
  4. after each update, read the BOOTED status over RS485 and check the running
     firmware's ELF SHA256 matches the transmitted image
  5. assert the running OTA partition FLIPPED between the two updates

Verification rationale: everything goes over the payload link (the FTDI RS422
adapter survives the ESP reboot; the USB-Serial-JTAG console does not, and this
test needs no USB-C at all). The BOOTED status carries the running firmware's
ELF SHA256 and OTA partition. Two things confirm the new software is running:
  - SHA256 match: the running firmware is the exact image we transmitted.
  - partition flip across two updates: esp_ota_write always targets the
    *inactive* partition and boots it, so each working update flips the running
    partition (ota_0<->ota_1). A different partition after the 2nd update than
    after the 1st proves the update reboots into the freshly-written slot — even
    when the image is byte-identical (same SHA), which a hash check can't tell.
A corrupt/short transfer makes esp_ota_end fail and downlink an `Update*` error
WITHOUT rebooting, so a fresh BOOTED after END already implies a good image.

The real-system update path is exactly this ANNOUNCE/BEGIN/DATA/END -> esp_ota
-> reboot sequence; running it twice is only how the *test* observes the flip
without a separate reset channel, not a change to how updates work.

Run standalone (verbose):
    ./.venv/bin/python tests/test_update_successfull.py
    ./.venv/bin/python tests/test_update_successfull.py --chunk 256 --image /tmp/app.bin
Run via pytest:
    ./.venv/bin/pytest tests/test_update_successfull.py -s

Env overrides (skip/redirect the build):
    BEACON_OTA_IMAGE   path to a prebuilt app image (.bin) to send as-is
    BEACON_OTA_ELF     ELF to save-image instead of building
    BEACON_OTA_PROFILE cargo profile to build (default: release)
"""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import (  # noqa: E402
    SKIP_EXIT,
    Skipped,
    detect_port_or_skip,
    setup_logging,
    wait_reachable,
)
from tests.util.payload_board import MockPayloadBoard  # noqa: E402

log = logging.getLogger("ota_test")

# Default DATA payload size — 128 B, matching the CAN reference sender
# (ground/updater update_sstv.py). Larger chunks (256 B) proved unreliable over
# this RS422 link: a single dropped/garbled byte in a long frame desyncs the
# ESP's KISS decoder so a later firmware byte reads as a frame delimiter,
# truncating the chunk, and the protocol aborts on the resulting offset gap.
DEFAULT_CHUNK = 128
REPO = Path(__file__).resolve().parent.parent
RELEASE_ELF = REPO / "target/riscv32imafc-esp-espidf/release/beacon"


# The ESP app image embeds an esp_app_desc_t; its app_elf_sha256 is what the
# firmware reports via esp_app_get_description() and appends to its BOOTED
# status. The descriptor starts with this magic and the sha sits at a fixed
# offset from it, so we can read the expected hash straight out of the image.
APP_DESC_MAGIC = b"\x32\x54\xcd\xab"  # 0xABCD5432, little-endian
APP_ELF_SHA256_OFFSET = 144  # bytes from the magic to app_elf_sha256 in esp_app_desc_t


class UpdateFailed(Exception):
    """The ESP reported an Update* error or never rebooted into the new image."""


def image_elf_sha256(image: bytes) -> str:
    """Read app_elf_sha256 (hex) out of the app image's descriptor."""
    magic = image.find(APP_DESC_MAGIC, 0, 256)
    if magic < 0:
        raise UpdateFailed("app-descriptor magic not found — not an ESP app image?")
    return image[magic + APP_ELF_SHA256_OFFSET : magic + APP_ELF_SHA256_OFFSET + 32].hex()


def parse_booted(booted: bytes) -> tuple[str, str | None]:
    """Return (elf_sha256_hex, ota_partition) from a BOOTED status
    ("STATUS: BOOTED <version> <sha256hex> [<partition>]"). The partition is
    None for firmware that predates the partition field."""
    tokens = booted.decode("ascii", "replace").split()
    for i, tok in enumerate(tokens):
        if len(tok) == 64 and all(c in "0123456789abcdef" for c in tok):
            partition = tokens[i + 1] if i + 1 < len(tokens) else None
            return tok, partition
    raise UpdateFailed(f"BOOTED status carries no firmware hash: {booted!r}")


def build_ota_image() -> bytes:
    """Return the app image bytes to transmit: a prebuilt one if BEACON_OTA_IMAGE
    is set, otherwise `espflash save-image` of a freshly built (or given) ELF."""
    prebuilt = os.environ.get("BEACON_OTA_IMAGE")
    if prebuilt:
        log.info("using prebuilt image %s", prebuilt)
        return Path(prebuilt).read_bytes()

    elf = os.environ.get("BEACON_OTA_ELF")
    if not elf:
        profile = os.environ.get("BEACON_OTA_PROFILE", "release")
        flag = [] if profile == "debug" else [f"--{profile}"]  # debug is `cargo build` with no flag
        log.info("building firmware (cargo build %s) ...", " ".join(flag))
        subprocess.run(["cargo", "build", *flag], cwd=REPO, check=True)
        elf = REPO / f"target/riscv32imafc-esp-espidf/{profile}/beacon"

    out = Path(tempfile.gettempdir()) / "beacon_ota_app.bin"
    log.info("converting %s -> app image via espflash save-image", elf)
    subprocess.run(
        ["espflash", "save-image", "--chip", "esp32p4", "-s", "16mb", str(elf), str(out)],
        check=True,
        capture_output=True,
    )
    return out.read_bytes()


def transmit_update(board: MockPayloadBoard, image: bytes, chunk: int, delay: float) -> None:
    """Send the full image via ANNOUNCE/BEGIN/DATA.../END, aborting if the ESP
    downlinks an Update* error mid-transfer."""
    chunks = [image[i : i + chunk] for i in range(0, len(image), chunk)]
    log.info("transmitting %d bytes in %d chunks of %d B", len(image), len(chunks), chunk)

    board.update_announce(chunk)
    time.sleep(0.2)
    board.update_begin(len(image))
    time.sleep(0.2)
    for idx, data in enumerate(chunks):
        board.update_data(idx * chunk, data)
        if delay:
            time.sleep(delay)
        if idx % 400 == 0 or idx == len(chunks) - 1:
            log.info("  %d/%d chunks (%d%%)", idx + 1, len(chunks), (idx + 1) * 100 // len(chunks))
        _abort_on_update_error(board)
    board.update_end()
    log.info("END sent; awaiting reboot")


def _abort_on_update_error(board: MockPayloadBoard) -> None:
    """Drain pending frames; raise if the ESP downlinked an OTA error. Boot-time
    camera errors (RgbInit/ThermalInit) are ignored — only `Update*` counts."""
    while (pkt := board.receive(timeout=0)) is not None:
        if pkt.dst == 2 and pkt.payload.startswith(b"Update"):
            raise UpdateFailed(pkt.payload.decode("ascii", "replace"))


def wait_for_reboot(board: MockPayloadBoard, timeout: float = 20.0) -> bytes:
    """Wait for a fresh BOOTED status (proof the ESP restarted). Raise if an
    Update* error arrives first, or nothing reboots in time."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pkt = board.receive(timeout=deadline - time.monotonic())
        if pkt is None:
            continue
        if pkt.dst == 2 and pkt.payload.startswith(b"Update"):
            raise UpdateFailed(pkt.payload.decode("ascii", "replace"))
        if pkt.dst == 1 and pkt.payload.startswith(b"STATUS: BOOTED"):
            return bytes(pkt.payload)
    raise UpdateFailed(f"no reboot within {timeout:.0f}s after END")


def _one_update(
    board: MockPayloadBoard, image: bytes, expected_sha: str, chunk: int, delay: float
) -> tuple[bytes, str | None]:
    """Run one full OTA cycle and return (BOOTED payload, running OTA partition).

    Verifies the transfer completed, the ESP rebooted, it is running the exact
    image we sent (SHA256 match) and is servicing the link again. The partition
    is None if this firmware predates the field.
    """
    while board.receive(timeout=0) is not None:  # drop stale frames
        pass
    transmit_update(board, image, chunk, delay)
    booted = wait_for_reboot(board)
    log.info("rebooted: %s", booted.decode("ascii", "replace"))
    running_sha, partition = parse_booted(booted)
    if running_sha != expected_sha:
        raise UpdateFailed(f"running firmware hash {running_sha} != transmitted {expected_sha}")
    log.info("firmware hash matches the transmitted image ✓")
    if not wait_reachable(board, timeout=12.0):
        raise UpdateFailed("ESP did not answer a ping after rebooting into the new image")
    return booted, partition


def run_ota_update(port: str | None = None, chunk: int = DEFAULT_CHUNK, delay: float = 0.025) -> bytes:
    """OTA the image, then OTA it a second time, and assert the running OTA
    partition flips between the two updates.

    Two consecutive updates are the payload-link-only (no USB-C) proof that the
    update actually reboots into the freshly-written slot: esp_ota_write always
    targets the *inactive* partition and boots it, so a working update flips
    ota_0<->ota_1. Observing the flip proves the new software is running even
    when the image is byte-identical to what was there before (same SHA256),
    which a hash check alone cannot establish.
    """
    port = port or detect_port_or_skip()  # skip (before the slow build) if no adapter
    image = build_ota_image()
    expected_sha = image_elf_sha256(image)
    log.info("transmitted image ELF SHA256: %s", expected_sha)

    with MockPayloadBoard(port) as board:
        time.sleep(0.3)
        if not wait_reachable(board, timeout=12.0):
            raise UpdateFailed("ESP not reachable over the payload link before update")
        log.info("ESP reachable; first update")
        _, part1 = _one_update(board, image, expected_sha, chunk, delay)

        log.info("first update done (partition %s); second update to prove the flip", part1)
        booted2, part2 = _one_update(board, image, expected_sha, chunk, delay)

        if part1 is not None and part2 is not None:
            if part1 == part2:
                raise UpdateFailed(
                    f"running partition did not change ({part2}) — the update is not booting the new slot"
                )
            log.info("running partition flipped %s -> %s across the two updates ✓", part1, part2)
        else:
            log.warning("firmware does not report its OTA partition — flip check skipped")
        log.info("ESP responsive on the new firmware")
        return booted2


def test_update_successful():
    """pytest entry point: OTA the firmware and assert the ESP reboots into it."""
    setup_logging()
    port = detect_port_or_skip()
    booted = run_ota_update(port)
    assert booted.startswith(b"STATUS: BOOTED")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="RS422 serial device (default: auto-detect)")
    parser.add_argument("-c", "--chunk", type=int, default=DEFAULT_CHUNK, help="DATA chunk size")
    parser.add_argument("-d", "--delay", type=float, default=0.025, help="seconds between chunks")
    parser.add_argument("--image", help="prebuilt app image to send (skips build)")
    args = parser.parse_args()

    setup_logging()
    if args.image:
        os.environ["BEACON_OTA_IMAGE"] = args.image

    t0 = time.monotonic()
    try:
        run_ota_update(args.port, chunk=args.chunk, delay=args.delay)
    except Skipped as exc:
        log.warning("SKIP: %s", exc)
        return SKIP_EXIT
    except (UpdateFailed, subprocess.CalledProcessError) as exc:
        log.error("UPDATE FAILED: %s", exc)
        return 1
    log.info("OK — ESP rebooted into the new firmware (%.0fs)", time.monotonic() - t0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
