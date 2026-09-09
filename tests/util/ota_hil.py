#!/usr/bin/env python3
"""Shared helpers for the hardware-in-the-loop OTA tests over the payload link.

Used by tests/test_update_successfull.py and the per-case tests/test_update_fail_*.py.
Each failure test is its own file so they can be run and reported individually;
this module holds the setup and the assertions they have in common.
"""

from __future__ import annotations

import glob
import logging
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator, NoReturn

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # repo root
from tests.util.payload_board import MockPayloadBoard  # noqa: E402

log = logging.getLogger("ota_hil")

CHUNK = 256  # announced chunk size for the failure-case sequences
SKIP_EXIT = 77  # standalone exit code for "skipped" (automake convention)


class Skipped(Exception):
    """Raised to skip a standalone run (no hardware / no image)."""


def skip(msg: str) -> NoReturn:
    # Under pytest, translate to a real skip; standalone, raise our own type.
    # pytest is only in sys.modules when actually running under pytest (this
    # module never imports it), so a plain `python test_x.py` run raises Skipped.
    if "pytest" in sys.modules:
        import pytest

        pytest.skip(msg)
    raise Skipped(msg)


def detect_port_or_skip() -> str:
    """First RS422 adapter, or skip if none is connected."""
    ports = sorted(glob.glob("/dev/cu.usbserial-*")) or sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        skip("no RS422 adapter detected — HIL test requires the payload link")
    return ports[0]


def wait_reachable(board: MockPayloadBoard, timeout: float = 12.0) -> bool:
    """Ping until the ESP answers or `timeout` elapses. After a (re)boot the ESP
    spends a few seconds in camera init before it reaches idle() and services the
    link, so a single short ping is not enough."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if board.ping(timeout=1.0):
            return True
    return False


def valid_image_prefix(nbytes: int) -> bytes | None:
    """First `nbytes` of a real ESP app image (valid header), so esp_ota_write
    accepts the write. Found via BEACON_OTA_IMAGE or a staged /tmp image."""
    for path in (os.environ.get("BEACON_OTA_IMAGE"), "/tmp/beacon_ota.bin"):
        if path and Path(path).is_file():
            data = Path(path).read_bytes()
            if data[:1] == b"\xe9":  # ESP image magic
                return data[:nbytes]
    return None


def drain(board: MockPayloadBoard) -> None:
    while board.receive(timeout=0) is not None:
        pass


def expect_update_error(board: MockPayloadBoard, expected_prefix: bytes, timeout: float = 10.0) -> bytes:
    """Wait for the ESP to downlink an `Update*` error, failing if it reboots
    first (a failed update must never reboot). Returns the error payload."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pkt = board.receive(timeout=deadline - time.monotonic())
        if pkt is None:
            continue
        if pkt.dst == 1 and pkt.payload.startswith(b"STATUS: BOOTED"):
            raise AssertionError(f"ESP rebooted on a failed update (must not happen): {pkt.payload!r}")
        if pkt.dst == 2 and pkt.payload.startswith(b"Update"):
            got = bytes(pkt.payload)
            assert got.startswith(expected_prefix), f"expected {expected_prefix!r}, got {got!r}"
            log.info("got expected error: %s", got.decode("ascii", "replace"))
            return got
    raise AssertionError(f"no {expected_prefix!r} error downlinked within {timeout:.0f}s")


def assert_recovered(board: MockPayloadBoard) -> None:
    """After a failed update the ESP must not reboot and must still serve the link."""
    boot = board.wait_for(lambda p: p.dst == 1 and p.payload.startswith(b"STATUS: BOOTED"), timeout=2.0)
    assert boot is None, f"ESP rebooted after a failed update: {bytes(boot.payload)!r}"
    assert wait_reachable(board, timeout=8.0), "ESP unreachable after a failed update"
    log.info("ESP still idle and reachable ✓")


def setup_logging() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s.%(msecs)03d %(message)s", datefmt="%H:%M:%S")
    # The board logs every TX/RX frame; during a full OTA that buries the output.
    logging.getLogger("payload_board").setLevel(logging.WARNING)


@contextmanager
def open_board() -> Iterator[MockPayloadBoard]:
    """Open the payload link and confirm the ESP is reachable, or skip."""
    board = MockPayloadBoard(detect_port_or_skip())
    board.start()
    time.sleep(0.3)
    if not wait_reachable(board, timeout=12.0):
        board.close()
        skip("ESP not reachable over the payload link")
    try:
        yield board
    finally:
        board.close()


def run_case(case: Callable[[MockPayloadBoard], None]) -> int:
    """Standalone harness for one HIL case: 0 = pass, 1 = fail, SKIP_EXIT = skipped."""
    setup_logging()
    try:
        with open_board() as board:
            case(board)
    except Skipped as exc:
        log.warning("SKIP: %s", exc)
        return SKIP_EXIT
    except AssertionError as exc:
        log.error("FAIL: %s", exc)
        return 1
    log.info("PASS")
    return 0
