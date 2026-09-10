#!/usr/bin/env python3
"""HIL test: an SSTV command is bracketed by BUSY ... AVAILABLE status messages.

idle() handles Command::Sstv by sending BUSY, running transmit_sstv, then sending
AVAILABLE. This checks that both status messages are downlinked to the payload
node, in order.

It expects the ESP to run firmware built with BOTH cameras disabled
(`cargo build --features no-rgb-camera,no-thermal-camera`): then transmit_sstv
has no cameras to capture and returns immediately, so BUSY -> AVAILABLE is
near-instant instead of a multi-second SSTV transmission per camera. The timing
bound below also flags firmware that still has a working camera (SSTV would then
take far longer than a no-op).

Run:  ./.venv/bin/python tests/test_sstv_busy_available.py
"""

import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import drain, run_case
from tests.util.payload_board import NODE_PAYLOAD, MockPayloadBoard

log = logging.getLogger("sstv_test")

# With both cameras disabled, transmit_sstv is a no-op, so BUSY -> AVAILABLE is
# sub-second; allow generous slack. A working camera makes SSTV take tens of
# seconds and this bound fails, signalling the wrong firmware.
MAX_SSTV_SECONDS = 8.0


def case(board: MockPayloadBoard) -> None:
    drain(board)
    t0 = time.monotonic()
    board.send_sstv()

    busy = board.wait_for_text(b"BUSY", timeout=5.0)
    assert busy is not None, "no BUSY status after the SSTV command"
    assert busy.dst == NODE_PAYLOAD, f"BUSY went to node {busy.dst}, expected {NODE_PAYLOAD}"

    available = board.wait_for_text(b"AVAILABLE", timeout=MAX_SSTV_SECONDS)
    assert available is not None, (
        f"no AVAILABLE within {MAX_SSTV_SECONDS:.0f}s of BUSY — is a camera still enabled? "
        "build with --features no-rgb-camera,no-thermal-camera"
    )
    assert available.dst == NODE_PAYLOAD, f"AVAILABLE went to node {available.dst}, expected {NODE_PAYLOAD}"

    elapsed = time.monotonic() - t0
    log.info("SSTV bracketed by BUSY -> AVAILABLE in %.2fs ✓", elapsed)


def test_sstv_busy_available(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
