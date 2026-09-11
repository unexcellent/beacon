#!/usr/bin/env python3
"""HIL test: an SSTV command is bracketed by BUSY ... AVAILABLE status messages.

idle() handles Command::Sstv by sending BUSY, running transmit_sstv, then sending
AVAILABLE. This checks that both status messages are downlinked to the payload
node, in order.

Works with any firmware: with cameras disabled transmit_sstv returns almost
instantly, with a camera enabled it runs a full ~36 s Robot36 transmission. The
generous AVAILABLE timeout covers both, and waiting for AVAILABLE also leaves the
ESP idle for the next test in the suite (a real SSTV keeps it busy for ~36 s).

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

# Covers both a no-op (cameras disabled) and a full Robot36 transmission (~36 s).
MAX_SSTV_SECONDS = 60.0


def case(board: MockPayloadBoard) -> None:
    drain(board)
    t0 = time.monotonic()
    board.send_sstv()

    busy = board.wait_for_text(b"BUSY", timeout=5.0)
    assert busy is not None, "no BUSY status after the SSTV command"
    assert busy.dst == NODE_PAYLOAD, f"BUSY went to node {busy.dst}, expected {NODE_PAYLOAD}"

    available = board.wait_for_text(b"AVAILABLE", timeout=MAX_SSTV_SECONDS)
    assert available is not None, (
        f"no AVAILABLE within {MAX_SSTV_SECONDS:.0f}s of BUSY — transmit_sstv never finished"
    )
    assert available.dst == NODE_PAYLOAD, f"AVAILABLE went to node {available.dst}, expected {NODE_PAYLOAD}"

    elapsed = time.monotonic() - t0
    log.info("SSTV bracketed by BUSY -> AVAILABLE in %.2fs ✓", elapsed)


def test_sstv_busy_available(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
