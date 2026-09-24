#!/usr/bin/env python3
"""HIL test: an SSTV command is bracketed by BUSY ... AVAILABLE status messages.

idle() handles Command::Sstv by sending BUSY, running transmit_sstv, then sending
AVAILABLE. This checks that both status messages are downlinked to the payload
node, in order. Works with any firmware: cameras disabled makes transmit_sstv a
near-instant no-op, a camera enabled runs a full ~36 s Robot36 transmission, and
the generous AVAILABLE timeout covers both.

Run:  ./.venv/bin/python tests/test_sstv_busy_available.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import run_case
from tests.util.payload_board import MockPayloadBoard
from tests.util.sstv_capture import send_sstv_command, wait_for_available


def test_sstv_busy_available(board: MockPayloadBoard) -> None:
    send_sstv_command(board)
    wait_for_available(board)


if __name__ == "__main__":
    sys.exit(run_case(test_sstv_busy_available))
