#!/usr/bin/env python3
"""OTA failure: an END with no preceding BEGIN.

The ESP must reject it with UpdateNotInProgress, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_end_before_begin.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import CHUNK, assert_recovered, drain, expect_update_error, run_case
from tests.util.payload_board import MockPayloadBoard


def case(board: MockPayloadBoard) -> None:
    drain(board)
    board.update_announce(CHUNK)
    time.sleep(0.2)
    board.update_end()
    expect_update_error(board, b"UpdateNotInProgress")
    assert_recovered(board)


def test_end_before_begin(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
