#!/usr/bin/env python3
"""OTA failure: a DATA chunk at an unexpected offset (a gap in the stream).

The ESP must reject it with UpdatePackageOffset, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_wrong_offset.py
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
    time.sleep(0.1)
    board.update_begin(100_000)
    time.sleep(0.1)
    board.update_data(CHUNK, bytes(CHUNK))  # ESP expects offset 0 first
    expect_update_error(board, b"UpdatePackageOffset")
    assert_recovered(board)


def test_wrong_offset(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
