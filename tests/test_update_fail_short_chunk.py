#!/usr/bin/env python3
"""OTA failure: a non-final chunk shorter than the announced chunk size.

The ESP must reject it with UpdateChunkIncomplete, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_short_chunk.py
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
    board.update_data(0, bytes(CHUNK // 2))  # short, and far from the end
    expect_update_error(board, b"UpdateChunkIncomplete")
    assert_recovered(board)


def test_short_chunk(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
