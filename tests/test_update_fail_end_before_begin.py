#!/usr/bin/env python3
"""OTA failure: an END with no preceding BEGIN.

The ESP must reject it with UpdateNotInProgress, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_end_before_begin.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import (
    assert_recovered,
    expect_update_error,
    run_case,
    send_update_announcement,
    send_update_end,
)
from tests.util.payload_board import MockPayloadBoard


def test_end_before_begin(board: MockPayloadBoard) -> None:
    send_update_announcement(board)
    send_update_end(board)
    expect_update_error(board, b"UpdateNotInProgress")
    assert_recovered(board)


if __name__ == "__main__":
    sys.exit(run_case(test_end_before_begin))
