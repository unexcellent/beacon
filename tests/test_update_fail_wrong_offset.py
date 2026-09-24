#!/usr/bin/env python3
"""OTA failure: a DATA chunk at an unexpected offset (a gap in the stream).

The ESP must reject it with UpdatePackageOffset, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_wrong_offset.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import (
    CHUNK,
    assert_recovered,
    expect_update_error,
    run_case,
    send_update_announcement,
    send_update_begin,
    send_update_data,
)
from tests.util.payload_board import MockPayloadBoard


def test_wrong_offset(board: MockPayloadBoard) -> None:
    send_update_announcement(board)
    send_update_begin(board, 100_000)
    send_update_data(board, CHUNK, bytes(CHUNK))
    expect_update_error(board, b"UpdatePackageOffset")
    assert_recovered(board)


if __name__ == "__main__":
    sys.exit(run_case(test_wrong_offset))
