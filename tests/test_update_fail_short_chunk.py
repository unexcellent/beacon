#!/usr/bin/env python3
"""OTA failure: a DATA chunk shorter than the announced size, not near the end.

The ESP must reject it with UpdateChunkIncomplete, not reboot, and stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_short_chunk.py
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


def test_short_chunk(board: MockPayloadBoard) -> None:
    send_update_announcement(board)
    send_update_begin(board, 100_000)
    send_update_data(board, 0, bytes(CHUNK // 2))
    expect_update_error(board, b"UpdateChunkIncomplete")
    assert_recovered(board)


if __name__ == "__main__":
    sys.exit(run_case(test_short_chunk))
