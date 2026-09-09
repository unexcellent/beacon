#!/usr/bin/env python3
"""OTA failure: a full-size chunk that is not a valid ESP app image.

esp_ota_write validates the image header on the first write, so a bad-magic chunk
is rejected with UpdateWrite; the ESP must not reboot and must stay reachable.

Run:  ./.venv/bin/python tests/test_update_fail_invalid_image.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import CHUNK, assert_recovered, drain, expect_update_error, run_case
from tests.util.payload_board import MockPayloadBoard


def case(board: MockPayloadBoard) -> None:
    garbage = bytes((i * 7 + 1) & 0xFF for i in range(CHUNK))  # first byte != 0xE9
    assert garbage[0] != 0xE9
    drain(board)
    board.update_announce(CHUNK)
    time.sleep(0.1)
    board.update_begin(100_000)
    time.sleep(0.1)
    board.update_data(0, garbage)
    expect_update_error(board, b"UpdateWrite")
    assert_recovered(board)


def test_invalid_image(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
