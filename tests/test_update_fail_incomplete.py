#!/usr/bin/env python3
"""OTA failure: END before all announced bytes arrived.

The ESP must reject it with UpdateIncomplete, not reboot, and stay reachable.
The chunk must be a valid image header (or esp_ota_write rejects it first), so
this needs a real image prefix (BEACON_OTA_IMAGE / /tmp/beacon_ota.bin) and
skips if none is available.

Run:  ./.venv/bin/python tests/test_update_fail_incomplete.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import (
    CHUNK,
    assert_recovered,
    drain,
    expect_update_error,
    run_case,
    skip,
    valid_image_prefix,
)
from tests.util.payload_board import MockPayloadBoard


def case(board: MockPayloadBoard) -> None:
    prefix = valid_image_prefix(CHUNK)
    if prefix is None:
        skip("no valid ESP image available (set BEACON_OTA_IMAGE) for a valid header")
    drain(board)
    board.update_announce(CHUNK)
    time.sleep(0.1)
    board.update_begin(100_000)  # announce far more than we send
    time.sleep(0.1)
    board.update_data(0, prefix)  # one valid chunk...
    time.sleep(0.1)
    board.update_end()  # ...then END, well short of the announced total
    expect_update_error(board, b"UpdateIncomplete")
    assert_recovered(board)


def test_incomplete(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
