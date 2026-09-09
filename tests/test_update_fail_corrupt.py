#!/usr/bin/env python3
"""OTA failure: a valid header but incomplete image.

Announce exactly what we send (so the received==total check passes and finish()
calls esp_ota_end), but the image is only one chunk while its header declares the
full size, so esp_ota_end's validation fails -> UpdateCorrupt. The boot partition
is never switched. Needs a real image prefix (BEACON_OTA_IMAGE /
/tmp/beacon_ota.bin) and skips if none is available.

Run:  ./.venv/bin/python tests/test_update_fail_corrupt.py
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
    board.update_begin(len(prefix))  # total == what we send -> reaches esp_ota_end
    time.sleep(0.1)
    board.update_data(0, prefix)
    time.sleep(0.1)
    board.update_end()
    expect_update_error(board, b"UpdateCorrupt", timeout=15.0)  # esp_ota_end validation
    assert_recovered(board)


def test_corrupt(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
