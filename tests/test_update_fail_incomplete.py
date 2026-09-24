#!/usr/bin/env python3
"""OTA failure: a *short* transfer — END before the announced bytes all arrived.

We announce a total far larger than we send, then END, so the ESP's
received<announced check fails at END -> UpdateIncomplete; it never reaches
esp_ota_end validation and never switches the boot partition.

Contrast test_update_fail_corrupt, where the transfer completes but the image is
invalid. The one chunk we send must be a valid image header (or esp_ota_write
rejects it first), so this needs a real image (BEACON_OTA_IMAGE /
/tmp/beacon_ota.bin) and skips if none is available.

Run:  ./.venv/bin/python tests/test_update_fail_incomplete.py
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
    send_update_end,
    staged_firmware_image,
)
from tests.util.payload_board import MockPayloadBoard


def test_incomplete(board: MockPayloadBoard) -> None:
    header = staged_firmware_image()[:CHUNK]
    send_update_announcement(board)
    send_update_begin(board, 100 * len(header))
    send_update_data(board, 0, header)
    send_update_end(board)
    expect_update_error(board, b"UpdateIncomplete")
    assert_recovered(board)


if __name__ == "__main__":
    sys.exit(run_case(test_incomplete))
