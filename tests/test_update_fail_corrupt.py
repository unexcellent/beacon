#!/usr/bin/env python3
"""OTA failure: a *complete* transfer of a truncated (structurally invalid) image.

We announce exactly the bytes we send, so the ESP's received==announced check
passes and it calls esp_ota_end. But those bytes are only the start of a real
image whose header declares a much larger multi-segment image, so esp_ota_end's
validation fails -> UpdateCorrupt and the boot partition is never switched.

Contrast test_update_fail_incomplete, where the *transfer* is short. Needs a real
image (BEACON_OTA_IMAGE / /tmp/beacon_ota.bin) and skips if none is available.

Run:  ./.venv/bin/python tests/test_update_fail_corrupt.py
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


def test_corrupt(board: MockPayloadBoard) -> None:
    truncated_image = staged_firmware_image()[:CHUNK]
    send_update_announcement(board)
    send_update_begin(board, len(truncated_image))
    send_update_data(board, 0, truncated_image)
    send_update_end(board)
    expect_update_error(board, b"UpdateCorrupt", timeout=15.0)
    assert_recovered(board)


if __name__ == "__main__":
    sys.exit(run_case(test_corrupt))
