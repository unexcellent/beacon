#!/usr/bin/env python3
"""HIL test: the RGB camera's image survives the SSTV round-trip over I2S.

The ESP encodes the RGB frame as Robot36 audio; we capture it off the Pi's I2S
slave input, decode it, and check it looks like a real picture.

Requires:
  - ESP running firmware with the thermal camera disabled, RGB enabled:
        cargo build --release --features no-thermal-camera   (flash / OTA it)
    so the SSTV downlink is a single 320x240 RGB frame.
  - The RGB camera connected and working.
  - I2S capture on the Pi as an ALSA card named 'esp-i2s' (see hosts/odin nix).
  - `arecord` (alsa-utils) and the `sstv` Python package (pip install sstv).

Skips cleanly (exit 77) when the capture card / arecord is missing.

Run:  ./.venv/bin/python tests/test_rgb_only_transmission.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import run_case
from tests.util.payload_board import MockPayloadBoard
from tests.util.sstv_capture import (
    assert_valid_image,
    capture_and_decode,
    send_sstv_command,
)

CAMERA_HINT = (
    "Is the RGB camera working and thermal disabled (--features no-thermal-camera)?"
)


def case(board: MockPayloadBoard) -> None:
    send_sstv_command(board)
    image = capture_and_decode(board, camera_hint=CAMERA_HINT)
    assert_valid_image(image, min_std=6.0, min_smoothness=0.35)


def test_rgb_only_transmission(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
