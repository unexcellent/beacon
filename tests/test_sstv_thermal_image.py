#!/usr/bin/env python3
"""HIL test: the thermal camera's image survives the SSTV round-trip over I2S.

Mirror of the RGB test for the MI48Dx thermal camera. The ESP encodes the thermal
frame as Robot36 audio; we capture it off the Pi's I2S slave input, decode it, and
check it looks like a real frame.

Requires:
  - ESP running firmware with the RGB camera disabled, thermal enabled:
        cargo build --release --features no-rgb-camera   (flash / OTA it)
    so the SSTV downlink is a single 320x240 thermal frame.
  - The thermal camera connected and working, pointed at a scene with some
    temperature contrast (e.g. a hand / warm object) so the frame isn't flat.
  - I2S capture on the Pi as an ALSA card named 'esp-i2s' (see hosts/odin nix).
  - `arecord` (alsa-utils) and the `sstv` Python package (pip install sstv).

The thermal frame is rendered grayscale (src/camera/sensors/mi48.rs: R=G=B), so
it is lower-contrast than a photo — min_std is a bit lower and may need tuning
once a real thermal capture is available.

Skips cleanly (exit 77) when the capture card / arecord is missing.

Run:  ./.venv/bin/python tests/test_sstv_thermal_image.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import run_case
from tests.util.payload_board import MockPayloadBoard
from tests.util.sstv_capture import assert_valid_image, capture_and_decode, send_sstv_command

CAMERA_HINT = "Is the thermal camera working and RGB disabled (--features no-rgb-camera)?"


def case(board: MockPayloadBoard) -> None:
    # 1. Ask the ESP to transmit the camera image as SSTV.
    send_sstv_command(board)

    # 2. Capture the I2S audio and decode it into an image.
    image = capture_and_decode(board, camera_hint=CAMERA_HINT)

    # 3. Check the decoded image is a real frame (grayscale thermal is lower-contrast).
    assert_valid_image(image, min_std=5.0, min_smoothness=0.35)


def test_sstv_thermal_image(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
