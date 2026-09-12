#!/usr/bin/env python3
"""HIL test: capture the RGB camera's SSTV image over I2S and check it's a real picture.

Records the ESP's I2S audio while an SSTV command runs, decodes the Robot36
frame, validates it, and saves tests/captures/<YYYY-MM-DD_HH-MM-SS>_rgb.png
(timestamp = when the test started). See tests/util/sstv_capture.py for the flow.

Requires:
  - ESP running firmware with the thermal camera disabled, RGB enabled:
        cargo build --release --features no-thermal-camera   (flash / OTA it)
    so the SSTV downlink is a single 320x240 RGB frame.
  - The RGB camera connected and working.
  - I2S capture on the Pi as an ALSA card named 'esp-i2s' (see hosts/odin nix).
  - `arecord` (alsa-utils) and the `sstv` Python package (pip install sstv).

Skips cleanly (exit 77) when the capture card / arecord / toolchain is missing.

Run:  ./.venv/bin/python tests/test_sstv_rgb_image.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import run_case
from tests.util.payload_board import MockPayloadBoard
from tests.util.sstv_capture import capture_sstv_image


def case(board: MockPayloadBoard) -> None:
    capture_sstv_image(
        board,
        "rgb",
        camera_hint="Is the RGB camera working and thermal disabled (--features no-thermal-camera)?",
        min_std=6.0,          # a real photo has plenty of contrast
        min_smoothness=0.35,  # neighbours mostly similar (not noise)
    )


def test_sstv_rgb_image(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
