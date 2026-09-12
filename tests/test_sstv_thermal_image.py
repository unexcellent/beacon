#!/usr/bin/env python3
"""HIL test: capture the thermal camera's SSTV image over I2S and check it's a real frame.

Mirror of the RGB test for the MI48Dx thermal camera. Records the ESP's I2S audio
while an SSTV command runs, decodes the Robot36 frame, validates it, and saves
tests/captures/<YYYY-MM-DD_HH-MM-SS>_thermal.png (timestamp = when the test
started). See tests/util/sstv_capture.py for the flow.

Requires:
  - ESP running firmware with the RGB camera disabled, thermal enabled:
        cargo build --release --features no-rgb-camera   (flash / OTA it)
    so the SSTV downlink is a single 320x240 thermal frame.
  - The thermal camera connected and working, pointed at a scene with some
    temperature contrast (e.g. a hand / warm object) so the frame isn't flat.
  - I2S capture on the Pi as an ALSA card named 'esp-i2s' (see hosts/odin nix).
  - `arecord` (alsa-utils) and the `sstv` Python package (pip install sstv).

The thermal frame is rendered grayscale (src/camera/sensors/mi48.rs: R=G=B), so
it is lower-contrast than a photo — the min_std threshold is a bit lower and may
need tuning once a real thermal capture is available.

Skips cleanly (exit 77) when the capture card / arecord / toolchain is missing.

Run:  ./.venv/bin/python tests/test_sstv_thermal_image.py
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
        "thermal",
        camera_hint="Is the thermal camera working and RGB disabled (--features no-rgb-camera)?",
        min_std=5.0,          # grayscale thermal is lower-contrast than a photo
        min_smoothness=0.35,  # a real thermal frame is smooth, not noise
    )


def test_sstv_thermal_image(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
