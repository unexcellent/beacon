"""Pytest fixtures shared by the hardware-in-the-loop tests.

Standalone runs (`python tests/test_x.py`) don't use this — each test file has a
`__main__` that opens its own board via ota_hil.run_case. This fixture is only
for `pytest tests/`, where one board is shared across the whole session.
"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # repo root
from tests.util.ota_hil import detect_port_or_skip, setup_logging, wait_reachable
from tests.util.payload_board import MockPayloadBoard


@pytest.fixture(scope="session")
def board():
    setup_logging()
    b = MockPayloadBoard(detect_port_or_skip())
    b.start()
    time.sleep(0.3)
    if not wait_reachable(b, timeout=12.0):
        b.close()
        pytest.skip("ESP not reachable over the payload link")
    yield b
    b.close()
