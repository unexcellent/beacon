#!/usr/bin/env python3
"""Building blocks for the SSTV image-capture HIL tests (RGB and thermal).

The tests compose three steps: send the SSTV command, capture + decode the image
the ESP transmits over I2S, and check the image is valid. Both cameras render
into the same Robot36 320x240 frame, so only the firmware camera selection and
the validity thresholds differ between the RGB and thermal tests.
"""

import array
import logging
import os
import re
import shutil
import subprocess
import tempfile
import time
import wave
from pathlib import Path

import sstv
from PIL import ImageStat

from tests.util.ota_hil import drain, skip
from tests.util.payload_board import NODE_PAYLOAD, MockPayloadBoard

log = logging.getLogger("sstv_capture")

SAMPLE_RATE = 16000       # ESP audio rate (src/audio/move-iiia.rs)
RECORD_SECONDS = 42       # Robot36 is ~36 s; record a bit longer
AVAILABLE_TIMEOUT = 60.0  # SSTV takes ~36 s before AVAILABLE
ROBOT36 = (320, 240)


def send_sstv_command(board: MockPayloadBoard) -> None:
    """Send the SSTV command and confirm the ESP acknowledged it with BUSY."""
    drain(board)
    board.send_sstv()
    busy = board.wait_for_text(b"BUSY", timeout=5.0)
    assert busy is not None and busy.dst == NODE_PAYLOAD, "no BUSY after the SSTV command"
    log.info("SSTV command sent; ESP is BUSY, transmitting (~36s) ...")


def capture_and_decode(board: MockPayloadBoard, *, camera_hint: str):
    """Record the I2S audio through the transmission, then decode the Robot36 image.

    Waits for the ESP's AVAILABLE (transmission done), verifies the recording
    isn't silent (a silent-but-clocked capture means no I2S data reached the Pi),
    and returns the decoded PIL image. `camera_hint` is shown if SSTV never ends.
    """
    device = alsa_capture_device()
    wav = Path(tempfile.gettempdir()) / f"sstv_capture_{os.getpid()}.wav"

    log.info("recording I2S for %ds on %s", RECORD_SECONDS, device)
    rec = subprocess.Popen(
        ["arecord", "-D", device, "-f", "S16_LE", "-r", str(SAMPLE_RATE),
         "-c", "2", "-d", str(RECORD_SECONDS), "-t", "wav", str(wav)],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    time.sleep(0.5)  # let capture start (the tones only begin a few seconds in)
    if rec.poll() is not None:  # arecord died immediately (bad device/format)
        err = rec.stderr.read().decode("utf-8", "replace") if rec.stderr else ""
        raise AssertionError(f"arecord failed to start: {err.strip()}")

    available = board.wait_for_text(b"AVAILABLE", timeout=AVAILABLE_TIMEOUT)
    assert available is not None, f"no AVAILABLE — SSTV never finished. {camera_hint}"
    log.info("AVAILABLE received; finishing recording")
    rc = rec.wait(timeout=RECORD_SECONDS + 10)  # arecord stops itself at -d
    if rc not in (0, None):
        log.warning("arecord exited %s", rc)

    samples, rate = _read_active_channel(wav)
    peak = max((abs(s) for s in samples), default=0)
    log.info("recorded peak=%d (%.1f%% FS)", peak, 100 * peak / 32768)
    assert peak > 200, (
        f"capture is silent (peak={peak}). The I2S clock is working but no audio data "
        "arrived — check the data wire (ESP data-out -> Pi GPIO20 / pin 38), or the ESP "
        "was not actually transmitting SSTV."
    )

    images = sstv.decode(samples, rate, mode=sstv.Mode.ROBOT_36)
    assert images, "no SSTV image could be decoded from the recording"
    return images[0]


def assert_valid_image(image, *, min_std: float, min_smoothness: float) -> None:
    """Check the decoded image is a real picture: right size, has content, not noise."""
    (w, h), std, smooth = _image_stats(image)
    log.info("decoded %dx%d std=(%.0f,%.0f,%.0f) smoothness=%.2f", w, h, *std, smooth)
    assert (w, h) == ROBOT36, f"decoded {w}x{h}, expected {ROBOT36}"
    assert max(std) > min_std, f"image is flat/blank (per-channel std {std}) — no real picture"
    assert smooth > min_smoothness, f"image looks like noise (smoothness {smooth:.2f}) — bad capture/decode"


def alsa_capture_device() -> str:
    """The ESP I2S ALSA capture device (hw:<card>,0), or skip if not set up."""
    dev = os.environ.get("SSTV_ALSA_DEVICE")
    if dev:
        return dev
    if not shutil.which("arecord"):
        skip("arecord not installed (alsa-utils) — capture not set up")
    try:
        cards = Path("/proc/asound/cards").read_text()
    except OSError:
        skip("no ALSA cards — I2S capture not set up on this host")
    # Match the card by name/id and use its number (robust to renumbering; the
    # ALSA card id is 'espi2s', the pretty name 'esp-i2s').
    for line in cards.splitlines():
        if "espi2s" in line or "esp-i2s" in line:
            m = re.match(r"\s*(\d+)\s", line)
            if m:
                return f"hw:{m.group(1)},0"
    skip("no 'esp-i2s' capture card — set up the Pi I2S slave capture first")


def _read_active_channel(path: Path) -> tuple[list[int], int]:
    """Read a 16-bit WAV, returning (samples, rate) for the loudest channel.

    The ESP puts the SSTV tones on a single I2S channel and silence on the other,
    so pick the higher-energy channel rather than the first one.
    """
    with wave.open(str(path), "rb") as w:
        channels = max(w.getnchannels(), 1)
        rate = w.getframerate()
        samples = array.array("h")
        samples.frombytes(w.readframes(w.getnframes()))
    if channels <= 1:
        return list(samples), rate
    energy = [sum(x * x for x in samples[c::channels]) for c in range(channels)]
    best = max(range(channels), key=energy.__getitem__)
    return list(samples[best::channels]), rate


def _image_stats(image) -> tuple[tuple[int, int], list[float], float]:
    """(size, per-channel stddev, horizontal-neighbour similarity fraction)."""
    image = image.convert("RGB")
    w, h = image.size
    std = ImageStat.Stat(image).stddev
    px = image.load()
    similar = pairs = 0
    for y in range(h):
        prev = px[0, y]
        for x in range(1, w):
            cur = px[x, y]
            for c in range(3):
                if abs(cur[c] - prev[c]) <= 24:
                    similar += 1
                pairs += 1
            prev = cur
    smoothness = similar / pairs if pairs else 0.0
    return (w, h), std, smoothness
