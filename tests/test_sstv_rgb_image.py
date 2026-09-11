#!/usr/bin/env python3
"""HIL test: capture the RGB camera's SSTV image over I2S and check it's a real picture.

Flow: start recording the ESP's I2S audio, send the SSTV command, let the ESP
encode the RGB frame as Robot36 (~36 s) and play it out I2S, then decode the
recording to a PNG, validate it looks like an image, and save it to
tests/captures/<YYYY-MM-DD_HH-MM-SS>_rgb.png (timestamp = when the test started).

Requires:
  - ESP running firmware with the thermal camera disabled, RGB enabled:
        cargo build --release --features no-thermal-camera   (flash / OTA it)
    so the SSTV downlink is a single 320x240 RGB frame.
  - The RGB camera connected and working.
  - I2S capture on the Pi as an ALSA card named 'esp-i2s' (see hosts/odin nix).
  - `arecord` (alsa-utils) and a Rust host toolchain (to build tools/sstv-decode).

Skips cleanly (exit 77) when the capture card / arecord / toolchain is missing.

Run:  ./.venv/bin/python tests/test_sstv_rgb_image.py
"""

import logging
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import drain, run_case, skip
from tests.util.payload_board import NODE_PAYLOAD, MockPayloadBoard

log = logging.getLogger("sstv_rgb")

REPO = Path(__file__).resolve().parent.parent
CAPTURES = REPO / "tests" / "captures"
DECODE_SRC = REPO / "tools" / "sstv-decode"

SAMPLE_RATE = 16000            # ESP audio rate (src/audio/move-iiia.rs)
RECORD_SECONDS = 42            # Robot36 is ~36 s; record a bit longer
AVAILABLE_TIMEOUT = 60.0       # SSTV takes ~36 s before AVAILABLE

ROBOT36 = (320, 240)
MIN_STD = 6.0                  # image must have content (not a flat/blank frame)
MIN_SMOOTHNESS = 0.35          # a real image (neighbours similar), not noise


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


def sstv_decoder() -> Path:
    """Locate the sstv-decode binary, or build it as a fallback.

    Prefers a prebuilt binary (shipped by local/deploy_sstv_decode.sh) so the
    slow Pi never compiles it: $SSTV_DECODE_BIN, then ~/.local/bin/sstv-decode,
    then $PATH. Only if none exist do we build it locally.
    """
    for cand in (os.environ.get("SSTV_DECODE_BIN"),
                 str(Path.home() / ".local/bin/sstv-decode"),
                 shutil.which("sstv-decode")):
        if cand and Path(cand).is_file() and os.access(cand, os.X_OK):
            return Path(cand)

    if not (DECODE_SRC / "Cargo.toml").is_file():
        skip("no sstv-decode binary and tools/sstv-decode source not found")
    if not shutil.which("cargo"):
        skip("no sstv-decode binary; run local/deploy_sstv_decode.sh from a build host")
    # Build OUTSIDE the beacon repo so its .cargo/config.toml (riscv target +
    # build-std) doesn't apply — a clean host build of the decoder.
    build_dir = Path(tempfile.gettempdir()) / "beacon-sstv-decode"
    binp = build_dir / "target" / "release" / "sstv-decode"
    if not binp.is_file():
        log.info("building sstv-decode locally (no prebuilt binary found) ...")
        if build_dir.exists():
            shutil.rmtree(build_dir)
        shutil.copytree(DECODE_SRC, build_dir)
        subprocess.run(["cargo", "build", "--release"], cwd=build_dir, check=True)
    return binp


def wav_peak(path: Path) -> int:
    """Largest absolute 16-bit sample across all channels (0 = silence)."""
    import array
    import wave
    with wave.open(str(path), "rb") as w:
        samples = array.array("h")
        samples.frombytes(w.readframes(w.getnframes()))
    return max((abs(s) for s in samples), default=0)


def case(board: MockPayloadBoard) -> None:
    device = alsa_capture_device()
    decoder = sstv_decoder()
    CAPTURES.mkdir(parents=True, exist_ok=True)

    started = datetime.now()  # filename timestamp = test start, not receive time
    stamp = started.strftime("%Y-%m-%d_%H-%M-%S")
    wav = Path(tempfile.gettempdir()) / f"sstv_{stamp}.wav"
    png = CAPTURES / f"{stamp}_rgb.png"

    drain(board)
    log.info("recording I2S for %ds on %s", RECORD_SECONDS, device)
    rec = subprocess.Popen(
        ["arecord", "-D", device, "-f", "S16_LE", "-r", str(SAMPLE_RATE),
         "-c", "2", "-d", str(RECORD_SECONDS), "-t", "wav", str(wav)],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    time.sleep(0.5)  # let capture start before the tones begin
    if rec.poll() is not None:  # arecord died immediately (bad device/format)
        err = rec.stderr.read().decode("utf-8", "replace") if rec.stderr else ""
        raise AssertionError(f"arecord failed to start: {err.strip()}")

    board.send_sstv()
    busy = board.wait_for_text(b"BUSY", timeout=5.0)
    assert busy is not None and busy.dst == NODE_PAYLOAD, "no BUSY after the SSTV command"
    log.info("BUSY received; SSTV transmitting (~36s) ...")
    available = board.wait_for_text(b"AVAILABLE", timeout=AVAILABLE_TIMEOUT)
    assert available is not None, (
        "no AVAILABLE — SSTV never finished. Is the RGB camera working and thermal disabled?"
    )
    log.info("AVAILABLE received; finishing recording")

    rc = rec.wait(timeout=RECORD_SECONDS + 10)  # arecord stops itself at -d
    if rc not in (0, None):
        log.warning("arecord exited %s", rc)

    # Distinguish a silent capture (clock present, no data) from a decode failure.
    peak = wav_peak(wav)
    log.info("recorded peak=%d (%.1f%% FS)", peak, 100 * peak / 32768)
    assert peak > 200, (
        f"capture is silent (peak={peak}). The I2S clock is working but no audio data "
        "arrived — check the data wire (ESP data-out -> Pi GPIO20 / pin 38), or the ESP "
        "was not actually transmitting SSTV."
    )

    result = subprocess.run([str(decoder), str(wav), str(png)], capture_output=True, text=True)
    assert result.returncode == 0, f"SSTV decode failed: {result.stderr.strip()}"
    w, h, std_r, std_g, std_b, smooth = result.stdout.split()
    w, h = int(w), int(h)
    std = (float(std_r), float(std_g), float(std_b))
    smooth = float(smooth)
    log.info("decoded %dx%d std=(%.0f,%.0f,%.0f) smoothness=%.2f -> %s", w, h, *std, smooth, png)

    assert (w, h) == ROBOT36, f"decoded {w}x{h}, expected {ROBOT36}"
    assert max(std) > MIN_STD, f"image is flat/blank (per-channel std {std}) — no real picture"
    assert smooth > MIN_SMOOTHNESS, f"image looks like noise (smoothness {smooth:.2f}) — bad capture/decode"
    log.info("valid RGB image saved: %s", png)


def test_sstv_rgb_image(board):
    case(board)


if __name__ == "__main__":
    sys.exit(run_case(case))
