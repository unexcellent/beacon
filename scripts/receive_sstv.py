#!/usr/bin/env python3
"""
receive_sstv.py — RTL-SDR SSTV receiver on 145.0 MHz

Shows a live waterfall and optionally prints decoded SSTV tone frequencies
or plays them through the speakers.

Usage:
    python receive_sstv.py [--tones] [--audio] [--freq 145.0] [--gain 40]

Flags:
    --tones        Print SSTV tone Hz + level to stdout each frame
    --audio        Play the FM-demodulated audio through the speakers
    --freq FLOAT   Center frequency in MHz (default: 145.0)
    --gain INT     RTL-SDR gain in dB (default: 40; use 0 for auto)
"""

import argparse
import os
import sys
import threading
import queue
import time

# ── macOS: help ctypes find Homebrew's librtlsdr via DYLD_LIBRARY_PATH ───────
# ctypes.util.find_library doesn't search /opt/homebrew/lib, so re-exec with
# the path set if needed (must be in the environment before dlopen is called).
def _ensure_dyld():
    import ctypes.util
    if ctypes.util.find_library("rtlsdr") is not None:
        return  # already visible
    if os.environ.get("_RTLSDR_PATH_FIXED"):
        return  # already re-exec'd once; give up
    for _brew in ["/opt/homebrew/lib", "/usr/local/lib"]:
        if os.path.exists(os.path.join(_brew, "librtlsdr.dylib")):
            new_env = os.environ.copy()
            existing = new_env.get("DYLD_LIBRARY_PATH", "")
            new_env["DYLD_LIBRARY_PATH"] = f"{_brew}:{existing}" if existing else _brew
            new_env["_RTLSDR_PATH_FIXED"] = "1"
            os.execve(sys.executable, [sys.executable] + sys.argv, new_env)

if sys.platform == "darwin":
    _ensure_dyld()

try:
    import numpy as np
except ImportError:
    print("numpy not installed — run: pip install numpy", file=sys.stderr)
    sys.exit(1)

try:
    from scipy.signal import firwin, lfilter, lfilter_zi
except ImportError:
    print("scipy not installed — run: pip install scipy", file=sys.stderr)
    sys.exit(1)

try:
    from rtlsdr import RtlSdr
except Exception as e:
    print(f"Cannot import rtlsdr: {e}", file=sys.stderr)
    print("Ensure pyrtlsdr is installed and librtlsdr is on the system.", file=sys.stderr)
    print("  macOS:  brew install librtlsdr && pip install pyrtlsdr", file=sys.stderr)
    print("  Linux:  sudo apt install rtl-sdr && pip install pyrtlsdr", file=sys.stderr)
    sys.exit(1)

import matplotlib
_backends = ["MacOSX", "Qt5Agg", "GTK3Agg", "TkAgg", "Agg"]
for _b in _backends:
    try:
        matplotlib.use(_b)
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        from matplotlib.colors import LinearSegmentedColormap
        break
    except Exception:
        continue
else:
    print("No usable matplotlib backend found.", file=sys.stderr)
    sys.exit(1)

# ── constants ─────────────────────────────────────────────────────────────────
CENTER_FREQ   = 145.0e6   # Hz
SAMPLE_RATE   = 2.048e6   # MSPS  (RTL-SDR sweet spot)
AUDIO_RATE    = 32000     # Hz after decimation
DECIM         = int(SAMPLE_RATE / AUDIO_RATE)   # 64

FFT_SIZE      = 2048
WATERFALL_H   = 300       # rows kept in history
OVERLAP       = FFT_SIZE // 2

SSTV_LO       = 1100      # Hz — lowest SSTV tone
SSTV_HI       = 2400      # Hz — highest SSTV tone
SSTV_SYNC     = 1200      # sync pulse
SSTV_BLACK    = 1500      # black level
SSTV_WHITE    = 2300      # white level

TONE_SMOOTHING = 0.2      # IIR coeff for tone display (lower = smoother)

# FM deviation for audio scaling: NFM = 5 kHz, so tones at 1500 Hz occupy
# 1500/1e6 of the normalised range — boost to make them audible at full scale.
FM_DEVIATION  = 5000      # Hz (adjust if signal uses wider deviation)

# ── color palette — dark navy → teal → gold → ember ──────────────────────────
_WF_COLORS = [
    (0.00, "#05050d"),
    (0.15, "#0b1a3e"),
    (0.35, "#0e4d6e"),
    (0.55, "#14998a"),
    (0.72, "#f0c040"),
    (0.88, "#e85d20"),
    (1.00, "#ffffff"),
]
cmap_wf = LinearSegmentedColormap.from_list(
    "sstv_wf", [(v, c) for v, c in _WF_COLORS]
)

# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tones", action="store_true",
                   help="Print SSTV tone Hz + level each FFT frame")
    p.add_argument("--audio", action="store_true",
                   help="Play FM-demodulated audio through speakers")
    p.add_argument("--freq",  type=float, default=145.0,
                   help="Center frequency in MHz (default: 145.0)")
    p.add_argument("--gain",  type=int,   default=40,
                   help="RTL-SDR gain in dB (0 = auto, default: 40)")
    return p.parse_args()

# ── channel filter: complex LPF on IQ before FM demod ────────────────────────
# Passes only ±10 kHz around the tuned carrier, rejecting every other signal
# and noise source in the 2 MHz capture bandwidth.  Applied to I and Q
# separately (equivalent to a bandpass centred at 0 Hz in complex baseband).
_CH_TAPS  = firwin(255, cutoff=10000, fs=SAMPLE_RATE, window="hamming")
_ch_zi_i  = lfilter_zi(_CH_TAPS, [1.0]) * 0.0
_ch_zi_q  = lfilter_zi(_CH_TAPS, [1.0]) * 0.0

# ── anti-aliasing LPF after FM demod, before decimation ──────────────────────
# Cutoff just below half the output sample rate.
_LPF_TAPS = firwin(127, cutoff=15000, fs=SAMPLE_RATE, window="hamming")
_lpf_zi   = lfilter_zi(_LPF_TAPS, [1.0]) * 0.0

# ── FM discriminator — two independent state sets (display / audio threads) ───
# Each thread needs its own filter state so they never race on the same globals.
_disp_prev  = np.complex64(1.0 + 0j)
_disp_ch_zi_i = _ch_zi_i.copy();  _disp_ch_zi_q = _ch_zi_q.copy()
_disp_lpf_zi  = _lpf_zi.copy()

_audio_prev   = np.complex64(1.0 + 0j)
_audio_ch_zi_i = _ch_zi_i.copy(); _audio_ch_zi_q = _ch_zi_q.copy()
_audio_lpf_zi  = _lpf_zi.copy()

def _demod(iq, state):
    """Channel-filter → FM discriminate → decimate.  state dict is mutated."""
    i_f, state["chi"] = lfilter(_CH_TAPS, [1.0], iq.real, zi=state["chi"])
    q_f, state["chq"] = lfilter(_CH_TAPS, [1.0], iq.imag, zi=state["chq"])
    iq_c = (i_f + 1j * q_f).astype(np.complex64)
    ext  = np.concatenate([[state["prev"]], iq_c])
    state["prev"] = iq_c[-1]
    d    = np.angle(ext[1:] * np.conj(ext[:-1])) / np.pi
    filt, state["lpf"] = lfilter(_LPF_TAPS, [1.0], d, zi=state["lpf"])
    return (filt[::DECIM] * (SAMPLE_RATE / 2.0 / FM_DEVIATION)).astype(np.float32)

_disp_state  = {"prev": np.complex64(1+0j),
                "chi": _ch_zi_i.copy(), "chq": _ch_zi_q.copy(),
                "lpf": _lpf_zi.copy()}
_audio_state = {"prev": np.complex64(1+0j),
                "chi": _ch_zi_i.copy(), "chq": _ch_zi_q.copy(),
                "lpf": _lpf_zi.copy()}

# ── dominant tone in SSTV band (post-FM-demod audio) ─────────────────────────
def detect_tone(audio: np.ndarray, fs: float) -> tuple[float, float]:
    """Return (freq_hz, magnitude_dB) of the dominant tone in the SSTV band."""
    n     = len(audio)
    win   = np.hanning(n)
    spec  = np.abs(np.fft.rfft(audio * win))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    lo, hi = np.searchsorted(freqs, SSTV_LO), np.searchsorted(freqs, SSTV_HI)
    if lo >= hi:
        return 0.0, -120.0
    band_spec = spec[lo:hi]
    peak_idx  = np.argmax(band_spec)
    peak_mag  = band_spec[peak_idx]
    mag_db    = 20 * np.log10(peak_mag + 1e-12)
    freq_hz   = freqs[lo + peak_idx]
    return float(freq_hz), float(mag_db)

# ── queues ────────────────────────────────────────────────────────────────────
# The SDR callback fans out to two independent queues so the audio thread and
# the display loop never block each other.
display_q:   queue.Queue = queue.Queue(maxsize=8)   # raw IQ → display loop
audio_raw_q: queue.Queue = queue.Queue(maxsize=32)  # raw IQ → audio thread
audio_q:     queue.Queue = queue.Queue(maxsize=64)  # decoded PCM → sounddevice

_audio_mode = False   # set to True in main() when --audio is active

def sdr_callback(samples, context):
    try:
        display_q.put_nowait(samples)
    except queue.Full:
        pass
    if _audio_mode:
        try:
            audio_raw_q.put_nowait(samples)
        except queue.Full:
            pass  # audio thread is behind; drop rather than block

# ── main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    # ── init SDR ──────────────────────────────────────────────────────────────
    sdr = RtlSdr()
    sdr.sample_rate   = SAMPLE_RATE
    sdr.center_freq   = args.freq * 1e6
    sdr.gain          = args.gain if args.gain != 0 else "auto"
    print(f"Tuned to {args.freq:.3f} MHz  |  SR={SAMPLE_RATE/1e6:.3f} MSPS  |  gain={sdr.gain} dB")

    # ── waterfall state ────────────────────────────────────────────────────────
    wf_data  = np.full((WATERFALL_H, FFT_SIZE), -80.0, dtype=np.float32)
    freq_ax  = np.fft.fftshift(
        np.fft.fftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)
    ) / 1e6 + args.freq

    # ── tone indicator state ───────────────────────────────────────────────────
    tone_hz_smooth   = SSTV_BLACK
    tone_mag_smooth  = -80.0

    # ── figure layout ─────────────────────────────────────────────────────────
    BG   = "#080810"
    FG   = "#c8ccd8"
    GRID = "#1a1a2e"

    plt.rcParams.update({
        "figure.facecolor":  BG,
        "axes.facecolor":    BG,
        "axes.edgecolor":    GRID,
        "axes.labelcolor":   FG,
        "xtick.color":       FG,
        "ytick.color":       FG,
        "text.color":        FG,
        "grid.color":        GRID,
        "grid.linestyle":    "--",
        "grid.linewidth":    0.5,
    })

    fig = plt.figure(figsize=(11, 7), facecolor=BG)
    fig.canvas.manager.set_window_title("SSTV Receiver — 145.0 MHz")

    # row 0: spectrum  |  row 1: waterfall  |  row 2: tone bar
    gs  = fig.add_gridspec(3, 1, height_ratios=[1, 3, 0.4],
                           hspace=0.08, left=0.07, right=0.97,
                           top=0.95, bottom=0.07)

    ax_spec = fig.add_subplot(gs[0])
    ax_wf   = fig.add_subplot(gs[1], sharex=ax_spec)
    ax_tone = fig.add_subplot(gs[2])

    # spectrum
    spec_line, = ax_spec.plot(freq_ax, wf_data[0], color="#14998a",
                              lw=0.8, alpha=0.9)
    ax_spec.set_ylim(-90, 0)
    ax_spec.set_ylabel("dBFS", fontsize=8)
    ax_spec.grid(True)
    ax_spec.tick_params(labelbottom=False, labelsize=7)
    ax_spec.set_title("Live Spectrum + Waterfall  —  RTL-SDR @ 145.0 MHz",
                      color=FG, fontsize=9, pad=4)

    # waterfall
    wf_img = ax_wf.imshow(
        wf_data,
        aspect="auto",
        origin="upper",
        extent=[freq_ax[0], freq_ax[-1], WATERFALL_H, 0],
        vmin=-80, vmax=-10,
        cmap=cmap_wf,
        interpolation="nearest",
    )
    ax_wf.set_ylabel("Time →", fontsize=8)
    ax_wf.set_xlabel("Frequency (MHz)", fontsize=8)
    ax_wf.tick_params(labelsize=7)

    # tone bar (maps SSTV_LO…SSTV_HI to colour)
    _tone_grad = np.linspace(0, 1, 256).reshape(1, -1)
    ax_tone.imshow(_tone_grad, aspect="auto", cmap=cmap_wf,
                   extent=[SSTV_LO, SSTV_HI, 0, 1])
    tone_marker = ax_tone.axvline(SSTV_BLACK, color="#ffffff", lw=2, alpha=0.85)
    ax_tone.set_xlim(SSTV_LO, SSTV_HI)
    ax_tone.set_ylim(0, 1)
    ax_tone.set_yticks([])
    ax_tone.set_xlabel("SSTV tone (Hz)", fontsize=8)
    ax_tone.tick_params(labelsize=7)
    # reference lines
    for hz, lbl in [(SSTV_SYNC, "sync"), (SSTV_BLACK, "blk"),
                    (SSTV_WHITE, "wht")]:
        ax_tone.axvline(hz, color=FG, lw=0.8, ls=":")
        ax_tone.text(hz + 8, 0.5, lbl, color=FG, fontsize=6, va="center")

    tone_text = ax_spec.text(0.01, 0.92, "", transform=ax_spec.transAxes,
                             fontsize=8, color="#f0c040", va="top")

    # ── optional audio output via sounddevice ─────────────────────────────────
    audio_stream   = None
    audio_proc_thr = None
    stop_audio     = threading.Event()

    if args.audio:
        global _audio_mode
        _audio_mode = True

        try:
            import sounddevice as sd
        except ImportError:
            print("sounddevice not installed — run: pip install sounddevice",
                  file=sys.stderr)
            sys.exit(1)

        # Continuous ring-buffer so the sounddevice callback never allocates.
        # Sized for ~500 ms of audio to absorb any display-loop jitter.
        _RING = int(AUDIO_RATE * 0.5)
        ring     = np.zeros(_RING, dtype=np.float32)
        ring_w   = 0   # write cursor
        ring_r   = 0   # read cursor (sounddevice callback)
        ring_lck = threading.Lock()

        def _audio_proc():
            """Dedicated thread: IQ → FM demod → ring buffer.  Runs at SDR rate."""
            while not stop_audio.is_set():
                try:
                    raw = audio_raw_q.get(timeout=0.05)
                except queue.Empty:
                    continue
                pcm = np.tanh(_demod(raw.astype(np.complex64), _audio_state))
                with ring_lck:
                    nonlocal ring_w
                    for s in pcm:
                        ring[ring_w % _RING] = s
                        ring_w += 1

        def _audio_cb(outdata, frames, _time, _status):
            nonlocal ring_r
            with ring_lck:
                avail = ring_w - ring_r
            for i in range(frames):
                if avail > 0:
                    outdata[i, 0] = ring[ring_r % _RING]
                    ring_r += 1
                    avail  -= 1
                else:
                    outdata[i, 0] = 0.0   # underrun — silent sample

        audio_proc_thr = threading.Thread(target=_audio_proc, daemon=True)
        audio_proc_thr.start()

        audio_stream = sd.OutputStream(
            samplerate=AUDIO_RATE,
            channels=1,
            dtype="float32",
            callback=_audio_cb,
            blocksize=512,   # smaller block = lower latency
        )
        audio_stream.start()
        print(f"Audio output active  —  {AUDIO_RATE} Hz mono")

    # ── SDR read thread ────────────────────────────────────────────────────────
    def _read():
        sdr.read_samples_async(sdr_callback, num_samples=FFT_SIZE * 16)

    t = threading.Thread(target=_read, daemon=True)
    t.start()

    # IIR state for spectrum smoothing
    smooth_spec = np.full(FFT_SIZE, -80.0, dtype=np.float32)
    ALPHA_S = 0.35

    # ── animation update ───────────────────────────────────────────────────────
    def update(_frame):
        nonlocal wf_data, smooth_spec, tone_hz_smooth, tone_mag_smooth

        # drain queue; process up to 3 blocks per frame
        blocks = []
        for _ in range(3):
            try:
                blocks.append(display_q.get_nowait())
            except queue.Empty:
                break
        if not blocks:
            return wf_img, spec_line, tone_marker, tone_text

        for raw in blocks:
            iq = raw.astype(np.complex64)

            # ── FFT for waterfall / spectrum ───────────────────────────────────
            n_ffts = len(iq) // FFT_SIZE
            for i in range(n_ffts):
                chunk = iq[i * FFT_SIZE:(i + 1) * FFT_SIZE]
                win   = np.hanning(FFT_SIZE)
                spec  = np.fft.fftshift(np.abs(np.fft.fft(chunk * win)))
                # dB, normalised to block amplitude
                with np.errstate(divide="ignore", invalid="ignore"):
                    spec_db = 20 * np.log10(spec / FFT_SIZE + 1e-12)
                smooth_spec = ALPHA_S * spec_db + (1 - ALPHA_S) * smooth_spec
                wf_data = np.roll(wf_data, 1, axis=0)
                wf_data[0] = smooth_spec

            # ── FM demod for tone indicator (display thread only) ─────────────
            audio_d  = _demod(iq, _disp_state)
            audio_hz = audio_d * FM_DEVIATION

            hz, mag   = detect_tone(audio_hz, float(AUDIO_RATE))
            tone_hz_smooth  += TONE_SMOOTHING * (hz  - tone_hz_smooth)
            tone_mag_smooth += TONE_SMOOTHING * (mag - tone_mag_smooth)

            if args.tones:
                level_norm = (tone_hz_smooth - SSTV_LO) / (SSTV_HI - SSTV_LO)
                print(f"tone={tone_hz_smooth:7.1f} Hz  level={level_norm:.3f}  mag={tone_mag_smooth:.1f} dBFS",
                      flush=True)

        # ── update artists ─────────────────────────────────────────────────────
        spec_line.set_ydata(smooth_spec)
        wf_img.set_data(wf_data)

        marker_x = np.clip(tone_hz_smooth, SSTV_LO, SSTV_HI)
        tone_marker.set_xdata([marker_x, marker_x])

        level_norm = (tone_hz_smooth - SSTV_LO) / (SSTV_HI - SSTV_LO)
        tone_text.set_text(
            f"SSTV  {tone_hz_smooth:.0f} Hz  |  level {level_norm:.2f}  |  {tone_mag_smooth:.1f} dBFS"
        )

        return wf_img, spec_line, tone_marker, tone_text

    ani = animation.FuncAnimation(
        fig, update, interval=50, blit=True, cache_frame_data=False
    )

    plt.show()

    # ── cleanup ────────────────────────────────────────────────────────────────
    stop_audio.set()
    if audio_stream is not None:
        audio_stream.stop()
        audio_stream.close()
    sdr.cancel_read_async()
    sdr.close()
    print("SDR closed.")


if __name__ == "__main__":
    main()
