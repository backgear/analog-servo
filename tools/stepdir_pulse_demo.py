#!/usr/bin/env python3
"""Regenerate the readme-style telemetry charts for Mode A (step/dir)
tracking -- all SYNTHETIC/SIMULATED, not bench captures: there's no real
hardware involved anywhere in this script.

Compiles firmware/test/support/stepdir_pulse_demo_main.c directly with the
host cc (Control.c has no HAL dependency, same reason `pio test -e native`
can run it on the host), runs it against a few synthetic scenarios, and
hands the combined output straight to tools/plot_telemetry.py UNCHANGED:
the demo driver prints one JSON object per tick in exactly
tools/monitor_frame.py's decoded-frame shape, so the existing renderer
needs no changes at all to plot it.

This models only the quantized edge-COUNT a real pulse train would
produce (see PulseTrain.h's own disclaimer) -- not real TIM2 edge-counting
or real electrical jitter. Mark any output from this script as synthetic
wherever it's used; never present it as bench-validated.

This is also the one-command way to regenerate every assets/stepdir-*.png
after a Control.c change -- run it with no arguments:

    python3 tools/stepdir_pulse_demo.py

writes assets/stepdir-pulse-demo.png, assets/stepdir-ramp-demo.png, and
assets/stepdir-filter-shift-demo.png in place. Pass --out-dir to write
elsewhere instead (e.g. for a one-off comparison without touching the
checked-in images).
"""
import argparse
import json
import pathlib
import subprocess
import sys
import tempfile

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
FIRMWARE = REPO_ROOT / "firmware"
DRIVER_SRC = FIRMWARE / "test" / "support" / "stepdir_pulse_demo_main.c"
CONTROL_SRC = FIRMWARE / "src" / "Control.c"
PLOT_SCRIPT = REPO_ROOT / "tools" / "plot_telemetry.py"


def build_driver(build_dir):
    binary = build_dir / "stepdir_pulse_demo"
    cmd = [
        "cc", "-std=gnu11", "-O2", "-Wall",
        "-I", str(FIRMWARE / "include"),
        "-I", str(FIRMWARE / "test" / "support"),
        str(DRIVER_SRC), str(CONTROL_SRC),
        "-lm", "-o", str(binary),
    ]
    subprocess.run(cmd, check=True)
    return binary


def run_segment(binary, rate, ticks, t0, jitter=0.0, ramp_seconds=0.0, filter_shift=0):
    cmd = [str(binary), "--rate", str(rate), "--ticks", str(ticks),
           "--jitter", str(jitter), "--t0", str(t0)]
    if ramp_seconds:
        cmd += ["--ramp-seconds", str(ramp_seconds)]
    if filter_shift:
        cmd += ["--filter-shift", str(filter_shift)]
    out = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return json.loads(out.stdout)


def render(frames, windows, out, title, build_dir):
    frames_path = build_dir / ("frames-%d.json" % id(frames))
    frames_path.write_text(json.dumps(frames))
    cmd = [sys.executable, str(PLOT_SCRIPT), "--frames", str(frames_path)]
    for label, start, end in windows:
        cmd += ["--window", "%s:%.3f:%.3f" % (label, start, end)]
    cmd += ["--title", title, "--out", str(out)]
    subprocess.run(cmd, check=True)   # plot_telemetry.py itself prints "Saved <out>"


def build_constant_rate_demo(binary, out, build_dir):
    """Clean vs. jittered pulse train at a fixed rate -- the original demo:
    does steady-state tracking stay smooth with/without pulse-timing jitter."""
    duration, gap = 2.0, 1.0
    ticks = int(duration * 1000)
    clean = run_segment(binary, 2500, ticks, 0.0, jitter=0.0)
    t0 = duration + gap
    noisy = run_segment(binary, 2500, ticks, t0, jitter=0.3)
    windows = [("Clean pulses (synthetic)", 0.0, duration),
               ("With jitter injected (synthetic)", t0, t0 + duration)]
    render(clean + noisy, windows, out,
           "analog-servo Mode A (step/dir) tracking -- SIMULATED, not a bench capture",
           build_dir)


def build_ramp_demo(binary, out, build_dir):
    """An accelerating pulse train (an indexer ramping its own step rate
    up) -- does Control.c's purely-reactive Mode A tracking (no
    feedforward reaches it, see main.c) pick up the acceleration without
    excess lag, clean vs. jittered?"""
    rate, ramp_seconds, duration, gap = 20000, 0.1, 1.0, 1.0
    ticks = int(duration * 1000)
    clean = run_segment(binary, rate, ticks, 0.0, jitter=0.0, ramp_seconds=ramp_seconds)
    t0 = duration + gap
    noisy = run_segment(binary, rate, ticks, t0, jitter=0.3, ramp_seconds=ramp_seconds)
    windows = [("Ramped rate, clean (synthetic)", 0.0, duration),
               ("Ramped rate, with jitter (synthetic)", t0, t0 + duration)]
    render(clean + noisy, windows, out,
           "analog-servo Mode A (step/dir) tracking an ACCELERATING pulse train -- SIMULATED",
           build_dir)


def build_filter_shift_demo(binary, out, build_dir):
    """Does smoothing the measured velocity (velocityFilterShift) cut the
    reverse-torque chatter from pulse-train quantization? shift=1 is a
    mild, inconsistent improvement; shift=3 is a known trap with this
    gain set (no feedforward reaches Mode A to offset the added filter
    lag) -- shown here deliberately, not just described in a comment."""
    rate, duration, gap = 2500, 1.0, 0.5
    ticks = int(duration * 1000)
    segments = []
    windows = []
    t = 0.0
    for label, shift in (("filterShift=0 (synthetic)", 0),
                         ("filterShift=1 (synthetic)", 1),
                         ("filterShift=3 (synthetic, destabilized)", 3)):
        segments += run_segment(binary, rate, ticks, t, jitter=0.3, filter_shift=shift)
        windows.append((label, t, t + duration))
        t += duration + gap
    render(segments, windows, out,
           "analog-servo Mode A (step/dir): velocityFilterShift vs. torque chatter -- SIMULATED",
           build_dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=str(REPO_ROOT / "assets"),
                     help="directory to write the three PNGs into (default: assets/)")
    args = ap.parse_args()
    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as build_dir_str:
        build_dir = pathlib.Path(build_dir_str)
        binary = build_driver(build_dir)
        build_constant_rate_demo(binary, out_dir / "stepdir-pulse-demo.png", build_dir)
        build_ramp_demo(binary, out_dir / "stepdir-ramp-demo.png", build_dir)
        build_filter_shift_demo(binary, out_dir / "stepdir-filter-shift-demo.png", build_dir)


if __name__ == "__main__":
    main()
