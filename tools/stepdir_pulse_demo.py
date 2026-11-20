#!/usr/bin/env python3
"""Render a readme-style telemetry chart for Mode A (step/dir) tracking --
but SYNTHETIC/SIMULATED, not a bench capture: there's no real hardware
involved anywhere in this script.

Compiles firmware/test/support/stepdir_pulse_demo_main.c directly with the
host cc (Control.c has no HAL dependency, same reason `pio test -e native`
can run it on the host), runs it twice -- once against a clean synthetic
step/dir pulse train, once with short-term pulse-timing jitter injected
(see firmware/test/support/PulseTrain.h) -- and hands the combined output
straight to tools/plot_telemetry.py, UNCHANGED: the demo driver prints one
JSON object per tick in exactly tools/monitor_frame.py's decoded-frame
shape, so the existing renderer needs no changes at all to plot it.

This models only the quantized edge-COUNT a real pulse train would
produce (see PulseTrain.h's own disclaimer) -- not real TIM2 edge-counting
or real electrical jitter. Mark any output from this script as synthetic
wherever it's used; never present it as bench-validated.

Usage:
    python3 stepdir_pulse_demo.py --out /tmp/stepdir-pulse-demo.png
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


def run_segment(binary, rate, jitter, ticks, t0):
    out = subprocess.run([str(binary), str(rate), str(jitter), str(ticks), str(t0)],
                          check=True, capture_output=True, text=True)
    return json.loads(out.stdout)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=float, default=2500.0,
                     help="synthetic step/dir pulse rate, edges/sec (default: 2500)")
    ap.add_argument("--jitter", type=float, default=0.3,
                     help="jitter fraction for the second (noisy) segment (default: 0.3)")
    ap.add_argument("--duration", type=float, default=2.0,
                     help="seconds per segment (default: 2.0)")
    ap.add_argument("--out", required=True, help="output PNG path")
    args = ap.parse_args()

    ticks = int(args.duration * 1000)
    gap = 1.0   # seconds of dead space between the two segments' time windows

    with tempfile.TemporaryDirectory() as build_dir_str:
        build_dir = pathlib.Path(build_dir_str)
        binary = build_driver(build_dir)

        clean = run_segment(binary, args.rate, 0.0, ticks, 0.0)
        noisy_t0 = args.duration + gap
        noisy = run_segment(binary, args.rate, args.jitter, ticks, noisy_t0)

        frames = clean + noisy
        frames_path = build_dir / "frames.json"
        frames_path.write_text(json.dumps(frames))

        cmd = [
            sys.executable, str(PLOT_SCRIPT),
            "--frames", str(frames_path),
            "--window", "Clean pulses (synthetic):0:%.3f" % args.duration,
            "--window", "With jitter injected (synthetic):%.3f:%.3f" % (noisy_t0, noisy_t0 + args.duration),
            "--title", "analog-servo Mode A (step/dir) tracking -- SIMULATED, not a bench capture",
            "--out", args.out,
        ]
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
