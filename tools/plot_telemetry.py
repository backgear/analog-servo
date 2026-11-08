#!/usr/bin/env python3
"""Render a styled position/velocity/torque chart from a monitor-port
capture (the JSON frame list tools/capture_moves.py or any other script
built on monitor_frame.FrameSync produces).

One column per --window (label:start:end, seconds, relative to the
capture's first frame); with no --window given, plots the whole capture
as a single column. Each column is three stacked panels (position,
velocity, torque) rather than a dual-axis chart -- different units,
different scales, so they get separate axes, not one shared one.

Usage:
    python3 plot_telemetry.py --frames /tmp/two_moves_frames.json \\
        --window "Move 1 - sharp position step:0.55:3.25" \\
        --window "Move 2 - triangular velocity ramp:6.00:6.70" \\
        --out /tmp/two_moves_telemetry.png
"""
import argparse
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
BLUE = "#2a78d6"
ORANGE = "#eb6834"


def parse_window(spec):
    label, start, end = spec.rsplit(":", 2)
    return label, float(start), float(end)


def panel(ax, data, t0, cmd_key, act_key, color, cmd_label, act_label, ylabel, baseline=0):
    ts = [f["t"] - t0 for f in data]
    ax.axhline(0, color=BASELINE, linewidth=1, zorder=1)
    if cmd_key:
        ax.plot(ts, [f[cmd_key] - baseline for f in data], color=INK_MUTED, linewidth=1.6,
                 linestyle=(0, (4, 2)), label=cmd_label, zorder=2)
    ax.plot(ts, [f[act_key] - baseline for f in data], color=color, linewidth=2, label=act_label, zorder=3)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", linewidth=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.legend(frameon=False, fontsize=8, loc="upper right", labelcolor=INK_SECONDARY)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", required=True, help="JSON frame list to plot")
    ap.add_argument("--window", action="append", default=[],
                     metavar="LABEL:START:END",
                     help="one column per window, times in seconds relative to the "
                          "capture's first frame; repeatable. Default: whole capture, one column.")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="analog-servo-f103 bench telemetry")
    args = ap.parse_args()

    frames = json.load(open(args.frames))
    if not frames:
        raise SystemExit("no frames in %s" % args.frames)
    base_t = frames[0]["t"]
    for f in frames:
        f["t"] -= base_t

    windows = [parse_window(w) for w in args.window]
    if not windows:
        windows = [("", 0.0, frames[-1]["t"])]

    n = len(windows)
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": INK_SECONDARY,
        "text.color": INK_PRIMARY, "xtick.color": INK_MUTED, "ytick.color": INK_MUTED,
        "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
        "axes.titlecolor": INK_PRIMARY, "grid.color": GRID, "grid.linewidth": 0.8,
    })
    fig, axes = plt.subplots(3, n, figsize=(5.5 * n, 7.5), sharex="col", squeeze=False)

    any_fault = False
    for col, (label, start, end) in enumerate(windows):
        data = [f for f in frames if start <= f["t"] <= end]
        if not data:
            raise SystemExit("window %r matched no frames" % (label,))
        any_fault = any_fault or any(f["fault_latched"] for f in data)
        wt0 = data[0]["t"]
        pos_baseline = data[0]["position"]
        panel(axes[0, col], data, wt0, "cmd_position", "position", BLUE, "commanded", "actual",
              "position (ct, rel. to window start)", baseline=pos_baseline)
        panel(axes[1, col], data, wt0, "cmd_velocity", "velocity", ORANGE, "commanded", "actual", "velocity (ct/tick)")
        panel(axes[2, col], data, wt0, None, "torque", INK_PRIMARY, None, "torque.command", "torque")
        if label:
            axes[0, col].set_title(label)
        axes[2, col].set_xlabel("time (s)")

    fig.suptitle(args.title, color=INK_PRIMARY, fontsize=12, fontweight="bold", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(args.out, dpi=150, facecolor=SURFACE)
    print("Saved", args.out)
    if any_fault:
        print("WARNING: at least one plotted window contains a fault_latched frame")


if __name__ == "__main__":
    main()
