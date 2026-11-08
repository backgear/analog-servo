#!/usr/bin/env python3
"""Capture bench telemetry across a small, repeatable sequence of moves:
one sharp position step (the built-in test-signal generator, unramped)
and one triangular velocity ramp (a brief velocity.command pulse through
Ramp.c's own accel limiter). Saves the decoded monitor-port frames as
JSON for tools/plot_telemetry.py or any other offline analysis.

Why a position STEP uses `test square position` instead of just
`set position.command <target>`: the latter ramps the velocity reference
up from 0 via PositionMoveTick/Ramp.c, so torque demand stays low at
first -- under a stiff tune or real static friction that can stall out
past FAULT_ZERO_ENCODER's window before ever producing real motion.
`test square position` feeds the full step error straight into the
cascade with no ramp (see Signal.h/main.c), so the proportional term
demands enough torque to break static friction immediately.

Before touching anything, this resets two bits of state that are NOT
reset by enable/disable and so can otherwise bleed into a move from
whatever was last commanded in an entirely different session:
  - velocity.command (a persistent host-set target, see main.h's
    AppSetCommandedVelocity)
  - position.command (PositionMove.c's point-to-point move target --
    "disabling mid-move is a pause, not a cancel", main.c) -- snapped to
    the CURRENT position so there's zero distance left to chase, rather
    than cancelled outright (no CLI command does that; zero distance has
    the same practical effect).
A stale value in either one, left over from an earlier aborted attempt,
can otherwise drive large unplanned motion the instant the axis enables
-- this is not hypothetical, it is exactly what happened once during
this script's own development (see docs/monitor-protocol.md history /
git log for this file if you want the full story).

Usage:
    python3 capture_moves.py --out /tmp/two_moves_frames.json

Any CLI variable this script changes outside what it's demonstrating
(trajectory.accel_limit, for the velocity move) is restored before exit.
Aborts and clears the firmware fault, cleanly, on the first fault seen.
"""
import argparse
import json
import sys
import threading
import time

import serial

from monitor_frame import FrameSync


def cli_cmd(cli, text, wait=0.15):
    cli.reset_input_buffer()
    cli.write((text + "\r\n").encode())
    time.sleep(wait)
    return cli.read(4000).decode(errors="replace")


def cli_get(cli, name):
    resp = cli_cmd(cli, "get " + name)
    for line in resp.splitlines():
        if line.startswith(name + "="):
            return line.split("=", 1)[1]
    raise RuntimeError("no response for get %s: %r" % (name, resp))


def reader_thread(mon, captured, fault_seen, stop_event):
    sync = FrameSync()
    while not stop_event.is_set():
        data = mon.read(max(1, mon.in_waiting) if mon.in_waiting else 1)
        if not data:
            continue
        for frame in sync.feed(data):
            frame["t"] = time.monotonic()
            captured.append(frame)
            if frame["fault_latched"]:
                fault_seen.set()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cli-port", default="/dev/ttyACM1")
    ap.add_argument("--mon-port", default="/dev/ttyACM2")
    ap.add_argument("--cli-baud", type=int, default=115200)
    ap.add_argument("--mon-baud", type=int, default=2500000)
    ap.add_argument("--out", default="/tmp/two_moves_frames.json")
    ap.add_argument("--step-amplitude", type=int, default=1024,
                     help="position-step amplitude, encoder counts")
    ap.add_argument("--step-half-period-ticks", type=int, default=400,
                     help="also used as the one-shot duration (one half-cycle only)")
    ap.add_argument("--velocity-peak", type=int, default=28,
                     help="peak velocity.command, counts/tick -- keep under "
                          "trajectory.velocity_limit and velocity.limit")
    ap.add_argument("--velocity-accel", type=int, default=250,
                     help="trajectory.accel_limit used only for the velocity move "
                          "(lower = more visibly sloped triangle); restored after")
    args = ap.parse_args()

    cli = serial.Serial(args.cli_port, args.cli_baud, timeout=0.5)
    time.sleep(0.3)
    cli.reset_input_buffer()

    if cli_get(cli, "fault.latched") != "0":
        print("ABORT: fault already latched before starting:", cli_cmd(cli, "status"))
        sys.exit(1)

    captured = []
    fault_seen = threading.Event()
    stop_event = threading.Event()

    mon = serial.Serial(args.mon_port, args.mon_baud, timeout=0.2)
    time.sleep(0.2)
    mon.reset_input_buffer()
    t = threading.Thread(target=reader_thread, args=(mon, captured, fault_seen, stop_event), daemon=True)
    t.start()

    print(cli_cmd(cli, "set velocity.command 0"))
    cur_pos = int(cli_get(cli, "position.raw"))
    print(cli_cmd(cli, "set position.command %d" % cur_pos))
    print(cli_cmd(cli, "monitor start"))
    print(cli_cmd(cli, "enable"))
    time.sleep(0.3)

    # ---- Move 1: a sharp position step ----
    print("Move 1 (sharp position step, test square position %d/%d/%d):" %
          (args.step_amplitude, args.step_half_period_ticks, args.step_half_period_ticks))
    print(cli_cmd(cli, "test square position %d %d %d" %
                  (args.step_amplitude, args.step_half_period_ticks, args.step_half_period_ticks)))

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if fault_seen.is_set():
            break
        if cli_get(cli, "test.active") == "0":
            break
        time.sleep(0.05)
    time.sleep(0.3)

    if fault_seen.is_set():
        print("ABORT: fault latched during move 1:", cli_cmd(cli, "status"))
        print(cli_cmd(cli, "test stop"))
        print(cli_cmd(cli, "fault clear"))
        print(cli_cmd(cli, "disable"))
        print(cli_cmd(cli, "monitor stop"))
        sys.exit(1)

    # Clean handoff back to normal Mode B operation: AppSetEnabled(1)
    # resyncs sPositionTargetAccum = EncoderGetPositionCounts() (main.c),
    # so disable->enable means no stale target and no snap-back.
    print(cli_cmd(cli, "disable"))
    time.sleep(0.2)
    print(cli_cmd(cli, "enable"))
    time.sleep(0.2)

    # ---- Move 2: a velocity move with a triangular ramp ----
    orig_accel = int(cli_get(cli, "trajectory.accel_limit"))
    print(cli_cmd(cli, "set trajectory.accel_limit %d" % args.velocity_accel))
    ramp_seconds = args.velocity_peak / args.velocity_accel
    print("Move 2 (triangular velocity ramp): accel_limit %d (restoring %d after), "
          "peak %d ct/tick, ~%.0fms up/down" %
          (args.velocity_accel, orig_accel, args.velocity_peak, ramp_seconds * 1000))
    # cli_cmd()'s own ack-wait would dwarf a short ramp -- fire both writes
    # back-to-back with no wait in between so the real hold at peak is just
    # the sleep(ramp_seconds), then read both acks together after.
    cli.write(("set velocity.command %d\r\n" % args.velocity_peak).encode())
    time.sleep(ramp_seconds)
    cli.write(b"set velocity.command 0\r\n")
    time.sleep(0.2)
    print(cli.read(500).decode(errors="replace"))
    time.sleep(ramp_seconds + 0.4)   # ramp back down + settle
    print(cli_cmd(cli, "set trajectory.accel_limit %d" % orig_accel))

    if fault_seen.is_set():
        print("ABORT: fault latched during move 2:", cli_cmd(cli, "status"))
        print(cli_cmd(cli, "fault clear"))

    time.sleep(0.2)
    print(cli_cmd(cli, "disable"))
    print(cli_cmd(cli, "monitor stop"))
    stop_event.set()
    t.join(timeout=1.0)
    mon.close()
    cli.close()

    with open(args.out, "w") as f:
        json.dump(captured, f)
    print("Captured %d frames -> %s" % (len(captured), args.out))


if __name__ == "__main__":
    main()
