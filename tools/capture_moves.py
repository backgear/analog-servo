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
Aborts on a firmware fault without clearing it. Always attempts to stop motion
and restore the entry acceleration setting. --leave-enabled resumes a stationary
hold on success; otherwise exits disabled. The short velocity pulse is host-timed,
not a firmware watchdog: use only on an attended, appropriate bench.
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
    ap.add_argument('--leave-enabled', action='store_true',
                    help='resume stationary hold on successful completion')
    args = ap.parse_args()
    if args.step_amplitude <= 0 or args.step_half_period_ticks <= 0 or not 1 <= args.velocity_peak <= 30 or args.velocity_accel <= 0:
        ap.error('positive step/duration/acceleration required; velocity peak must be 1..30')

    cli = serial.Serial(args.cli_port, args.cli_baud, timeout=0.5)
    time.sleep(0.3)
    cli.reset_input_buffer()

    captured = []
    fault_seen, stop_event = threading.Event(), threading.Event()
    mon, t, orig_accel = None, None, None
    completed = False
    metadata = dict(parameters=vars(args), completed=False)
    try:
        settings = cli_cmd(cli, 'settings')
        metadata['entry_settings'] = settings
        parsed = dict(line.split('=', 1) for line in settings.splitlines() if '=' in line)
        if parsed.get('fault.latched') != '0' or parsed.get('input.source') != 'host':
            raise RuntimeError('requires fault-free host mode')
        if parsed.get('velocity.command') != '0' or any(parsed.get(k) != '0' for k in ('test.active', 'autotune.active', 'excite.active')):
            raise RuntimeError('requires no active command/test')
        orig_accel = int(parsed['trajectory.accel_limit'])
        mon = serial.Serial(args.mon_port, args.mon_baud, timeout=.05)
        mon.reset_input_buffer()
        t = threading.Thread(target=reader_thread, args=(mon, captured, fault_seen, stop_event), daemon=True)
        t.start()

        def checked(text):
            response = cli_cmd(cli, text, .05)
            print(response)
            if any(line.startswith('error=') for line in response.splitlines()):
                raise RuntimeError(response)
            return response

        def wait(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                if fault_seen.is_set():
                    raise RuntimeError('firmware fault; not clearing it')
                if captured and (abs(captured[-1]['velocity']) > 60 or time.monotonic()-captured[-1]['t'] > .5):
                    raise RuntimeError('speed/stale-telemetry guard')
                time.sleep(.005)

        checked('set velocity.command 0')
        checked('set position.command '+cli_get(cli, 'position.raw'))
        checked('monitor start')
        checked('enable')
        wait(.3)
        print('Move 1: sharp position step')
        checked(f'test square position {args.step_amplitude} {args.step_half_period_ticks} {args.step_half_period_ticks}')
        wait(args.step_half_period_ticks/1000+.4)
        if cli_get(cli, 'test.active') != '0':
            raise RuntimeError('position test did not finish')

        checked('disable')
        wait(.2)
        checked('enable')
        wait(.2)
        checked(f'set trajectory.accel_limit {args.velocity_accel}')
        ramp_seconds = args.velocity_peak/args.velocity_accel
        print(f'Move 2: triangular velocity ramp, nominal peak {args.velocity_peak}, up/down {ramp_seconds*1000:.0f} ms')
        cli.write((f'set velocity.command {args.velocity_peak}\r\n').encode())
        try:
            wait(ramp_seconds)
        finally:
            cli.write(b'set velocity.command 0\r\n')
        wait(ramp_seconds+.7)
        completed = True
    finally:
        try:
            if orig_accel is not None:
                cli_cmd(cli, 'test stop', .05)
                cli_cmd(cli, 'set velocity.command 0', .05)
                cli_cmd(cli, f'set trajectory.accel_limit {orig_accel}', .05)
                if completed and args.leave_enabled and not fault_seen.is_set():
                    cli_cmd(cli, 'set position.command '+cli_get(cli, 'position.raw'), .05)
                    cli_cmd(cli, 'enable', .05)
                    time.sleep(.3)
                else:
                    cli_cmd(cli, 'disable', .05)
                cli_cmd(cli, 'monitor stop', .05)
                metadata['exit_settings'] = cli_cmd(cli, 'settings')
        finally:
            stop_event.set()
            if t is not None:
                t.join(timeout=1)
            if mon is not None:
                mon.close()
            cli.close()
            metadata['completed'] = completed
            with open(args.out, 'w') as target:
                json.dump(captured, target)
            with open(args.out+'.meta.json', 'w') as target:
                json.dump(metadata, target, indent=2)
            print(f'Captured {len(captured)} frames -> {args.out}')


if __name__ == "__main__":
    main()
