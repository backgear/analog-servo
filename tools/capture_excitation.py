#!/usr/bin/env python3
"""Capture bounded additive torque excitation; no gain/config save changes.

Firmware enforces motion/time bounds even if this process disconnects. Requires
known-good host holding tune and trim. Leaves axis enabled, holding its target.
"""
import argparse
import json
import threading
import time
import serial
from capture_moves import cli_cmd, cli_get, reader_thread


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--amplitude', type=int, default=100)
    ap.add_argument('--hold', type=int, default=5)
    ap.add_argument('--ticks', type=int, default=1000)
    ap.add_argument('--velocity', type=int, default=0,
                    help='explicit moving test at +/-20 counts/tick max; stops on exit')
    ap.add_argument('--direct-torque', action='store_true',
                    help='bypass both loops; firmware resumes hold at new position')
    ap.add_argument('--travel', type=int, default=2000)
    ap.add_argument('--speed', type=int, default=20)
    ap.add_argument('--torque-limit', type=int, default=2000,
                    help='temporary symmetric clamp, never wider than entry clamps')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if args.direct_torque and args.velocity:
        ap.error('direct torque and moving hold are mutually exclusive')
    if not 1 <= args.travel <= 10000 or not 1 <= args.speed <= 100:
        ap.error('invalid travel/speed bounds')
    if not 1 <= args.torque_limit <= 10000:
        ap.error('torque limit must be within 1..10000')
    if abs(args.velocity) > 20 or (args.velocity and args.amplitude > 500):
        ap.error('moving tests require velocity within +/-20 and amplitude <=500')
    max_amplitude = 10000 if args.direct_torque else 1000
    if not 1 <= args.amplitude <= max_amplitude or not 1 <= args.hold <= 100 or not 1 <= args.ticks <= 10000 or args.ticks % (2*args.hold):
        ap.error('invalid amplitude, hold 1..100, duration <=10000 and divisible by 2*hold required')
    if args.direct_torque and args.amplitude >= 3276 and args.ticks > 1000:
        ap.error('high-amplitude direct tests are limited to one-second bursts; do not bypass sustained-torque protection')
    cli = serial.Serial('/dev/ttyACM1', timeout=.1)
    mon = serial.Serial('/dev/ttyACM2', timeout=.05)
    frames, fault, stop = [], threading.Event(), threading.Event()
    worker = threading.Thread(target=reader_thread, args=(mon, frames, fault, stop), daemon=True)
    limits = None
    result = None
    try:
        if cli_get(cli, 'fault.latched') != '0' or cli_get(cli, 'input.source') != 'host':
            raise RuntimeError('requires fault-free host mode')
        if cli_get(cli, 'velocity.command') != '0' or cli_get(cli, 'test.active') != '0' or cli_get(cli, 'autotune.active') != '0':
            raise RuntimeError('requires no active command/test')
        limits = (cli_get(cli, 'torque.min'), cli_get(cli, 'torque.max'))
        if int(limits[0]) >= 0 or int(limits[1]) <= 0:
            raise RuntimeError('requires bipolar entry torque clamps')
        temporary_min = max(int(limits[0]), -args.torque_limit)
        temporary_max = min(int(limits[1]), args.torque_limit)
        if args.amplitude > min(-temporary_min, temporary_max):
            raise RuntimeError('requested amplitude exceeds entry/temporary clamps')
        setup = [f'set torque.min {temporary_min}', f'set torque.max {temporary_max}']
        if cli_get(cli, 'axis.enabled') != '1':
            setup.append('enable')
        for text in setup:
            response = cli_cmd(cli, text, .05)
            if 'error=' in response:
                raise RuntimeError(response)
        worker.start()
        print(cli_cmd(cli, 'monitor start', .05))
        time.sleep(.3)
        travel, speed, mode = 200, 8, 'start'
        if args.direct_torque:
            travel, speed, mode = args.travel, args.speed, 'torque'
        if args.velocity:
            print(cli_cmd(cli, 'set velocity.command ' + str(args.velocity), .05))
            time.sleep(1.5)
            measured = int(cli_get(cli, 'velocity.raw'))
            if abs(measured - args.velocity) > 2 or fault.is_set():
                raise RuntimeError('failed to establish steady moving baseline')
            travel = min(250000, abs(args.velocity) * (args.ticks + 500))
            speed, mode = min(30, abs(args.velocity) + 5), 'moving'
        start_command = f'excite {mode} {args.amplitude} {args.hold} {args.ticks} {travel} {speed}'
        for attempt in range(5):
            response = cli_cmd(cli, start_command, .05)
            print(response)
            if 'state=running' in response:
                break
            # Encoder-boundary jitter can reject an otherwise stationary start.
            # Retry only this explicit refusal, never an ambiguous/no reply.
            if 'error=excitation requires steady enabled host operation' not in response or attempt == 4:
                raise RuntimeError(response)
            time.sleep(.1)
        deadline = time.monotonic() + args.ticks / 1000 + 1
        while time.monotonic() < deadline:
            if fault.is_set():
                raise RuntimeError('firmware fault; not clearing it')
            time.sleep(.05)
        result = cli_get(cli, 'excite.result')
        print('excite.result=' + result)
        if result != '2':
            raise RuntimeError('excitation aborted with result ' + result)
    finally:
        print(cli_cmd(cli, 'excite stop', .05))
        if args.velocity:
            print(cli_cmd(cli, 'set velocity.command 0', .05))
            time.sleep(.7)
        print(cli_cmd(cli, 'monitor stop', .05))
        stop.set()
        if worker.ident is not None:
            worker.join(timeout=1)
        if limits:
            cli_cmd(cli, 'set torque.min ' + limits[0], .05)
            cli_cmd(cli, 'set torque.max ' + limits[1], .05)
        with open(args.out, 'w') as target:
            json.dump(frames, target)
        with open(args.out + '.meta.json', 'w') as target:
            json.dump(dict(amplitude=args.amplitude, hold_ticks=args.hold,
                           duration_ticks=args.ticks, velocity_command=args.velocity,
                           temporary_torque_limit=args.torque_limit, direct_torque=args.direct_torque,
                           travel_bound=args.travel if args.direct_torque else None,
                           speed_bound=args.speed if args.direct_torque else None, result=result,
                           completed=result == '2'), target, indent=2)
        mon.close()
        cli.close()
        print(f'{len(frames)} frames saved to {args.out}')


if __name__ == '__main__':
    main()
