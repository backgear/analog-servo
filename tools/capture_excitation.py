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
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if not 1 <= args.amplitude <= 1000 or not 1 <= args.hold <= 100 or not 1 <= args.ticks <= 10000 or args.ticks % (2*args.hold):
        ap.error('require amplitude 1..1000, hold 1..100, duration <=10000 and divisible by 2*hold')
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
        for text in ['set torque.min -2000', 'set torque.max 2000', 'enable']:
            response = cli_cmd(cli, text, .05)
            if 'error=' in response:
                raise RuntimeError(response)
        worker.start()
        print(cli_cmd(cli, 'monitor start', .05))
        time.sleep(.3)
        response = cli_cmd(cli, f'excite start {args.amplitude} {args.hold} {args.ticks} 200 8', .05)
        print(response)
        if 'state=running' not in response:
            raise RuntimeError(response)
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
                           duration_ticks=args.ticks, result=result,
                           completed=result == '2'), target, indent=2)
        mon.close()
        cli.close()
        print(f'{len(frames)} frames saved to {args.out}')


if __name__ == '__main__':
    main()
