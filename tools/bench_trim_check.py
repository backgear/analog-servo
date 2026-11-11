#!/usr/bin/env python3
"""Short, supervised trim observations on the F103 bench.

Requires a known stable host-mode holding tune and correct output polarity.
Disabled output is NOT a hardware zero: trim remains active. No unattended use.
Uses a host motion guard, not a firmware safety guarantee. Does not save config.
"""
import argparse
import json
import time
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trims', type=int, nargs='+', default=[300])
    parser.add_argument('--seconds', type=float, default=1.0)
    args = parser.parse_args()
    if not 0 < args.seconds <= 5 or any(abs(t) > 500 for t in args.trims):
        parser.error('require duration (0,5] seconds and trims within +/-500')
    cli = serial.Serial('/dev/ttyACM1', timeout=.02)
    log = []

    def command(text):
        cli.reset_input_buffer()
        cli.write((text + '\n').encode())
        time.sleep(.015)
        reply = cli.read(8192).decode()
        if 'error=' in reply:
            raise RuntimeError((text, reply))
        return reply

    def get(name):
        reply = command('get ' + name)
        return int(next(line.split('=', 1)[1] for line in reply.splitlines()
                        if line.startswith(name + '=')))

    def sample():
        reply = command('sample')
        for line in reply.splitlines():
            try:
                values = [int(x) for x in line.removeprefix('sample=').split(',')]
            except ValueError:
                continue
            if len(values) == 6:
                return values
        raise RuntimeError(('bad sample', reply))

    if get('fault.latched') or not get('axis.enabled'):
        raise RuntimeError('requires fault-free holding axis')
    low, high, original = get('torque.min'), get('torque.max'), get('output.trim')
    try:
        command('set torque.min -2000')
        command('set torque.max 2000')
        for trim in args.trims:
            command('set output.trim ' + str(trim))
            command('enable')
            time.sleep(1.5)
            start = sample()
            if abs(start[2]) > 1:
                raise RuntimeError('not stationary before observation')
            command('disable')
            samples = []
            try:
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    row = sample()
                    samples.append(row)
                    if abs(row[2]) > 8 or abs(row[1] - start[1]) > 500 or row[5] & 2:
                        raise RuntimeError('motion/fault guard')
            finally:
                command('enable')
            entry = dict(trim=trim, start=start, samples=samples)
            log.append(entry)
            print(json.dumps(entry), flush=True)
    finally:
        command('set output.trim ' + str(original))
        command('enable')
        command('set torque.min ' + str(low))
        command('set torque.max ' + str(high))
        cli.close()
        with open('/tmp/servo-trim-check.json', 'w') as target:
            json.dump(log, target, indent=2)


if __name__ == '__main__':
    main()
