#!/usr/bin/env python3
"""Bounded bidirectional position-step A/B tests; restore entry gains on exit.

Uses the existing finite position signal, not the trajectory generator (whose
catch-up behavior is a separate issue). Records scorecard Recorder directories
for bench_report.py. Does not save config, learn polarity, or clear faults.
"""
import argparse
import json
from pathlib import Path
import threading
import time
import numpy as np
import serial
from monitor_frame import FrameSync
from scorecard_core import Recorder


def trial_metrics(frames, expected_ticks=800, tolerance=5, stable_ticks=50):
    active = [f for f in frames if f['synthetic']]
    if len(active) != expected_ticks or any(((b['seq']-a['seq']) & 0xffffffff) != 1 for a, b in zip(active, active[1:])):
        raise ValueError('incomplete/noncontiguous position-test capture')
    target = active[0]['cmd_position']
    if any(f['cmd_position'] != target for f in active):
        raise ValueError('position target changed within test')
    # First captured actual position is after at most one tick of motion.
    direction = 1 if target >= active[0]['position'] else -1
    error = np.array([target-f['position'] for f in active])
    inside = abs(error) <= tolerance
    # Report first entry that persists through the rest of the observation,
    # with >=50 ms actually observed inside the band.
    outside = np.flatnonzero(~inside)
    entry = int(outside[-1]+1) if len(outside) else 0
    settle = entry if len(active)-entry >= stable_ticks else None
    tail = active[-100:]
    torque = np.array([f['torque'] for f in active], dtype=float)
    return dict(target=target, frames=len(active), settle_ms=settle,
                overshoot=int(max(0, np.max(-direction*error))),
                final_error_counts=float(np.mean(error[-100:])),
                final_error_abs_counts=float(np.mean(abs(error[-100:]))),
                peak_speed=max(abs(f['velocity']) for f in active),
                peak_torque=float(max(abs(torque))),
                saturation_percent=float(np.mean(abs(torque) >= 9990)*100),
                tail_torque_ac_rms=float(np.std([f['torque'] for f in tail])),
                tail_position_span=max(f['position'] for f in tail)-min(f['position'] for f in tail))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    ap.add_argument('--repeats', type=int, default=3, help='per direction per condition')
    ap.add_argument('--amplitude', type=int, default=64)
    ap.add_argument('--tunes', nargs='+', choices=['baseline', 'moderate', 'stiffer'],
                    default=['baseline', 'moderate', 'stiffer'])
    ap.add_argument('--deadbands', type=int, nargs='+', choices=[0, 1, 2], default=[1, 0])
    ap.add_argument('--idle-seconds', type=float, default=2)
    ap.add_argument('--firmware-label', default='unspecified')
    args = ap.parse_args()
    if not .2 <= args.idle_seconds <= 10:
        ap.error('idle observation must be .2..10 seconds')
    if not 1 <= args.repeats <= 10 or not 16 <= args.amplitude <= 1024:
        ap.error('require 1..10 repeats, 16..1024 position counts')
    root = Path(args.out)
    root.mkdir(parents=True, exist_ok=False)
    cli = serial.Serial('/dev/ttyACM1', timeout=.02, write_timeout=.5)
    mon = serial.Serial('/dev/ttyACM2', timeout=.02)
    frames, stop, problem = [], threading.Event(), []
    context = dict(recorder=None, guarding=False, origin=0)
    lock = threading.Lock()

    def command(text):
        cli.reset_input_buffer()
        cli.write((text+'\n').encode())
        response = bytearray()
        until = time.monotonic()+1
        while time.monotonic() < until:
            data = cli.read(max(1, cli.in_waiting))
            if data:
                response.extend(data)
            elif response:
                break
        reply = response.decode(errors='replace')
        rec = context['recorder']
        if rec:
            rec.event('command', command=text, response=reply,
                      seq=frames[-1]['seq'] if frames else None)
        if not response or any(line.startswith('error=') for line in reply.splitlines()):
            raise RuntimeError((text, reply))
        return reply

    def get(name):
        reply = command('get '+name)
        return next(line.split('=', 1)[1] for line in reply.splitlines() if line.startswith(name+'='))

    def reader():
        sync = FrameSync()
        try:
            while not stop.is_set():
                data = mon.read(max(1, mon.in_waiting))
                if not data:
                    continue
                decoded = sync.feed(data)
                now = time.monotonic()
                with lock:
                    for f in decoded:
                        f['received'] = now
                        frames.append(f)
                        if f['fault_latched']:
                            problem.append('firmware fault: '+f['fault_reason'])
                        if context['guarding'] and (abs(f['velocity']) > (20 if args.amplitude <= 128 else 60) or abs(f['position']-context['origin']) > max(256, args.amplitude+128)):
                            problem.append('speed/travel guard')
                    if context['recorder']:
                        context['recorder'].write(data, decoded)
        except Exception as exc:
            problem.append('reader failed: '+str(exc))

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if problem:
                raise RuntimeError(problem[0])
            if frames and time.monotonic()-frames[-1]['received'] > .5:
                raise RuntimeError('stale telemetry')
            time.sleep(.01)

    worker = threading.Thread(target=reader, daemon=True)
    saved, results = {}, []
    keys = ('position.kp', 'velocity.kp', 'velocity.ki', 'velocity.deadband')
    try:
        settings = command('settings')
        (root/'entry-settings.txt').write_text(settings)
        settings = dict(line.split('=', 1) for line in settings.splitlines() if '=' in line)
        if settings['fault.latched'] != '0' or settings['axis.enabled'] != '1' or settings['input.source'] != 'host':
            raise RuntimeError('requires fault-free enabled host hold')
        if settings['velocity.command'] != '0' or any(settings[k] != '0' for k in ('test.active', 'autotune.active', 'excite.active')):
            raise RuntimeError('active command/test; refusing to interfere')
        if settings['torque.min'] != '-10000' or settings['torque.max'] != '10000':
            raise RuntimeError('requires recorded +/-10000 torque safety clamps')
        saved = {k: settings[k] for k in keys}
        # Cancel a stale point trajectory at its present position before any enable.
        command('set position.command '+get('position.raw'))
        worker.start()
        command('monitor start')
        wait(.3)
        candidates = [('baseline', int(saved['position.kp']), int(saved['velocity.kp']), int(saved['velocity.ki'])),
                      ('moderate', 16, 289000, 82), ('stiffer', 21, 365000, 82)]
        for deadband in args.deadbands:
            for name, pos, kp, ki in candidates:
                if name not in args.tunes:
                    continue
                label = f'{name}-db{deadband}'
                rec = Recorder(root/label)
                with lock:
                    context['recorder'] = rec
                rec.event('session', tool='compare_tunes', tune=name, position_kp=pos,
                          velocity_kp=kp, velocity_ki=ki, deadband=deadband,
                          amplitude=args.amplitude, observation_ticks=800,
                          test='unramped position signal', cpr_estimate=4000,
                          firmware_label=args.firmware_label, idle_seconds=args.idle_seconds)
                trials = []
                condition_complete = False
                try:
                    for key, value in zip(keys, [pos, kp, ki, deadband]):
                        command(f'set {key} {value}')
                    command('enable')  # clear PI and snap normal hold reference
                    wait(.5)
                    for repeat in range(args.repeats):
                        for direction in [1, -1]:
                            origin = int(get('position.raw'))
                            with lock:
                                first = len(frames)
                                context.update(guarding=True, origin=origin)
                            amplitude = direction*args.amplitude
                            rec.event('position_step', amplitude=amplitude, repeat=repeat,
                                      origin=origin, seq=frames[-1]['seq'])
                            command(f'test square position {amplitude} 800 800')
                            wait(1.15)
                            with lock:
                                observed = list(frames[first:])
                                context['guarding'] = False
                            metrics = trial_metrics(observed)
                            metrics.update(direction=direction, repeat=repeat)
                            trials.append(metrics)
                            rec.event('step_metrics', **metrics)
                            print(label, json.dumps(metrics), flush=True)
                    wait(args.idle_seconds)
                    with lock:
                        last = frames[-1]['seq']
                        idle = [f for f in frames if ((last-f['seq']) & 0xffffffff) < args.idle_seconds*1000]
                    summary = dict(tune=name, position_kp=pos, velocity_kp=kp, velocity_ki=ki,
                                   deadband=deadband, trials=trials,
                                   firmware_label=args.firmware_label,
                                   idle_observation_seconds=args.idle_seconds,
                                   idle_frames=len(idle),
                                   idle_torque_mean=float(np.mean([f['torque'] for f in idle])),
                                   idle_speed_rms=float(np.sqrt(np.mean([f['velocity']**2 for f in idle]))),
                                   idle_torque_ac_rms=float(np.std([f['torque'] for f in idle])),
                                   idle_position_span=max(f['position'] for f in idle)-min(f['position'] for f in idle))
                    settles = [r['settle_ms'] for r in trials if r['settle_ms'] is not None]
                    rec.event('confirm_summary', settles=settles,
                              overshoots=[r['overshoot'] for r in trials], fails=len(trials)-len(settles))
                    results.append(summary)
                    condition_complete = True
                finally:
                    with lock:
                        context['recorder'] = None
                    rec.close(dict(trials=trials, aborted=not condition_complete))
    finally:
        try:
            if saved:
                command('test stop')
                command('set velocity.command 0')
                for key, value in saved.items():
                    command(f'set {key} {value}')
                if get('fault.latched') == '0':
                    # All tests are finite; cancel residual motion and reset hold at actual.
                    command('set position.command '+get('position.raw'))
                    command('enable')
                    time.sleep(.3)
                command('monitor stop')
                (root/'exit-settings.txt').write_text(command('settings'))
        finally:
            stop.set()
            if worker.ident:
                worker.join(1)
            cli.close()
            mon.close()
            (root/'comparison.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
