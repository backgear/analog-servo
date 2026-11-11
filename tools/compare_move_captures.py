#!/usr/bin/env python3
"""Compare capture_moves.py recordings using firmware time, not USB arrival time.

Plots align each position step / velocity ramp at its own command onset. This
is an offline comparison, not a repeated-trial significance test.
"""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np


def step_metrics(active, tick_hz=1000):
    if len(active) < 2 or any(((b['seq']-a['seq']) & 0xffffffff) != 1 for a, b in zip(active, active[1:])):
        raise ValueError('need a contiguous position-step segment')
    target = active[0]['cmd_position']
    if any(r['cmd_position'] != target for r in active):
        raise ValueError('step target is not constant')
    start = active[0]['position']
    if target == start:
        raise ValueError('zero-distance step')
    direction = 1 if target > start else -1
    error = np.array([target-r['position'] for r in active])
    progress = np.array([(r['position']-start)/(target-start) for r in active])
    t = np.array([((r['seq']-active[0]['seq']) & 0xffffffff)*1000/tick_hz for r in active])
    i10, i90 = np.flatnonzero(progress >= .1), np.flatnonzero(progress >= .9)
    outside = np.flatnonzero(abs(error) > 5)
    entry = int(outside[-1]+1) if len(outside) else 0
    settled = entry < len(active) and len(active)-entry >= tick_hz*.05
    return dict(rise_10_to_90_ms=float(t[i90[0]]-t[i10[0]]) if len(i10) and len(i90) else None,
                reach_90_ms=float(t[i90[0]]) if len(i90) else None,
                settle_5_counts_ms=float(t[entry]) if settled else None,
                overshoot_counts=int(max(0, max(-direction*error))),
                final_mean_error_counts=float(np.mean(error[-max(1, tick_hz//10):])),
                peak_speed_counts_per_tick=max(abs(r['velocity']) for r in active),
                peak_torque=max(abs(r['torque']) for r in active))


def ramp_segment(frames):
    """Select the main forward ramp, excluding small later hold retargets."""
    runs = []
    for frame in frames:
        if frame['synthetic'] or frame['cmd_velocity'] <= 0:
            continue
        if not runs or ((frame['seq']-runs[-1][-1]['seq']) & 0xffffffff) != 1:
            runs.append([])
        runs[-1].append(frame)
    if not runs:
        raise ValueError('no forward velocity ramp')
    return max(runs, key=lambda run: (max(r['cmd_velocity'] for r in run), len(run)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('captures', nargs='+')
    ap.add_argument('--labels', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--vary', choices=['torque-limit', 'velocity-filter'], default='torque-limit')
    args = ap.parse_args()
    if len(args.labels) != len(args.captures):
        ap.error('one label per capture required')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex='col', constrained_layout=True)
    summaries = []
    first_settings, first_recipe = None, None
    for index, (path, label) in enumerate(zip(args.captures, args.labels)):
        opener = gzip.open if path.endswith('.gz') else open
        with opener(path, 'rt') as source:
            frames = json.load(source)
        if not frames or any(r['fault_latched'] for r in frames):
            raise ValueError('empty/faulted capture: '+path)
        metadata = json.loads(Path(path+'.meta.json').read_text())
        if not metadata.get('completed'):
            raise ValueError('incomplete capture: '+path)
        settings = dict(line.split('=', 1) for line in metadata['entry_settings'].splitlines() if '=' in line)
        recipe_keys = ('step_amplitude', 'step_half_period_ticks', 'velocity_peak', 'velocity_accel')
        setting_keys = ('position.kp', 'velocity.kp', 'velocity.ki', 'velocity.filter_shift',
                        'velocity.deadband', 'velocity.limit', 'output.trim', 'trajectory.velocity_limit',
                        'torque.min', 'torque.max', 'torque.accel_feedforward', 'trajectory.move_feedforward',
                        'trajectory.jerk_limit', 'trajectory.accel_limit')
        allowed = {'torque.min', 'torque.max'} if args.vary == 'torque-limit' else {'velocity.filter_shift'}
        if first_settings is not None and (any(settings[k] != first_settings[k] for k in setting_keys if k not in allowed) or
                any(metadata['parameters'][k] != first_recipe[k] for k in recipe_keys)):
            raise ValueError('gain/filter/recipe mismatch outside selected comparison variable')
        first_settings, first_recipe = settings, metadata['parameters']
        active = [r for r in frames if r['synthetic']]
        ramp = ramp_segment(frames)
        metrics = step_metrics(active)
        low, high = int(settings['torque.min']), int(settings['torque.max'])
        metrics.update(label=label, source=path, frames=len(frames),
                       torque_limits=[low, high], velocity_filter_shift=int(settings['velocity.filter_shift']),
                       step_saturated_ticks=sum(r['torque'] <= low or r['torque'] >= high for r in active),
                       missing_tick_gaps=sum(((b['seq']-a['seq']) & 0xffffffff) != 1 for a, b in zip(frames, frames[1:])))
        for col, segment in enumerate([active, ramp]):
            origin = segment[0]['seq']
            def delta(row):
                n = (row['seq']-origin) & 0xffffffff
                return (n if n < 0x80000000 else n-0x100000000)/1000
            duration = ((segment[-1]['seq']-origin) & 0xffffffff)/1000
            selected = [r for r in frames if -.15 <= delta(r) <= duration+.4]
            time = [delta(r) for r in selected]
            baseline = selected[0]['position']
            color = f'C{index}'
            for row, key, reference in [(0, 'position', 'cmd_position'), (1, 'velocity', 'cmd_velocity'), (2, 'torque', None)]:
                offset = baseline if row == 0 else 0
                axes[row, col].plot(time, [r[key]-offset for r in selected], color=color, label=label, linewidth=1.2)
                if reference:
                    axes[row, col].plot(time, [r[reference]-offset for r in selected], color=color, alpha=.45, linestyle='--', linewidth=1)
            if col == 1:
                error = [r['cmd_position']-r['position'] for r in selected]
                metrics['ramp_peak_following_error_counts'] = max(abs(e) for e in error)
                metrics['ramp_peak_speed_counts_per_tick'] = max(abs(r['velocity']) for r in selected)
        summaries.append(metrics)
    for col, title in enumerate(['Position step: actual (solid), reference (dashed)', 'Velocity ramp: actual (solid), reference (dashed)']):
        axes[0, col].set_title(title)
        axes[2, col].set_xlabel('Time relative to command onset (s)')
        for row, ylabel in enumerate(['Relative position (counts)', 'Velocity (counts/tick)', 'Torque command']):
            axes[row, col].set_ylabel(ylabel)
            axes[row, col].grid(True, alpha=.25)
            axes[row, col].legend(fontsize=8)
    change = 'torque clamp' if args.vary == 'torque-limit' else 'P-feedback averaging'
    fig.suptitle(f'Same gains and soft band; only {change} changes — single capture pair')
    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(prefix.with_suffix('.png'), dpi=140)
    plt.close(fig)
    prefix.with_suffix('.json').write_text(json.dumps(summaries, indent=2)+'\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
