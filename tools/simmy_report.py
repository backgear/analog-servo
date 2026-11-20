#!/usr/bin/env python3
"""Generate a nine-case tune/feedforward report. SIMULATED, not bench-validated.

Build once; run baseline, optimized and balanced gains with no feedforward,
causal velocity feedforward, and velocity + acceleration feedforward.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from simmy_plot import ROOT, TUNES, build_driver, revision, run_driver, save_run, sequence_ticks

FEEDFORWARDS = ('none', 'velocity', 'velocity-acceleration')


def plot_comparison(cases, out, title):
    """Each column is an independent experiment; preserve original data files."""
    frames = []
    windows = []
    offset = 0.0
    for label, data in cases:
        first = data[0]['t']
        duration = data[-1]['t'] - first
        frames.extend(dict(f, t=f['t'] - first + offset) for f in data)
        windows += ['--window', f'{label}:{offset:.6f}:{offset + duration:.6f}']
        offset += duration + 1.0
    frames_path = out.with_suffix('.json')
    frames_path.write_text(json.dumps(frames) + '\n')
    subprocess.run([sys.executable, str(ROOT / 'tools/plot_telemetry.py'),
                    '--frames', str(frames_path), '--out', str(out),
                    '--shared-scales', '--title', title, *windows], check=True)


def score_table(rows, scope=None):
    lines = ['| Case | Motor RMS | Motor peak | Load RMS | Load peak | Torque RMS | Δtorque RMS | Clamp ticks |',
             '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in rows:
        m = row['metrics'][scope] if scope else row['metrics']
        if m is None:
            continue
        values = [f"{m[key]:.2f}" for key in ('motor_rms', 'motor_peak', 'load_rms', 'load_peak',
                                              'torque_rms', 'torque_delta_rms')]
        lines.append('| ' + row['name'] + ' | ' + ' | '.join(values)
                     + f" | {m['saturation_ticks']} |")
    return '\n'.join(lines)


def write_report(out, rows, manifest, plots):
    lines = ['# simmy control comparison', '', '**SIMULATED — not bench-validated.**', '',
             'Same plant, pulse seed, jitter and move sequence in every case. No firmware defaults changed.', '',
             f"- Firmware: `{manifest['firmware_revision']}`",
             f"- simmy: `{manifest['simmy_revision']}`",
             f"- Scenario: `{manifest['scenario']}`; {manifest['ticks']} ticks at 1 kHz; "
             f"rate {manifest['rate']} counts/s; ramp {manifest['ramp_seconds']} s; jitter {manifest['jitter']}",
             '- Each run directory contains `frames.json`, `metrics.json`, and `run.json` with gains, plant and source hashes.',
             '- Error units: counts. Torque units: raw signed command, clamped to ±1000.',
             '- Δtorque RMS is the RMS of adjacent per-tick command changes, not energy or physical jerk.', '',
             '## Presets', '', '| Tune | Position P Q8 | Velocity P Q8 | Velocity I Q8 | Filter shift |',
             '| --- | ---: | ---: | ---: | ---: |']
    for name, gains in TUNES.items():
        lines.append('| ' + name + ' | ' + ' | '.join(map(str, gains)) + ' |')
    lines += ['', '## Entire recorded experiment (motion + holds)', '', score_table(rows), '',
              '## Motion only (holds excluded)', '', score_table(rows, 'moving'), '',
              'Torque differences across excluded hold gaps are not counted as one-tick changes.', '',
              '## Stops and holds', '',
              'Settling requires absolute error ≤5 counts AND speed ≤1 count/tick for the rest of the hold,',
              'with at least 250 ms of evidence. **Not settled** means this recorded hold did not establish settling.',
              'A body can settle while the other does not; a low whole-run RMS is not proof of a quiet stop.', '',
              '| Case | Hold | Motor settle (s) | Load settle (s) | Final motor error | Final load error | Hold torque RMS | Hold Δtorque RMS |',
              '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in rows:
        for hold in row['metrics']['holds']:
            def settling(key):
                value = hold[key + '_settling_seconds']
                return 'Not settled' if value is None else f'{value:.3f}'
            lines.append(f"| {row['name']} | {hold['phase']} | {settling('motor')} | {settling('load')}"
                         f" | {hold['motor_final_error']} | {hold['load_final_error']}"
                         f" | {hold['torque_rms']:.2f} | {hold['torque_delta_rms']:.2f} |")
    if not any(row['metrics']['holds'] for row in rows):
        lines += ['', 'No holds recorded in this scenario; settling is not evaluated.']
    lines += ['', '## Interpretation and limits', '',
              '- `optimized` prioritizes moving tracking. `balanced` trades precision for lower torque activity.',
              '- Feedforward cases are **experimental Mode A paths**, not production behavior.',
              f"- Velocity reference: causal pulse-delta IIR, shift {manifest['ff_shift']}; no future or plant-state input.",
              f"- Acceleration case alone adds raw Q8 gain {manifest['accel_ff_q8']} through Control.c's existing causal path.",
              '- Acceleration gain is exploratory, not optimized; feedforward need not improve every metric.',
              '- A hold followed by reverse is tested; rapid reversal, parameter uncertainty and hardware stability are not.',
              '- Plot traces are not smoothed. Gray hold shading and phase boundaries indicate the input sequence.',
              '- Comparison columns use identical y scales, including right-axis following error.', '', '## Plots', '']
    if plots:
        for label, name in plots:
            lines += [f'### {label}', '', f'![{label}: simulated]({name})', '']
    else:
        lines += ['Plots skipped with `--no-plots`.', '']
    (out / 'report.md').write_text('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simmy-dir', type=Path,
                    default=Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser())
    ap.add_argument('--config', type=Path, default=ROOT / 'tools/simmy/two_mass.json')
    ap.add_argument('--out-dir', type=Path, default=ROOT / '.cache/simmy-report')
    ap.add_argument('--scenario', choices=('sequence', 'ramp-cruise'), default='sequence')
    ap.add_argument('--rate', type=float, default=2500)
    ap.add_argument('--ramp-seconds', type=float, default=1)
    ap.add_argument('--jitter', type=float, default=0.3)
    ap.add_argument('--ff-shift', type=int, choices=range(9), default=3)
    ap.add_argument('--accel-ff-q8', type=int, default=18432)
    ap.add_argument('--no-plots', action='store_true', help='data/report only; no matplotlib required')
    args = ap.parse_args()
    if not all(math.isfinite(v) for v in (args.rate, args.ramp_seconds, args.jitter)):
        ap.error('rate, ramp and jitter must be finite')
    if not 0.001 <= args.ramp_seconds <= 248 or not 0 <= args.jitter <= 1:
        ap.error('ramp must be 0.001..248 s; jitter must be 0..1')
    if not 0 <= args.accel_ff_q8 <= 32767:
        ap.error('accel-ff-q8 must be 0..32767')
    ticks = sequence_ticks(args.ramp_seconds) if args.scenario == 'sequence' else 2000
    if abs(args.rate) * ticks / 1000 * (1 + args.jitter) > 2**30:
        ap.error('requested travel exceeds safe int32 position range')
    simmy, config, out = (p.expanduser().resolve() for p in
                          (args.simmy_dir, args.config, args.out_dir))
    if not config.is_file():
        ap.error(f'missing plant config: {config}')
    out.mkdir(parents=True, exist_ok=True)
    manifest = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    manifest.update(ticks=ticks, simulated=True, firmware_revision=revision(ROOT),
                    simmy_revision=revision(simmy),
                    report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    rows, data = [], {}
    with tempfile.TemporaryDirectory(prefix='simmy-report-') as tmp:
        try:
            binary = build_driver(tmp, simmy)
        except FileNotFoundError as exc:
            ap.error(str(exc))
        for tune, gains in TUNES.items():
            for ff in FEEDFORWARDS:
                name = f'{tune}-{ff}'
                frames = run_driver(binary, config, gains, ticks, args.rate, args.ramp_seconds,
                                    args.jitter, args.scenario, ff, args.ff_shift, args.accel_ff_q8)
                parameters = dict(manifest, tune=tune, feedforward=ff, pos_kp=gains[0],
                                  vel_kp=gains[1], vel_ki=gains[2], filter_shift=gains[3])
                metrics = save_run(out / name, frames, config, simmy, parameters)
                rows.append(dict(name=name, tune=tune, feedforward=ff, metrics=metrics))
                data[tune, ff] = frames
    plots = []
    if not args.no_plots:
        for ff in FEEDFORWARDS:
            name = f'tunes-{ff}.png'
            label = (f'Tune comparison — feedforward: {ff}'
                     + (' (EXPERIMENTAL)' if ff != 'none' else ''))
            plot_comparison([(tune, data[tune, ff]) for tune in TUNES], out / name,
                            f'{label}\nSIMULATED, not bench-validated')
            plots.append((label, name))
        name = 'balanced-feedforward.png'
        plot_comparison([(ff, data['balanced', ff]) for ff in FEEDFORWARDS], out / name,
                        'Balanced gains — EXPERIMENTAL feedforward comparison\nSIMULATED, not bench-validated')
        plots.append(('Balanced gains: no / velocity / velocity+acceleration feedforward', name))
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (out / 'scores.json').write_text(json.dumps(rows, indent=2) + '\n')
    write_report(out, rows, manifest, plots)
    print(f'Saved {out / "report.md"}')


if __name__ == '__main__':
    main()
