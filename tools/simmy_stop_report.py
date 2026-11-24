#!/usr/bin/env python3
"""Stopping investigation: sensor fidelity, ablations, and idle gains.

EXPERIMENTAL simulation only, not bench-validated. No firmware defaults change.
The idle schedule is a candidate, NOT a robust fix; validation failures stay
in the report. No controller reads load position or scenario phase.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile

from simmy_plot import ROOT, TUNES, build_driver, revision, run_driver, save_run
from simmy_report import plot_comparison, score_table

IDLE_GAINS = (64, 2048, 16, 0, 1)


def hold_table(rows):
    lines = ['| Case | Hold | Motor settle s | Load settle s | Final motor error | Final load error | Tail motor span | Tail load span | Tail torque ripple RMS |',
             '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in rows:
        for h in row['metrics']['holds']:
            def settled(body):
                v = h[body + '_settling_seconds']
                return 'Not settled' if v is None else f'{v:.3f}'
            lines.append(f"| {row['name']} | {h['phase']} | {settled('motor')} | {settled('load')}"
                         f" | {h['motor_final_error']} | {h['load_final_error']}"
                         f" | {h['motor_tail_position_span']} | {h['load_tail_position_span']}"
                         f" | {h['tail_torque_ripple_rms']:.2f} |")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simmy-dir', type=Path,
                    default=Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser())
    ap.add_argument('--config', type=Path, default=ROOT / 'tools/simmy/two_mass.json')
    ap.add_argument('--out-dir', type=Path, default=ROOT / '.cache/simmy-stopping-report')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()
    simmy, config, out = (p.expanduser().resolve() for p in
                          (args.simmy_dir, args.config, args.out_dir))
    if not config.is_file():
        ap.error(f'missing config: {config}')
    plant = json.loads(config.read_text())
    out.mkdir(parents=True, exist_ok=True)
    configs = out / 'configs'
    configs.mkdir(exist_ok=True)
    variants = {'unchanged': plant}
    for name in ('no-ripple', 'smooth', 'substeps-100'):
        p = copy.deepcopy(plant)
        if name in ('no-ripple', 'smooth'):
            for body in p['bodies']:
                body['rippleAmplitude'] = 0
        if name == 'smooth':
            for body in p['bodies']:
                body.update(stictionLimit=0, coulombLevel=0, frictionBumpExtra=0)
        if name == 'substeps-100':
            p['subSteps'] = 100
        variants[name] = p
    for name, p in variants.items():
        (configs / f'{name}.json').write_text(json.dumps(p, indent=2) + '\n')

    cases = [dict(name='balanced-model', feedback='model'),
             dict(name='balanced-encoder'),
             dict(name='deadband-only', deadband=1),
             dict(name='idle-schedule', idle_gains=IDLE_GAINS),
             dict(name='constant-low-bandwidth', gains=(64, 2048, 16, 1), deadband=1)]
    for name in ('no-ripple', 'smooth', 'substeps-100'):
        for policy in ('unchanged', 'idle'):
            cases.append(dict(name=f'{name}-{policy}', plant=name,
                              idle_gains=IDLE_GAINS if policy == 'idle' else None))
    for label, overrides in [('negative', dict(rate=-2500)), ('slow', dict(rate=1000)),
                             ('fast', dict(rate=5000)), ('sparse-pulses', dict(rate=100)),
                             ('no-jitter', dict(jitter=0)), ('long-hold', dict(ticks=20000))]:
        for policy in ('unchanged', 'idle'):
            cases.append(dict(name=f'{label}-{policy}', idle_gains=IDLE_GAINS if policy == 'idle' else None,
                              **overrides))
    rows, frames_by_name = [], {}
    with tempfile.TemporaryDirectory(prefix='simmy-stopping-') as tmp:
        try:
            binary = build_driver(tmp, simmy)
        except FileNotFoundError as exc:
            ap.error(str(exc))
        for case in cases:
            gains = case.get('gains', TUNES['balanced'])
            path = configs / (case.get('plant', 'unchanged') + '.json')
            inputs = dict(ticks=case.get('ticks', 10000), rate=case.get('rate', 2500),
                          ramp_seconds=1, jitter=case.get('jitter', 0.3), scenario='sequence',
                          feedback=case.get('feedback', 'encoder'), feedforward='none',
                          deadband=case.get('deadband', 0), idle_gains=case.get('idle_gains'),
                          idle_after_ticks=50)
            frames = run_driver(binary, path, gains, **inputs)
            parameters = dict(inputs, velocity_deadband=inputs['deadband'], pos_kp=gains[0],
                              vel_kp=gains[1], vel_ki=gains[2], filter_shift=gains[3],
                              experiment='stopping, not bench-validated')
            metrics = save_run(out / case['name'], frames, path, simmy, parameters)
            rows.append(dict(name=case['name'], metrics=metrics, parameters=parameters))
            if case['name'] in ('balanced-encoder', 'deadband-only', 'idle-schedule'):
                frames_by_name[case['name']] = frames
    plots = []
    if not args.no_plots:
        for label, start, end in [('full-sequence', 0, 10), ('forward-stop', 2.8, 5), ('final-stop', 7.8, 10)]:
            name = label + '.png'
            plot_comparison([(case, [f for f in frames if start <= f['t'] < end])
                             for case, frames in frames_by_name.items()], out / name,
                            'Stopping: unchanged / deadband only / pulse-idle gains\nEXPERIMENTAL simulation, not bench-validated')
            plots.append(name)
    manifest = dict(simulated=True, firmware_revision=revision(ROOT), simmy_revision=revision(simmy),
                    idle_gains=IDLE_GAINS, idle_after_ticks=50, feedback='encoder except balanced-model',
                    report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (out / 'scores.json').write_text(json.dumps(rows, indent=2) + '\n')
    lines = ['# Stopping investigation', '', '**EXPERIMENTAL simulation, not bench-validated.**', '',
             f"Firmware `{manifest['firmware_revision']}`; simmy `{manifest['simmy_revision']}`.", '',
             '## What is being tested', '',
             '- Existing balanced gains, model velocity vs. encoder-position differences.',
             '- Existing velocity-error deadband alone (1 count/tick), not a position deadband.',
             '- Pulse-idle gain scheduling after 50 zero-delta ticks: Q8 gains 64 / 2048 / 16, filter 0, velocity deadband 1.',
             '- Switching preserves the PI integral and plant state. Any new pulse restores moving gains before that tick.',
             '- No torque-off, position snap, load feedback, analytic trajectory or future-position look-ahead.',
             '- Low-bandwidth constant gains are included to expose the motion/hold tradeoff.', '',
             '## Entire recorded experiment', '', score_table(rows), '',
             '## Motion only', '', score_table(rows, 'moving'), '',
             '## Holds — successes AND failures', '',
             'Unchanged settling criterion: error ≤5 counts AND speed ≤1 count/tick for the remainder of',
             'the recorded hold, with at least 250 ms of evidence. Not settled is not success.',
             'Tail spans and torque ripple use the final 250 samples; ripple removes the DC mean torque.',
             'Being inside the tolerance band does not prove an absolutely motionless or quiet hold.', '',
             hold_table(rows), '', '## Interpretation and limits', '',
             '- The idle schedule is a candidate, **not a robust fix**. Read the validation rows, not just the default move.',
             '- `smooth` removes static/Coulomb friction, the bump and ripple; it does not retune the moving controller.',
             '  Poor motion or saturation there exposes dependence on plant damping/friction, even if a later hold recovers.',
             '- Pulse inactivity alone cannot distinguish a stop from very slow step input; sparse-pulse rows test that limitation.',
             '- Motor settling and load centering are separate problems. Static friction can support coupling deflection',
             '  while the encoder is on target. At rest, the default load supports roughly 80/3.16 + 0.5 ≈26 counts',
             '  of offset. Motor-only feedback cannot guarantee ±5-count load positioning in that model.',
             '  This numerical bound describes the bundled plant only, not arbitrary custom configs.',
             '- No gain schedule or new deadband is enabled by default or added to production firmware.',
             '- Hardware adoption needs a separate design review, low-speed/reversal/noise/disturbance tests and bench validation.', '',
             '## Plots', '']
    for name in plots:
        lines += [f'![{name}: simulated]({name})', '']
    if not plots:
        lines += ['Plots skipped.', '']
    (out / 'report.md').write_text('\n'.join(lines))
    print(f'Saved {len(rows)} cases to {out / "report.md"}')


if __name__ == '__main__':
    main()
