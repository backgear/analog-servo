#!/usr/bin/env python3
"""Build real Control.c + an external simmy checkout, save frames, and plot.

All output is SIMULATED, not bench-validated. Run --help for parameters.
No Python/C bindings needed: one native subprocess emits a JSON frame list.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent

# Simulation-specific presets for tools/simmy/two_mass.json, NOT hardware gains.
TUNES = {
    'baseline': (32, 1024, 1, 0),
    'optimized': (256, 3072, 32, 0),
    'balanced': (256, 1024, 16, 1),
}


def build_driver(build_dir, simmy):
    """Compile once for a batch; never reuse external prebuilt archives."""
    sources = [ROOT / 'firmware/test/support/simmy_plot_main.c',
               ROOT / 'firmware/src/Control.c']
    sources += [simmy / 'src' / name for name in ('simmy.c', 'simmy_config.c', 'simmy_json.c')]
    for path in [simmy / 'include/simmy.h', *sources]:
        if not path.is_file():
            raise FileNotFoundError(f'missing {path}; set --simmy-dir or SIMMY_DIR')
    binary = Path(build_dir) / 'simmy_plot'
    cmd = shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-I', str(ROOT / 'firmware/include'), '-I', str(simmy / 'include'),
        *map(str, sources), '-lm', '-o', str(binary)]
    subprocess.run(cmd, check=True)
    return binary


def sequence_ticks(ramp_seconds):
    return 4 * int(math.floor(ramp_seconds * 1000 + 0.5)) + 6000


def run_driver(binary, config, gains, ticks=2000, rate=2500, ramp_seconds=1, jitter=0.3,
               scenario='ramp-cruise', feedforward='none', ff_shift=3, accel_ff_q8=18432):
    cmd = [str(binary), str(config), str(ticks), str(rate), str(ramp_seconds), str(jitter),
           *map(str, gains), scenario, feedforward, str(ff_shift), str(accel_ff_q8)]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def revision(repo):
    result = subprocess.run(['git', '-C', str(repo), 'describe', '--always', '--dirty'],
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else 'unknown'


def save_run(out, frames, config, simmy, parameters):
    """Shared recorder for single runs and batch reports, with exact input hashes."""
    from simmy_metrics import measure
    out.mkdir(parents=True, exist_ok=True)
    (out / 'frames.json').write_text(json.dumps(frames) + '\n')
    metrics = measure(frames)
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    metadata = {k: str(v) if isinstance(v, Path) else v for k, v in parameters.items()}
    hashes = {}
    inputs = [(ROOT, p) for p in (ROOT / 'firmware/test/support/simmy_plot_main.c',
                                  ROOT / 'firmware/src/Control.c',
                                  ROOT / 'firmware/include/Control.h',
                                  ROOT / 'firmware/include/Output.h',
                                  ROOT / 'tools/simmy_plot.py', ROOT / 'tools/simmy_metrics.py',
                                  ROOT / 'tools/plot_telemetry.py')]
    inputs += [(simmy, p) for folder in ('src', 'include') for p in (simmy / folder).glob('*')
               if p.suffix in ('.c', '.h')]
    for repo, path in inputs:
        label = ('firmware' if repo == ROOT else 'simmy') + ':' + str(path.relative_to(repo))
        hashes[label] = hashlib.sha256(path.read_bytes()).hexdigest()
    metadata.update(simulated=True, firmware_revision=revision(ROOT),
                    simmy_revision=revision(simmy), simmy_dir=str(simmy), config=str(config),
                    plant=json.loads(config.read_text()), tick_hz=1000, source_sha256=hashes,
                    torque_clamp=[-1000, 1000], pulse_seed=12345, analog_noise_seed=1,
                    host_cc=os.environ.get('CC', 'cc'),
                    effective_accel_ff_q8=(parameters.get('accel_ff_q8', 18432)
                                           if parameters.get('feedforward') == 'velocity-acceleration' else 0))
    (out / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simmy-dir', type=Path,
                    default=Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser())
    ap.add_argument('--config', type=Path, help='plant JSON (default: bundled representative two-mass model)')
    ap.add_argument('--out-dir', type=Path, default=ROOT / '.cache' / 'simmy')
    ap.add_argument('--scenario', choices=('ramp-cruise', 'sequence'), default='ramp-cruise',
                    help='sequence: accelerate, cruise, stop, hold, reverse, stop, hold')
    ap.add_argument('--ticks', type=int, help='1 kHz ticks (default: 2000 or complete sequence)')
    ap.add_argument('--rate', type=float, default=2500, help='signed pulse rate, counts/s')
    ap.add_argument('--ramp-seconds', type=float, default=1)
    ap.add_argument('--jitter', type=float, default=0.3)
    ap.add_argument('--tune', choices=TUNES, default='optimized',
                    help='simulation-only gains for the bundled plant (default: optimized)')
    ap.add_argument('--pos-kp', type=int, help='override preset: raw Q8.8 integer')
    ap.add_argument('--vel-kp', type=int, help='override preset: raw Q8.8 integer')
    ap.add_argument('--vel-ki', type=int, help='override preset: raw Q8.8 integer, per tick')
    ap.add_argument('--filter-shift', type=int, choices=range(5), help='override preset velocity filter')
    ap.add_argument('--feedforward', choices=('none', 'velocity', 'velocity-acceleration'),
                    default='none', help='EXPERIMENTAL Mode A path; never changes firmware')
    ap.add_argument('--ff-shift', type=int, choices=range(9), default=3,
                    help='causal pulse-delta IIR shift (0 raw; default 3)')
    ap.add_argument('--accel-ff-q8', type=int, default=18432,
                    help='torque per count/tick², raw Q8.8; only velocity-acceleration mode')
    ap.add_argument('--no-plot', action='store_true', help='only build/run/save JSON; no matplotlib needed')
    args = ap.parse_args()
    for name, value in zip(('pos_kp', 'vel_kp', 'vel_ki', 'filter_shift'), TUNES[args.tune]):
        if getattr(args, name) is None:
            setattr(args, name, value)
    if not all(math.isfinite(v) for v in (args.rate, args.ramp_seconds, args.jitter)):
        ap.error('rate, ramp, and jitter must be finite')
    if not 0 <= args.ramp_seconds <= 1000 or not 0 <= args.jitter <= 1:
        ap.error('ramp must be between 0 and 1000 seconds; jitter must be between 0 and 1')
    if args.scenario == 'sequence' and args.ramp_seconds < 0.001:
        ap.error('sequence requires --ramp-seconds >= 0.001')
    if args.ticks is None:
        args.ticks = sequence_ticks(args.ramp_seconds) if args.scenario == 'sequence' else 2000
    if not 2 <= args.ticks <= 1_000_000:
        ap.error('--ticks must be between 2 and 1000000')
    if abs(args.rate) * args.ticks / 1000 * (1 + args.jitter) > 2**30:
        ap.error('requested travel exceeds safe int32 position range')
    if any(not 0 <= v <= 32767 for v in (args.pos_kp, args.vel_kp, args.vel_ki)):
        ap.error('raw Q8.8 gains must be between 0 and 32767')

    if not 0 <= args.accel_ff_q8 <= 32767:
        ap.error('--accel-ff-q8 must be between 0 and 32767')
    simmy = args.simmy_dir.expanduser().resolve()
    config = (args.config or ROOT / 'tools' / 'simmy' / 'two_mass.json').expanduser().resolve()
    if not config.is_file():
        ap.error(f'missing plant config: {config}')
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='servo-simmy-') as tmp:
        try:
            binary = build_driver(tmp, simmy)
        except FileNotFoundError as exc:
            ap.error(str(exc))
        frames = run_driver(binary, config,
                            (args.pos_kp, args.vel_kp, args.vel_ki, args.filter_shift),
                            args.ticks, args.rate, args.ramp_seconds, args.jitter, args.scenario,
                            args.feedforward, args.ff_shift, args.accel_ff_q8)
    frames_path = out / 'frames.json'
    save_run(out, frames, config, simmy, vars(args))
    print(f'Saved {frames_path}', flush=True)
    if not args.no_plot:
        subprocess.run([sys.executable, str(ROOT / 'tools/plot_telemetry.py'),
                        '--frames', str(frames_path), '--out', str(out / 'telemetry.png'),
                        '--title', ('Control.c + simmy — SIMULATED, not a bench capture'
                                    + (f'\nEXPERIMENTAL {args.feedforward} feedforward'
                                       if args.feedforward != 'none' else ''))], check=True)


if __name__ == '__main__':
    main()
