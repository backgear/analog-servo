#!/usr/bin/env python3
"""Build real Control.c + an external simmy checkout, save frames, and plot.

All output is SIMULATED, not bench-validated. Run --help for parameters.
No Python/C bindings needed: one native subprocess emits a JSON frame list.
"""
import argparse
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
    'baseline': (32, 1024, 1),
    'optimized': (256, 3072, 32),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simmy-dir', type=Path,
                    default=Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser())
    ap.add_argument('--config', type=Path, help='plant JSON (default: bundled representative two-mass model)')
    ap.add_argument('--out-dir', type=Path, default=ROOT / '.cache' / 'simmy')
    ap.add_argument('--ticks', type=int, default=2000, help='1 kHz ticks (default: 2000)')
    ap.add_argument('--rate', type=float, default=2500, help='signed pulse rate, counts/s')
    ap.add_argument('--ramp-seconds', type=float, default=1)
    ap.add_argument('--jitter', type=float, default=0.3)
    ap.add_argument('--tune', choices=TUNES, default='optimized',
                    help='simulation-only gains for the bundled plant (default: optimized)')
    ap.add_argument('--pos-kp', type=int, help='override preset: raw Q8.8 integer')
    ap.add_argument('--vel-kp', type=int, help='override preset: raw Q8.8 integer')
    ap.add_argument('--vel-ki', type=int, help='override preset: raw Q8.8 integer, per tick')
    ap.add_argument('--filter-shift', type=int, choices=range(5), default=0)
    ap.add_argument('--no-plot', action='store_true', help='only build/run/save JSON; no matplotlib needed')
    args = ap.parse_args()
    for name, value in zip(('pos_kp', 'vel_kp', 'vel_ki'), TUNES[args.tune]):
        if getattr(args, name) is None:
            setattr(args, name, value)
    if not 2 <= args.ticks <= 1_000_000:
        ap.error('--ticks must be between 2 and 1000000')
    if not all(math.isfinite(v) for v in (args.rate, args.ramp_seconds, args.jitter)):
        ap.error('rate, ramp, and jitter must be finite')
    if args.ramp_seconds < 0 or not 0 <= args.jitter <= 1:
        ap.error('ramp must be nonnegative; jitter must be between 0 and 1')
    if abs(args.rate) * args.ticks / 1000 * (1 + args.jitter) > 2**30:
        ap.error('requested travel exceeds safe int32 position range')
    if any(not 0 <= v <= 32767 for v in (args.pos_kp, args.vel_kp, args.vel_ki)):
        ap.error('raw Q8.8 gains must be between 0 and 32767')

    simmy = args.simmy_dir.expanduser().resolve()
    config = (args.config or ROOT / 'tools' / 'simmy' / 'two_mass.json').expanduser().resolve()
    sources = [ROOT / 'firmware/test/support/simmy_plot_main.c',
               ROOT / 'firmware/src/Control.c']
    sources += [simmy / 'src' / name for name in ('simmy.c', 'simmy_config.c', 'simmy_json.c')]
    for path in [simmy / 'include/simmy.h', config, *sources]:
        if not path.is_file():
            ap.error(f'missing {path}; set --simmy-dir or SIMMY_DIR to a simmy checkout')
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Rebuild from current sources every time; never reuse simmy's potentially
    # stale or cross-compiled build/libsimmy.a.
    with tempfile.TemporaryDirectory(prefix='servo-simmy-') as tmp:
        binary = Path(tmp) / 'simmy_plot'
        cmd = shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
            '-I', str(ROOT / 'firmware/include'), '-I', str(simmy / 'include'),
            *map(str, sources), '-lm', '-o', str(binary)]
        subprocess.run(cmd, check=True)
        run = [str(binary), str(config), str(args.ticks), str(args.rate),
               str(args.ramp_seconds), str(args.jitter), str(args.pos_kp),
               str(args.vel_kp), str(args.vel_ki), str(args.filter_shift)]
        result = subprocess.run(run, check=True, capture_output=True, text=True)
    frames = json.loads(result.stdout)
    frames_path = out / 'frames.json'
    frames_path.write_text(json.dumps(frames) + '\n')
    # Preserve inputs separately: the frame list stays compatible with the
    # existing telemetry renderer, and runs remain attributable to revisions.
    def revision(repo):
        result = subprocess.run(['git', '-C', str(repo), 'describe', '--always', '--dirty'],
                                capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else 'unknown'
    metadata = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    metadata.update(simulated=True, firmware_revision=revision(ROOT),
                    simmy_revision=revision(simmy), simmy_dir=str(simmy),
                    plant=json.loads(config.read_text()), tick_hz=1000,
                    torque_clamp=[-1000, 1000])
    (out / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Saved {frames_path}', flush=True)
    if not args.no_plot:
        subprocess.run([sys.executable, str(ROOT / 'tools/plot_telemetry.py'),
                        '--frames', str(frames_path), '--out', str(out / 'telemetry.png'),
                        '--title', 'Control.c + simmy — SIMULATED, not a bench capture'], check=True)


if __name__ == '__main__':
    main()
