#!/usr/bin/env python3
"""Estimate operational trim from spatially matched opposite-direction motion.

Assumes direction-odd friction cancels and full-turn angle-dependent effort has
zero mean. This is NOT a direct measurement of electrical zero/current.
"""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np


def matched_pair(positive, negative, trim, cpr=4000, bin_counts=100):
    if cpr <= 0 or bin_counts <= 0 or cpr % bin_counts:
        raise ValueError('CPR must be a positive multiple of spatial bin width')
    lo = max(min(r['position'] for r in positive), min(r['position'] for r in negative))
    hi = min(max(r['position'] for r in positive), max(r['position'] for r in negative))
    start = int(np.ceil(lo/bin_counts)*bin_counts)
    turns = int((hi-start)//cpr)
    if turns < 1:
        raise ValueError('need at least one full revolution of common steady positions')
    edges = np.arange(start, start+turns*cpr+1, bin_counts)
    means, counts = [], []
    for rows in [positive, negative]:
        pos = np.array([r['position'] for r in rows])
        torque = np.array([r['torque'] for r in rows], dtype=float)
        chosen = (pos >= edges[0]) & (pos < edges[-1])
        n, _ = np.histogram(pos[chosen], bins=edges)
        total, _ = np.histogram(pos[chosen], bins=edges, weights=torque[chosen])
        if np.any(n == 0):
            raise ValueError('empty spatial bin')
        means.append(total/n)
        counts.append(n)
    even = (means[0]+means[1])/2
    odd = (means[0]-means[1])/2
    phase = edges[:-1] % cpr
    folded = [float(np.mean(even[phase == p])) for p in range(0, cpr, bin_counts)]
    return dict(trim=trim, common_start=start, common_end=int(edges[-1]), turns=turns,
                cpr=cpr, bin_counts=bin_counts,
                positive_mean=float(np.mean(means[0])), negative_mean=float(np.mean(means[1])),
                controller_midpoint=float(np.mean(even)),
                estimated_trim=float(trim+np.mean(even)),
                odd_effort_mean=float(np.mean(odd)),
                spatial_even_std=float(np.std(even)),
                # Spatial variation is not uncertainty of the mean.
                phase_counts=list(range(0, cpr, bin_counts)), phase_even_controller=folded,
                spatial_bin_starts=edges[:-1].tolist(),
                positive_bins=means[0].tolist(), negative_bins=means[1].tolist(),
                positive_samples=counts[0].tolist(), negative_samples=counts[1].tolist())


def load_steady(path, expected_ticks, warmup=1000, tail=250):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as source:
        frames = json.load(source)
    if not frames or any(r['fault_latched'] for r in frames):
        raise ValueError('empty/faulted recording')
    if any(((b['seq']-a['seq']) & 0xffffffff) != 1 for a, b in zip(frames, frames[1:])):
        raise ValueError('noncontiguous recording')
    active = [r for r in frames if r['excitation']]
    if len(active) != expected_ticks or any(((b['seq']-a['seq']) & 0xffffffff) != 1 for a, b in zip(active, active[1:])):
        raise ValueError('incomplete/noncontiguous bounded interval')
    if len(active) <= warmup+tail:
        raise ValueError('no steady interval')
    return active[warmup:-tail] if tail else active[warmup:]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('manifest', help='JSON list: pair, file, trim, velocity, duration_ticks')
    ap.add_argument('--out', required=True, help='output directory')
    ap.add_argument('--cpr', type=int, default=4000)
    ap.add_argument('--bin-counts', type=int, default=100)
    args = ap.parse_args()
    manifest = Path(args.manifest)
    grouped = {}
    for run in json.loads(manifest.read_text()):
        path = manifest.parent/run['file']
        metadata = json.loads(Path(str(path)+'.meta.json').read_text())
        if (not metadata.get('completed') or metadata['velocity_command'] != run['velocity'] or
                metadata['duration_ticks'] != run['duration_ticks']):
            raise ValueError('incomplete capture or manifest mismatch: '+str(path))
        rows = load_steady(path, run['duration_ticks'])
        velocity = float(np.mean([r['velocity'] for r in rows]))
        if abs(velocity-run['velocity']) > .05 or any(abs(r['torque']) >= 2000 for r in rows):
            raise ValueError('speed not established or clamp reached: '+str(path))
        group = grouped.setdefault(run['pair'], {})
        sign = 1 if run['velocity'] > 0 else -1
        if sign in group:
            raise ValueError('duplicate direction in pair')
        group[sign] = (run, rows)
    results = []
    for name, group in grouped.items():
        pos, neg = group[1], group[-1]
        if (pos[0]['trim'] != neg[0]['trim'] or pos[0]['velocity'] != -neg[0]['velocity'] or
                pos[0].get('profile') != neg[0].get('profile')):
            raise ValueError('pair mismatch')
        result = matched_pair(pos[1], neg[1], pos[0]['trim'], args.cpr, args.bin_counts)
        result.update(pair=name, speed=pos[0]['velocity'], profile=pos[0].get('profile', 'unspecified'))
        results.append(result)
    correlation = np.corrcoef([r['phase_even_controller'] for r in results]).tolist()
    estimates = [r['estimated_trim'] for r in results]
    by_profile = {}
    for r in results:
        by_profile.setdefault(r['profile'], []).append(r['estimated_trim'])
    profile_ranges = {k: dict(mean=float(np.mean(v)), minimum=min(v), maximum=max(v)) for k, v in by_profile.items()}
    summary = dict(method='equal spatial bins, matched opposite directions, complete turns',
                   direct_current_measurement=False, pairs=results,
                   estimate_mean=float(np.mean(estimates)), estimate_min=min(estimates),
                   estimate_max=max(estimates), phase_correlations=correlation,
                   profiles=profile_ranges, recommended_trim=None,
                   warning='No unique electrical-zero calibration; pooled mean is descriptive only, not a recommended trim.')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), constrained_layout=True)
    labels = []
    for index, r in enumerate(results):
        label=f"T{r['trim']}, v{r['speed']}, {r['profile']}"
        if label in labels:
            label += ' (repeat)'
        labels.append(label)
        axes[0].scatter(index, r['estimated_trim'], color=f'C{index}')
        axes[1].plot(r['phase_counts'], np.array(r['phase_even_controller'])+r['trim'], label=label)
    axes[0].set_xticks(range(len(results)), labels, rotation=35, ha='right', fontsize=7)
    axes[0].axhline(min(r['trim'] for r in results), color='gray', linestyle='--', label='Lowest tested trim')
    axes[0].set(xlabel='Paired measurement (trim, speed, feedback configuration)',
                ylabel='Motion-inferred trim (torque units)',
                title='The midpoint does not identify one electrical zero')
    axes[1].set(xlabel=f'Encoder phase (counts modulo {args.cpr})', ylabel='Even command component, including trim',
                title='Remaining effort is strongly position-dependent')
    for axis in axes:
        axis.grid(True, alpha=.25)
        axis.legend(fontsize=7)
    fig.suptitle('Motion-based trim estimates — configuration/history dependence remains')
    fig.savefig(out/'trim-analysis.png', dpi=150)
    plt.close(fig)
    lines=['# Generated trim-motion analysis', '',
           'No unique electrical-zero calibration: estimates depend on operating configuration/history.', '',
           '| Pair | Profile | Trim | Speed +/- | Positive mean | Negative mean | Midpoint | Inferred trim |',
           '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['pair']} | {r['profile']} | {r['trim']} | {r['speed']} | {r['positive_mean']:.2f} | {r['negative_mean']:.2f} | {r['controller_midpoint']:.2f} | {r['estimated_trim']:.2f} |")
    lines += ['', f"Observed inferred range {min(estimates):.2f}..{max(estimates):.2f} torque units; no unique recommended trim.",
              '', 'The pooled mean is descriptive only and is not an electrical-zero estimate.',
              'This range describes observations, not a calibrated uncertainty interval.',
              'Spatial even-component variation is not error of the mean and cannot be removed by one constant trim.',
              'Assumptions: odd friction cancels, complete-turn position-dependent effort has zero mean, and CPR is correct.',
              'Configuration/history dependence means these assumptions are not sufficient to identify electrical zero here.',
              'A static-output drive-current measurement is needed to calibrate true zero independently of motion feedback.',
              '', '![Matched direction and position analysis](trim-analysis.png)', '']
    (out/'report.md').write_text('\n'.join(lines))
    print('\n'.join(lines[:14]))


if __name__ == '__main__':
    main()
