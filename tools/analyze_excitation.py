#!/usr/bin/env python3
"""Compare reference-based closed-loop FRFs; never recommends gains automatically.

Requires numpy/scipy/matplotlib. Use only updated decoder excitation captures;
--polarity converts recorded physical command into encoder-axis torque.
"""
import argparse
import csv
import gzip
import json
from pathlib import Path
import numpy as np
from identify_spectrum import estimate_closed_loop, contiguous_runs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('captures', nargs='+')
    ap.add_argument('--out', required=True, help='output filename prefix')
    ap.add_argument('--nperseg', type=int, default=512)
    ap.add_argument('--polarity', type=int, choices=[-1, 1], required=True)
    args = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5, 1, figsize=(11, 16), constrained_layout=True)
    summaries = []
    for index, path in enumerate(args.captures):
        opener = gzip.open if path.endswith('.gz') else open
        with opener(path, 'rt') as source:
            frames = json.load(source)
        for frame in frames:
            frame['torque'] *= args.polarity
        result = estimate_closed_loop(frames, nperseg=args.nperseg)
        e = [f for f in frames if f.get('excitation')]
        amp = max(abs(f['injected_torque']) for f in e)
        meta_path = Path(path + '.meta.json')
        metadata = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        complete = metadata.get('completed', False)
        runs = contiguous_runs(e)
        label = f'+/-{amp}, hold {metadata.get("hold_ticks", "?")} ticks ({len(runs)} bursts)'
        if not complete:
            label += ' [incomplete/unverified]'
        f, h = result['frequency'], result['response']
        ci, cy = result['coherence_input'], result['coherence_motion']
        good = (ci >= .8) & (cy >= .8) & (f > 0) & (f <= 250)
        if not complete or result['windows'] < 8:
            good[:] = False
        color = f'C{index}'
        for burst, run in enumerate(runs):
            t = np.array([((r['seq'] - run[0]['seq']) & 0xffffffff) / 1000 for r in run])
            position = np.array([r['position'] - run[0]['position'] for r in run])
            axes[0].plot(t, position, color=color, label=label if burst == 0 else None, alpha=.6)
        axes[1].semilogy(f[1:], result['injection_psd'][1:], color=color, label=label)
        axes[2].semilogx(f[1:], abs(h[1:]), color=color, alpha=.3)
        axes[2].semilogx(f[good], abs(h[good]), '.', color=color, label=label)
        axes[3].semilogx(f[1:], np.angle(h[1:], deg=True), color=color, alpha=.3)
        axes[3].semilogx(f[good], np.angle(h[good], deg=True), '.', color=color)
        axes[4].semilogx(f[1:], cy[1:], color=color, label=label + ' motion')
        axes[4].semilogx(f[1:], ci[1:], '--', color=color, label=label + ' torque')
        prefix = Path(args.out)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        with open(str(prefix) + f'-{index}.csv', 'w') as target:
            writer = csv.writer(target)
            writer.writerow(['frequency_hz', 'velocity_gain_counts_per_s_per_torque',
                             'phase_deg', 'reference_torque_coherence', 'reference_motion_coherence', 'injection_psd'])
            writer.writerows(zip(f, abs(h), np.angle(h, deg=True), ci, cy, result['injection_psd']))
        summary = dict(capture=str(path), amplitude=amp, metadata=metadata,
                       frames=result['excited_frames'], runs=result['runs'], windows=result['windows'],
                       position_range=[min(r['position'] for r in e), max(r['position'] for r in e)],
                       burst_position_spans=[max(r['position'] for r in run) - min(r['position'] for r in run) for run in runs],
                       velocity_range=[min(r['velocity'] for r in e), max(r['velocity'] for r in e)],
                       torque_range=[min(r['torque'] for r in e), max(r['torque'] for r in e)],
                       high_coherence_bins_below_150_hz=f[good & (f <= 150)].tolist(),
                       median_motion_coherence_5_to_100=float(np.nanmedian(cy[(f >= 5) & (f <= 100)])))
        summaries.append(summary)
    for axis, title, ylabel in zip(axes,
            ['Encoder motion (bursts overlaid; each starts at zero)', 'Injected signal spectrum',
             'Estimated torque-to-velocity gain (dots: both coherences >=0.8)',
             'Estimated phase (faint traces are not necessarily trustworthy)',
             'Reference coherence: solid motion, dashed total torque'],
            ['Relative position (counts)', 'Torque units²/Hz', 'counts/s per torque unit', 'Degrees', 'Coherence']):
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=.25)
    axes[0].set_xlabel('Time (s)')
    axes[0].legend(fontsize=8)
    axes[2].set_yscale('log')
    axes[1].set_xlim(0, 250)
    axes[1].set_xlabel('Frequency (Hz)')
    for axis in axes[2:]:
        axis.set_xlim(2, 250)
        axis.set_xlabel('Frequency (Hz)')
    axes[4].set_ylim(0, 1.05)
    axes[4].axhline(.8, color='gray', linestyle=':')
    axes[4].legend(fontsize=7)
    fig.savefig(args.out + '.png', dpi=140)
    Path(args.out + '.json').write_text(json.dumps(summaries, indent=2) + '\n')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
