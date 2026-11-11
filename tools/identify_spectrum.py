#!/usr/bin/env python3
"""Estimate open-loop torque-to-motion FRFs from decoded monitor JSON.

Not yet bench-validated with excitation. Never interpolates across missing ticks.
Do not use this estimator for closed-loop identification: an independently
recorded injection reference is required to avoid feedback/noise bias.
Requires numpy and scipy; input format is capture_moves.py's list of frames.
"""
import argparse
import json


def contiguous_runs(frames):
    """Split at missing, duplicated, or out-of-order uint32 tick counters."""
    runs = []
    for frame in frames:
        if not runs or ((frame['seq'] - runs[-1][-1]['seq']) & 0xffffffff) != 1:
            runs.append([])
        runs[-1].append(frame)
    return runs


def estimate(frames, tick_hz=1000, nperseg=1024):
    import numpy as np
    from scipy.signal import csd, welch

    if tick_hz <= 0 or nperseg < 8:
        raise ValueError('positive tick rate and segment length >= 8 required')
    spectra = []
    for run in contiguous_runs(frames):
        for start in range(0, len(run) - nperseg + 1, nperseg // 2):
            segment = run[start:start + nperseg]
            if any(f['fault_latched'] for f in segment):
                continue
            u = np.array([f['torque'] for f in segment], dtype=float)
            y = np.array([f['velocity'] for f in segment], dtype=float) * tick_hz
            if np.ptp(u) == 0:
                continue
            kw = dict(fs=tick_hz, nperseg=nperseg, noverlap=0, detrend='constant')
            freq, suu = welch(u, **kw)
            _, syy = welch(y, **kw)
            _, suy = csd(u, y, **kw)  # scipy: conjugate(U) * Y
            spectra.append((suu, syy, suy))
    if len(spectra) < 2:
        raise ValueError('need at least two excited, contiguous, fault-free windows')
    suu, syy, suy = np.mean(np.array(spectra), axis=0)
    valid = (suu.real > 0) & (syy.real > 0) & (freq > 0)
    h = np.full(suy.shape, complex(float('nan'), 0))
    coherence = np.full(freq.shape, float('nan'))
    h[valid] = suy[valid] / suu[valid]
    coherence[valid] = np.clip(abs(suy[valid])**2 / (suu[valid].real * syy[valid].real), 0, 1)
    return freq, h, coherence, len(spectra)


def main():
    import csv
    import numpy as np
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('frames')
    ap.add_argument('--out', required=True)
    ap.add_argument('--tick-hz', type=float, default=1000)
    ap.add_argument('--nperseg', type=int, default=1024)
    ap.add_argument('--open-loop', action='store_true', required=True,
                    help='acknowledge that this estimator is only for open-loop data')
    args = ap.parse_args()
    with open(args.frames) as source:
        frames = json.load(source)
    freq, h, coherence, windows = estimate(frames, args.tick_hz, args.nperseg)
    with open(args.out, 'w') as target:
        writer = csv.writer(target)
        writer.writerow(['frequency_hz', 'gain_counts_per_second_per_torque_unit', 'phase_deg', 'coherence'])
        writer.writerows(zip(freq, abs(h), np.angle(h, deg=True), coherence))
    print(f'{windows} windows; {len(contiguous_runs(frames))} contiguous runs')


if __name__ == '__main__':
    main()
