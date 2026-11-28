#!/usr/bin/env python3
"""Pool completed identical excitation bursts without joining across tick gaps.

Accepts JSON or gzip JSON captures plus their .meta.json sidecars. Pooling
improves averaging; it does not make the short bursts a continuous long run.
"""
import argparse
import gzip
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('captures', nargs='+')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    frames, metadata = [], None
    keys = ('amplitude', 'hold_ticks', 'duration_ticks', 'velocity_command',
            'temporary_torque_limit', 'direct_torque', 'travel_bound', 'speed_bound')
    for path in args.captures:
        meta = json.loads(Path(path + '.meta.json').read_text())
        if not meta.get('completed'):
            raise ValueError('incomplete burst: ' + path)
        opener = gzip.open if path.endswith('.gz') else open
        with opener(path, 'rt') as source:
            capture = json.load(source)
        excited = [r for r in capture if r.get('excitation')]
        if len(excited) != meta['duration_ticks'] or any(r['fault_latched'] for r in excited):
            raise ValueError('missing/faulted excitation samples: ' + path)
        if any(((b['seq'] - a['seq']) & 0xffffffff) != 1 for a, b in zip(excited, excited[1:])):
            raise ValueError('noncontiguous burst: ' + path)
        if metadata is not None and any(meta.get(k) != metadata.get(k) for k in keys):
            raise ValueError('different burst parameters: ' + path)
        if frames and not 1 < ((excited[0]['seq'] - frames[-1]['seq']) & 0xffffffff) < 0x80000000:
            raise ValueError('overlapping/adjacent burst counters; cannot pool safely')
        # Store only marked samples, preserving actual tick gaps between bursts.
        frames.extend(excited)
        metadata = meta
    metadata = metadata | dict(pooled_bursts=len(args.captures), sources=args.captures)
    opener = gzip.open if args.out.endswith('.gz') else open
    with opener(args.out, 'wt') as target:
        json.dump(frames, target)
    Path(args.out + '.meta.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'{len(frames)} marked samples pooled from {len(args.captures)} bursts')


if __name__ == '__main__':
    main()
