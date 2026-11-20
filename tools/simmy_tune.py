#!/usr/bin/env python3
"""Repeatable balanced-gain grid search; SIMULATED, not bench-validated.

Builds once, scores unchanged two-second ramp/cruise runs, and saves every
candidate. This is a local search, not a claim of global optimality.
"""
import argparse
import itertools
import json
import os
from pathlib import Path
import tempfile

from simmy_metrics import balanced_score, measure
from simmy_plot import ROOT, build_driver, run_driver


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simmy-dir', type=Path,
                    default=Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser())
    ap.add_argument('--out', type=Path, default=ROOT / '.cache/simmy-balanced-search.json')
    args = ap.parse_args()
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        binary = build_driver(tmp, args.simmy_dir.expanduser().resolve())
        for gains in itertools.product((192, 256, 320), (512, 768, 1024, 1280),
                                       (8, 16, 24, 32), (0, 1, 2)):
            metrics = measure(run_driver(binary, ROOT / 'tools/simmy/two_mass.json', gains))
            results.append(dict(gains=gains, score=balanced_score(metrics), **metrics))
    results.sort(key=lambda item: item['score'])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results[:5], indent=2))
    print(f'Saved {len(results)} candidates to {args.out}')


if __name__ == '__main__':
    main()
