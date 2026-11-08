#!/usr/bin/env python3
"""Confirm a specific (pos_p, vel_ki) point with N repeated real moves.

A single sample isn't trustworthy here: real trial-to-trial noise at a given point can be
large enough to make one move look nothing like the point's typical behavior. This runs many
more trials and reports the full spread (mean/min/max), not just an average, so a bimodal or
occasionally-bad result is visible instead of hidden.
"""
import argparse
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy');os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import time
from datetime import datetime, timezone
from pathlib import Path

from scorecard_core import Criteria, Recorder
from scorecard_session import LiveSession
from psweep_tune import wait_for_gain, run_one_move


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port');p.add_argument('cli_port')
    p.add_argument('--baud', type=int, default=2500000);p.add_argument('--cli-baud', type=int, default=115200)
    p.add_argument('--cpr', type=int, default=4096, help='Decoded quadrature counts per revolution (4x encoder cycles)');p.add_argument('--tick-hz', type=int, default=1000)
    p.add_argument('--distance', type=int, default=512)
    p.add_argument('--max-command', type=int, default=40)
    p.add_argument('--move-settle-threshold', type=int, default=1, help='counts/tick |mean100ms_speed| must be under before a move will start -- raise this on a drive with persistent ambient drift that never settles near zero, or every move request gets refused client-side')
    p.add_argument('--record-dir', default='captures')
    p.add_argument('--pos-p', type=int, required=True)
    p.add_argument('--vel-ki', type=int, required=True)
    p.add_argument('--trials', type=int, default=10)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    criteria = Criteria(cpr=args.cpr, tick_hz=args.tick_hz)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    recorder = Recorder(Path(args.record_dir) / stamp)
    recorder.event('session', tool='confirm_point', monitor_port=args.port, cli_port=args.cli_port, pos_p=args.pos_p, vel_ki=args.vel_ki)
    session = LiveSession(args.port, args.cli_port, recorder, criteria, args.baud, args.cli_baud, args.max_command,
                           move_settle_threshold=args.move_settle_threshold)
    try:
        session.start()
    except Exception:
        recorder.close({'startup_failed': True});raise
    print('\a' + '=' * 60);print(f'CONFIRMING pos_p={args.pos_p} vel_ki={args.vel_ki} over {args.trials} real moves');print('=' * 60)
    try:
        time.sleep(0.5)
        wait_for_gain(session, 'position.kp', args.pos_p)
        wait_for_gain(session, 'velocity.ki', args.vel_ki)
        settles = [];overshoots = [];fails = 0
        for i in range(args.trials):
            move, failed, reason = run_one_move(session, args.distance)
            if failed or move is None or move.get('settle_ms') is None:
                fails += 1
                print(f"  trial {i + 1}/{args.trials}: FAILED ({reason})")
                continue
            settles.append(move['settle_ms']);overshoots.append(move['overshoot'])
            print(f"  trial {i + 1}/{args.trials}: settle {move['settle_ms']:.0f} ms, overshoot {move['overshoot']} ct")
        n = len(settles)
        print()
        if n:
            print(f"\aDONE: {n}/{args.trials} succeeded")
            print(f"  settle:    mean {sum(settles) / n:.0f} ms, min {min(settles):.0f}, max {max(settles):.0f}")
            print(f"  overshoot: mean {sum(overshoots) / n:.1f} ct, min {min(overshoots)}, max {max(overshoots)}")
            if fails:
                print(f"  ({fails} trial(s) failed to settle/guard-stopped)")
        else:
            print("\aDONE: every trial failed")
        recorder.event('confirm_summary', settles=settles, overshoots=overshoots, fails=fails)
    finally:
        session.close()


if __name__ == '__main__':
    main()
