#!/usr/bin/env python3
"""One-off grid-sweep-and-subdivide tuning experiment for pos_p ("P") and vel_ki ("I").

NOT the one-button autotune campaign (see autoautotune.py / scorecard_session.py's
AUTOTUNE_* coordinate-descent state machine) -- that nudges by a percentage of the
current value each trial, which can jump straight past an interesting region (e.g.
pos_p=1 -> pos_p=17 in one move, skipping everything between). This instead:

  1. Sweeps ~10 explicit P values, holding vel_kp/vel_ki fixed at whatever's live.
  2. Takes the best TWO, subdivides the interval between them, re-measures, keeps the
     overall best P.
  3. Holds that P fixed, does the same sweep-and-subdivide for I (vel_ki).
  4. Holds that I fixed, re-sweeps P once more (the plant's apparent best P can shift
     once I is no longer at its original value).

Reuses LiveSession's existing 'gain'/'move' actions (already thread-safe, queued through
the same control thread as the GUI/autoautotune use) rather than driving the serial port
directly -- this script only ever calls session.request(...) and polls session.model/
session.messages, exactly like autoautotune.py's own foreground loop.
"""
import argparse
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy');os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from scorecard_core import Criteria, Recorder
from scorecard_session import LiveSession, AUTOTUNE_OVERSHOOT_COST_WEIGHT, AUTOTUNE_TRIAL_REPEATS, AUTOTUNE_MIN_IMPROVEMENT_FRACTION

SETTLE_WAIT_S = 2.0
MOVE_TIMEOUT_S = 6.0
GAIN_APPLY_TIMEOUT_S = 2.0


def cost_of(move, failed):
    if failed or move is None or move.get('settle_ms') is None:
        return 1e9 + (abs(move['overshoot']) if move else 0)
    return move['settle_ms'] + abs(move['overshoot']) * AUTOTUNE_OVERSHOOT_COST_WEIGHT


def wait_for_gain(session, coord, value, timeout=GAIN_APPLY_TIMEOUT_S):
    session.request('gain', (coord, value))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if session.settings.get(coord) == str(value):
            return True
        time.sleep(0.05)
    return False


def clear_firmware_fault(session, timeout=2.0):
    # A real fault latch (e.g. the drive stalling at low torque -> zero_encoder) blocks every
    # later 'move' request client-side until cleared -- without this, one bad candidate poisons
    # the rest of the sweep instead of just failing its own trial.
    session.request('clear')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snap = session.model.snapshot()
        if snap['latest'] and not snap['latest']['fault_latched']:
            return True
        time.sleep(0.05)
    return False


def run_one_move(session, distance):
    deadline = time.monotonic() + SETTLE_WAIT_S
    while time.monotonic() < deadline:
        snap = session.model.snapshot()
        if snap['latest'] and abs(snap.get('mean100ms_speed', 0)) <= 1:
            break
        time.sleep(0.05)
    session.request('move', distance)
    deadline = time.monotonic() + MOVE_TIMEOUT_S
    while time.monotonic() < deadline:
        snap = session.model.snapshot()
        move = snap.get('move')
        if snap['guard']:
            if snap['guard'].startswith('Firmware fault:'):
                clear_firmware_fault(session)
            return move, True, snap['guard']
        if move and move.get('settle_ms') is not None:
            return move, False, None
        time.sleep(0.05)
    return session.model.snapshot().get('move'), True, 'timeout'


def measure(session, coord, value, distance, repeats=AUTOTUNE_TRIAL_REPEATS):
    if not wait_for_gain(session, coord, value):
        return None, None, True, 'gain apply timed out'
    costs = [];last_move = None;last_failed = False;last_reason = None
    for _ in range(repeats):
        move, failed, reason = run_one_move(session, distance)
        costs.append(cost_of(move, failed));last_move = move;last_failed = failed;last_reason = reason
    return sum(costs) / len(costs), last_move, last_failed, last_reason


def describe(move, failed, reason):
    if failed or move is None or move.get('settle_ms') is None:
        return f"FAILED ({reason})" if failed else 'did not settle'
    return f"settle {move['settle_ms']:.0f} ms, overshoot {move['overshoot']} ct"


def sweep_and_subdivide(session, coord, candidates, distance, label):
    results = []   # (value, cost, move, failed, reason)
    for value in candidates:
        cost, move, failed, reason = measure(session, coord, value, distance)
        results.append((value, cost, move, failed, reason))
        print(f"  [{label}] {coord}={value}: cost={cost:.0f} ({describe(move, failed, reason)})")
    results.sort(key=lambda r: r[1])
    lo, hi = sorted((results[0][0], results[1][0]))
    if hi > lo:
        subs = sorted(set(round(lo + (hi - lo) * f) for f in (0.2, 0.4, 0.5, 0.6, 0.8)) - {r[0] for r in results})
        for value in subs:
            cost, move, failed, reason = measure(session, coord, value, distance)
            results.append((value, cost, move, failed, reason))
            print(f"  [{label} subdivide] {coord}={value}: cost={cost:.0f} ({describe(move, failed, reason)})")
    results.sort(key=lambda r: r[1])
    best = results[0]
    wait_for_gain(session, coord, best[0])   # leave it AT the winning value, not whatever was tested last
    print(f"  -> best {coord}={best[0]} (cost {best[1]:.0f})")
    return best


def relative_candidates(center, fractions=(0.25, 0.4, 0.6, 0.8, 1.0, 1.25, 1.6, 2.0, 2.5, 3.0)):
    return sorted(set(max(1, round(center * f)) for f in fractions))


def alternate_to_convergence(session, coord_specs, distance, max_rounds=4):
    # Coordinate-wise sweeps can stop wherever a fixed schedule happens to end, not at an
    # actual joint optimum -- keep alternating every coordinate until a full round changes
    # NONE of them, instead of stopping after one fixed pass. Still coordinate-wise (see
    # coarse_grid_check for the other half of the fix -- that one stays P/I-only, see its
    # own comment for why extending it to every coordinate here wasn't worth the trial count).
    # coord_specs: list of (coord_name, make_candidates(session, best)->[values], label).
    best = {}
    best_cost = None
    summary = ''
    for round_num in range(1, max_rounds + 1):
        changed = False
        for coord, make_candidates, label in coord_specs:
            held = ', '.join(f'{c}={best[c]}' for c, _, _ in coord_specs if c in best and c != coord)
            print(f"\n--- round {round_num}: sweep {label}" + (f" (holding {held})" if held else '') + " ---")
            result = sweep_and_subdivide(session, coord, make_candidates(session, best), distance, f'{label} r{round_num}')
            if best.get(coord) != result[0]:
                changed = True
            best[coord], best_cost = result[0], result[1]
        summary = ' '.join(f"{label}={best[coord]}" for coord, _, label in coord_specs)
        if not changed:
            print(f"\nConverged after round {round_num}: {summary}")
            break
        print(f"\nRound {round_num} result: {summary} (cost {best_cost:.0f}) -- still changing, another round")
    else:
        print(f"\nHit the {max_rounds}-round cap without full convergence; using {summary}")
    return best, best_cost


def coarse_grid_check(session, best_p, best_i, best_cost, distance):
    # A bounded joint sanity pass: alternating sweeps only ever move along one axis at a
    # time, so this checks a handful of points that neither axis's own sweep could reach
    # (e.g. "more P AND less I together") without paying for a full dense 2D grid.
    p_grid = sorted(set(v for v in (max(0, best_p - 4), max(0, round(best_p * 0.5)), round(best_p * 1.5), best_p + 6) if v != best_p))
    i_grid = sorted(set(round(best_i * f) for f in (0.6, 0.85, 1.15, 1.5)))
    results = [(best_p, best_i, best_cost)]
    for p in p_grid:
        wait_for_gain(session, 'position.kp', p)
        for i in i_grid:
            cost, move, failed, reason = measure(session, 'velocity.ki', i, distance)
            results.append((p, i, cost))
            print(f"  [grid] P={p} I={i}: cost={cost:.0f} ({describe(move, failed, reason)})")
    results.sort(key=lambda r: r[2])
    grid_p, grid_i, grid_cost = results[0]
    if grid_cost < best_cost * (1.0 - AUTOTUNE_MIN_IMPROVEMENT_FRACTION):
        print(f"\nGrid found something better: P={grid_p} I={grid_i} (cost {grid_cost:.0f} vs converged {best_cost:.0f})")
        return grid_p, grid_i, grid_cost
    print(f"\nGrid confirms the converged point: nothing nearby beat P={best_p} I={best_i} (cost {best_cost:.0f}) by a real margin")
    return best_p, best_i, best_cost


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port');p.add_argument('cli_port')
    p.add_argument('--baud', type=int, default=2500000);p.add_argument('--cli-baud', type=int, default=115200)
    p.add_argument('--cpr', type=int, default=4096, help='Decoded quadrature counts per revolution (4x encoder cycles)');p.add_argument('--tick-hz', type=int, default=1000)
    p.add_argument('--distance', type=int, default=512)
    p.add_argument('--max-command', type=int, default=40)
    p.add_argument('--move-settle-threshold', type=int, default=1, help='counts/tick |mean100ms_speed| must be under before a move will start -- raise this on a drive with persistent ambient drift that never settles near zero, or every move request gets refused client-side')
    p.add_argument('--max-rounds', type=int, default=4, help='alternating-convergence round cap -- P/I/Kp did not converge within the default 4 on the first --include-kp run')
    p.add_argument('--record-dir', default='captures')
    p.add_argument('--p-values', default='1,2,3,4,6,8,10,13,16,20', help='comma-separated pos_p candidates for the sweep')
    p.add_argument('--include-kp', action='store_true', help='also alternate vel_kp into the convergence loop (P -> I -> Kp -> P -> ...), instead of holding it fixed')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    criteria = Criteria(cpr=args.cpr, tick_hz=args.tick_hz)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    recorder = Recorder(Path(args.record_dir) / stamp)
    recorder.event('session', tool='psweep_tune', monitor_port=args.port, cli_port=args.cli_port)
    session = LiveSession(args.port, args.cli_port, recorder, criteria, args.baud, args.cli_baud, args.max_command,
                           move_settle_threshold=args.move_settle_threshold)
    try:
        session.start()
    except Exception:
        recorder.close({'startup_failed': True});raise
    print('\a' + '=' * 60);print('STARTING P/I' + ('/Kp' if args.include_kp else '') + ' SWEEP -- motor will move');print('=' * 60)
    try:
        settings_deadline = time.monotonic() + 5.0
        while not session._settings_ready and time.monotonic() < settings_deadline:
            time.sleep(0.05)
        if not session._settings_ready:
            raise RuntimeError('Device settings never arrived (settings fetch timed out)')
        p_candidates = [int(v) for v in args.p_values.split(',')]
        coord_specs = [
            ('position.kp', lambda session, best: p_candidates, 'P'),
            ('velocity.ki', lambda session, best: relative_candidates(best.get('velocity.ki') or int(session.settings.get('velocity.ki', 500) or 500)), 'I'),
        ]
        if args.include_kp:
            coord_specs.append(('velocity.kp', lambda session, best: relative_candidates(best.get('velocity.kp') or int(session.settings.get('velocity.kp', 50000) or 50000), fractions=(0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0)), 'Kp'))
        else:
            print(f"vel_kp held fixed throughout at {session.settings.get('velocity.kp')}")

        print("\n=== Stage 1: alternate sweeps to convergence ===")
        best, best_cost = alternate_to_convergence(session, coord_specs, args.distance, max_rounds=args.max_rounds)

        print("\n=== Stage 2: coarse joint grid check (pos_p x vel_ki) around the converged point ===")
        # Stays 2D even with --include-kp -- a 3D grid at the same resolution is 4x the real
        # moves (64 vs 16) for a check that's already just a sanity pass, not the main search.
        final_p, final_i, final_cost = coarse_grid_check(session, best['position.kp'], best['velocity.ki'], best_cost, args.distance)
        if args.include_kp:
            wait_for_gain(session, 'velocity.kp', best['velocity.kp'])

        wait_for_gain(session, 'position.kp', final_p)
        wait_for_gain(session, 'velocity.ki', final_i)
        final_move, final_failed, final_reason = run_one_move(session, args.distance)
        kp_note = f" vel_kp={best['velocity.kp']}" if args.include_kp else ' (vel_kp unchanged)'
        print(f"\n\aFINAL: pos_p={final_p} vel_ki={final_i}{kp_note} (cost {final_cost:.0f}) -- {describe(final_move, final_failed, final_reason)}")
        recorder.event('psweep_final', pos_p=final_p, vel_ki=final_i, vel_kp=best.get('velocity.kp'), cost=final_cost, move=final_move)
    finally:
        session.close()


if __name__ == '__main__':
    main()
