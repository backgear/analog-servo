#!/usr/bin/env python3
"""Offline PI/P evaluation from measured command-to-encoder frequency response.

No serial access, gain application or config saves. This local linear model
omits quantization, deadband, saturation, trajectory logic and gravity variation.
Predictions are not bench validation or a safety/stability certificate.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares


def plant_response(frequency, parameters, tick_hz=1000):
    """Velocity in counts/tick per torque unit; q = one tick of delay.

    P(q)=b*q*(1+q)/2/(1-a*q)*(1-p)/(1-p*q).
    The q(1+q)/2 numerator captures held acceleration and interval encoder delta.
    a and p are effective fitted poles, not separate measured physical parts.
    """
    b, a, p = parameters
    q = np.exp(-2j * np.pi * np.asarray(frequency) / tick_hz)
    return b * q * (1+q) / 2 / (1-a*q) * (1-p) / (1-p*q)


def fit_plant(frequency, measured, weights=None, tick_hz=1000):
    frequency, measured = np.asarray(frequency), np.asarray(measured)
    if len(frequency) < 8 or np.any(abs(measured) == 0):
        raise ValueError('need >=8 nonzero complex measurements')
    weight = np.ones(len(frequency)) if weights is None else np.sqrt(weights)
    def residual(x):
        predicted = plant_response(frequency, (np.exp(x[0]), x[1], x[2]), tick_hz)
        error = (predicted / measured - 1) * weight
        return np.r_[error.real, error.imag]
    fit = least_squares(residual, [np.log(.00018), .995, .2],
                        bounds=([np.log(1e-7), .8, 0], [np.log(.01), .9999999, .95]))
    if not fit.success:
        raise ValueError('model fit failed: ' + fit.message)
    parameters = [float(np.exp(fit.x[0])), float(fit.x[1]), float(fit.x[2])]
    return parameters


def velocity_controller(frequency, kp_raw, ki_raw, tick_hz=1000):
    q = np.exp(-2j*np.pi*np.asarray(frequency)/tick_hz)
    # Firmware adds this tick's error to the integral before forming output.
    return kp_raw/256 + (ki_raw/256)/(1-q)


def crossings(frequency, values, level=0):
    result = []
    for i in range(len(frequency)-1):
        if values[i] >= level > values[i+1]:
            fraction = (level-values[i])/(values[i+1]-values[i])
            f = np.exp(np.log(frequency[i]) + fraction*np.log(frequency[i+1]/frequency[i]))
            result.append((i, float(fraction), float(f)))
    return result


def margins(frequency, loop):
    """Sampled-frequency diagnostics; only descending crossings are reported."""
    db = 20*np.log10(abs(loop))
    phase = np.rad2deg(np.unwrap(np.angle(loop)))
    unity = [dict(frequency_hz=f, phase_margin_deg=float(180+phase[i]+w*(phase[i+1]-phase[i])))
             for i, w, f in crossings(frequency, db)]
    phase_cross = [dict(frequency_hz=f, gain_margin_db=float(-(db[i]+w*(db[i+1]-db[i]))))
                   for i, w, f in crossings(frequency, phase, -180)]
    return dict(unity_crossings=unity, phase_crossings=phase_cross)


def closed_loop_poles(parameters, candidate):
    b, a, p = parameters
    k = b*(1-p)/2
    numerator = np.convolve([(candidate['velocity_kp']+candidate['velocity_ki'])/256,
                             -candidate['velocity_kp']/256], [0, k, k])
    denominator = np.convolve([1, -1], [1, -(a+p), a*p])
    velocity_den = denominator + numerator
    position_den = np.convolve(velocity_den, [1, -1]) + np.pad(
        numerator * candidate['position_kp']/256, (0, 1))
    return np.roots(velocity_den), np.roots(position_den)


def simulate(parameters, candidate, seconds=2, tick_hz=1000, position_loop=False):
    """Ideal linear unit steps. Not firmware emulation; no limits/deadband."""
    b, a, p = parameters
    kp, ki, pos = [candidate[key]/256 for key in ('velocity_kp', 'velocity_ki', 'position_kp')]
    velocity = np.zeros(int(seconds*tick_hz))
    torque = np.zeros_like(velocity)
    position = np.zeros_like(velocity)
    integral = 0.
    for i in range(len(velocity)):
        v1, v2 = (velocity[i-1] if i >= 1 else 0), (velocity[i-2] if i >= 2 else 0)
        u1, u2 = (torque[i-1] if i >= 1 else 0), (torque[i-2] if i >= 2 else 0)
        velocity[i] = (a+p)*v1-a*p*v2+b*(1-p)/2*(u1+u2)
        position[i] = (position[i-1] if i else 0)+velocity[i]
        target_velocity = pos*(1-position[i]) if position_loop else 1.
        error = target_velocity-velocity[i]
        integral += ki*error
        torque[i] = kp*error+integral
    return dict(time=np.arange(len(velocity))/tick_hz, velocity=velocity,
                position=position, torque=torque)


def step_metrics(response, tick_hz=1000):
    outside = np.flatnonzero(abs(response-1) > .02)
    settle = (int(outside[-1])+1)/tick_hz if len(outside) else 0.
    return dict(overshoot_percent=float(max(0, response.max()-1)*100),
                settle_2_percent_ms=float(settle*1000) if settle < len(response)/tick_hz else None)


def load_spectrum(path, tick_hz=1000):
    rows = np.genfromtxt(path, delimiter=',', names=True)
    f = rows['frequency_hz']
    # Captures/analysis use counts/s; the firmware controls counts/tick.
    measured = rows['velocity_gain_counts_per_s_per_torque']/tick_hz * np.exp(1j*np.deg2rad(rows['phase_deg']))
    coherence = np.minimum(rows['reference_torque_coherence'], rows['reference_motion_coherence'])
    return f, measured, coherence


def comparison_metrics(frequency, measured, coherence, parameters, low, high, tick_hz=1000):
    good = (frequency >= low) & (frequency <= high) & (coherence >= .9) & np.isfinite(measured)
    if not good.any():
        return dict(bins=0)
    ratio = plant_response(frequency[good], parameters, tick_hz)/measured[good]
    return dict(bins=int(good.sum()),
                median_gain_error_percent=float(np.median(abs(abs(ratio)-1))*100),
                p95_gain_error_percent=float(np.percentile(abs(abs(ratio)-1), 95)*100),
                median_phase_error_deg=float(np.median(abs(np.angle(ratio, deg=True)))),
                p95_phase_error_deg=float(np.percentile(abs(np.angle(ratio, deg=True)), 95)))


def evaluate(parameters, candidate, tick_hz=1000):
    frequency = np.geomspace(.02, tick_hz*.49, 5000)
    q = np.exp(-2j*np.pi*frequency/tick_hz)
    loop = plant_response(frequency, parameters, tick_hz)*velocity_controller(
        frequency, candidate['velocity_kp'], candidate['velocity_ki'], tick_hz)
    position_loop = candidate['position_kp']/256 * loop/(1+loop)/(1-q)
    vp, pp = closed_loop_poles(parameters, candidate)
    velocity_stable, position_stable = bool(max(abs(vp)) < 1), bool(max(abs(pp)) < 1)
    vs = simulate(parameters, candidate, tick_hz=tick_hz) if velocity_stable else None
    ps = simulate(parameters, candidate, tick_hz=tick_hz, position_loop=True) if position_stable else None
    return dict(candidate=candidate, velocity_stable_in_model=velocity_stable,
                position_stable_in_model=position_stable, velocity_margins=margins(frequency, loop),
                position_margins=margins(frequency, position_loop),
                velocity_max_pole_magnitude=float(max(abs(vp))),
                position_max_pole_magnitude=float(max(abs(pp))),
                velocity_step=step_metrics(vs['velocity'], tick_hz) if vs else None,
                position_step=step_metrics(ps['position'], tick_hz) if ps else None,
                unit_velocity_step_peak_torque=float(max(abs(vs['torque']))) if vs else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('spectrum', help='training CSV from analyze_excitation.py')
    ap.add_argument('--validate', nargs='*', default=[])
    ap.add_argument('--settings', required=True, help='recorded CLI settings snapshot')
    ap.add_argument('--candidates', help='optional JSON list with name/position_kp/velocity_kp/velocity_ki')
    ap.add_argument('--out', required=True, help='output prefix')
    ap.add_argument('--tick-hz', type=int, default=1000)
    args = ap.parse_args()
    if args.tick_hz <= 0:
        ap.error('positive tick rate required')
    settings = dict(line.split('=', 1) for line in Path(args.settings).read_text().splitlines() if '=' in line)
    if int(settings.get('velocity.filter_shift', '0')) != 0:
        ap.error('this evaluator currently requires velocity.filter_shift=0')
    current = dict(name='current', position_kp=int(settings['position.kp']),
                   velocity_kp=int(settings['velocity.kp']), velocity_ki=int(settings['velocity.ki']))
    candidates = [current, dict(name='gentler', position_kp=12, velocity_kp=183000, velocity_ki=82),
                  dict(name='balanced', position_kp=16, velocity_kp=289000, velocity_ki=82),
                  dict(name='balanced_integral_probe', position_kp=16, velocity_kp=289000, velocity_ki=500),
                  dict(name='stiffer', position_kp=21, velocity_kp=365000, velocity_ki=82)]
    if args.candidates:
        candidates = json.loads(Path(args.candidates).read_text())
    if not candidates or any(type(c[k]) is not int or not 1 <= c[k] <= 2147483647
                             for c in candidates for k in ('position_kp', 'velocity_kp', 'velocity_ki')):
        ap.error('positive int32 candidate gains required')
    f, measured, coherence = load_spectrum(args.spectrum, args.tick_hz)
    mask = (f >= 8) & (f <= 150) & (coherence >= .95) & np.isfinite(measured)
    parameters = fit_plant(f[mask], measured[mask], coherence[mask], args.tick_hz)
    checks = []
    for path in [args.spectrum, *args.validate]:
        vf, vm, vc = load_spectrum(path, args.tick_hz)
        checks.append(dict(source=path, training=path == args.spectrum,
                           fit_band=comparison_metrics(vf, vm, vc, parameters, 8, 150, args.tick_hz),
                           extrapolation=comparison_metrics(vf, vm, vc, parameters, 150, 250, args.tick_hz)))
    results = [evaluate(parameters, c, args.tick_hz) for c in candidates]
    for result, candidate in zip(results, candidates):
        result['position_error_counts_at_soft_band_boundary'] = (
            int(settings.get('velocity.deadband', '0'))*256/candidate['position_kp'])
        result['measured_velocity_margins_8_to_150_hz'] = margins(
            f[mask], measured[mask]*velocity_controller(f[mask], candidate['velocity_kp'],
                                                       candidate['velocity_ki'], args.tick_hz))
        for loop_name in ('velocity_margins', 'position_margins'):
            for crossings_list in result[loop_name].values():
                for crossing in crossings_list:
                    crossing['outside_training_band'] = not 8 <= crossing['frequency_hz'] <= 150
    report = dict(status='offline local linear predictions; not bench-validated',
                  warnings=['Fit uses 8–150 Hz only: DC friction, integral tuning and slow step tails are not independently identified.',
                            'Margins with crossings outside the training band are extrapolations.',
                            'Unit steps are ideal linear illustrations; nonlinear proportional noise-band shaping is not modeled.',
                            'Default candidates are illustrative comparisons, not optimized or approved gains.',
                            'Configured soft band reduces small-error P gain; I still receives the full error.'],
                  noise_band_behavior='half-slope P inside band, full-error I',
                  recorded_velocity_deadband=int(settings.get('velocity.deadband', '0')),
                  exclusions=['deadband', 'encoder quantization', 'saturation/anti-windup',
                              'trajectory and reference-limit switching', 'gravity/load variation'],
                  training_source=args.spectrum, settings_source=args.settings, tick_hz=args.tick_hz,
                  training_bins=int(mask.sum()), parameters=dict(zip(['b', 'a', 'p'], parameters)),
                  checks=checks, evaluations=results)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = ['# Offline tuning evaluation', '', '**Predictions only; not bench-validated. No hardware access or gain changes.**', '',
             f'Training: `{args.spectrum}`, {int(mask.sum())} bins at 8–150 Hz.', '',
             '| Candidate | Position P | Velocity P | Velocity I | Velocity crossover | Phase margin | Position crossover | Stable model poles |',
             '|---|---:|---:|---:|---:|---:|---:|---|']
    for result in results:
        c = result['candidate']
        v = result['velocity_margins']['unity_crossings']
        p = result['position_margins']['unity_crossings']
        lines.append(f'| {c["name"]} | {c["position_kp"]} | {c["velocity_kp"]} | {c["velocity_ki"]} | '
                     + (f'{v[0]["frequency_hz"]:.1f} Hz | {v[0]["phase_margin_deg"]:.1f}° | ' if v else 'not found | not found | ')
                     + (f'{p[0]["frequency_hz"]:.1f} Hz |' if p else 'not found |')
                     + (' yes |' if result['velocity_stable_in_model'] and result['position_stable_in_model'] else ' NO |'))
    lines += ['', 'All gain values above are raw Q8.8 integers.', '', '## Fit checks (8–150 Hz, coherence >=0.9)', '',
              '| Data | Fit/held-out | Bins | Median gain error | Median phase error |', '|---|---|---:|---:|---:|']
    for check in checks:
        stats = check['fit_band']
        if stats['bins']:
            lines.append(f'| `{check["source"]}` | {"fit" if check["training"] else "held-out"} | {stats["bins"]} | '
                         f'{stats["median_gain_error_percent"]:.1f}% | {stats["median_phase_error_deg"]:.1f}° |')
    lines += ['', '## Limitations', '', *['- '+warning for warning in report['warnings']], '',
              'Default candidates are manually chosen comparisons, not a search result or a ranked recommendation.',
              'Model coefficients are effective discrete-time parameters, not calibrated inertia/friction/current-loop measurements.',
              'See the JSON for poles, step metrics, measured-data margins and out-of-band checks.', '']
    out.with_suffix('.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    good = (f >= 5) & (f <= 250) & (coherence >= .9)
    axes[0, 0].loglog(f[good], abs(measured[good]), '.', label='measured, coherence >=0.9')
    grid = np.geomspace(5, 250, 1000)
    axes[0, 0].loglog(grid, abs(plant_response(grid, parameters, args.tick_hz)), label='fitted model')
    axes[0, 0].set(title='Local plant fit (8–150 Hz training)', xlabel='Hz', ylabel='counts/tick per torque unit')
    for c, result in zip(candidates, results):
        loop = plant_response(grid, parameters, args.tick_hz)*velocity_controller(grid, c['velocity_kp'], c['velocity_ki'], args.tick_hz)
        axes[0, 1].semilogx(grid, 20*np.log10(abs(loop)), label=c['name'])
        if result['velocity_stable_in_model']:
            vs = simulate(parameters, c, tick_hz=args.tick_hz)
            axes[1, 0].plot(vs['time']*1000, vs['velocity'], label=c['name'])
        if result['position_stable_in_model']:
            ps = simulate(parameters, c, tick_hz=args.tick_hz, position_loop=True)
            axes[1, 1].plot(ps['time']*1000, ps['position'], label=c['name'])
    axes[0, 1].axhline(0, color='gray', linestyle=':')
    axes[0, 1].set(title='Predicted velocity open-loop gain', xlabel='Hz', ylabel='dB')
    for axis, title in zip(axes[1], ['Ideal unit velocity step', 'Ideal unit position step']):
        axis.axhline(1, color='gray', linestyle=':')
        axis.set(title=title, xlabel='ms', ylabel='Normalized response', xlim=(0, 500))
    for axis in axes.flat:
        axis.grid(True, alpha=.25)
        axis.legend(fontsize=7)
    fig.suptitle('OFFLINE LINEAR PREDICTIONS — not firmware simulation or bench validation')
    fig.savefig(out.with_suffix('.png'), dpi=140)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
