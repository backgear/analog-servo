"""Common synthetic-run metrics. No plotting or external dependencies."""
import math


def rms(values):
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def hold_metrics(frames, band_counts=5, velocity_band=1, dwell_seconds=0.25):
    """Settle only if both error and speed stay in band to the hold's end.

    A suffix shorter than dwell_seconds is not sufficient evidence. None means
    'not settled within this recorded hold', never zero or missing evidence.
    """
    holds = []
    start = 0
    while start < len(frames):
        phase = frames[start].get('phase', '')
        end = start + 1
        while end < len(frames) and frames[end].get('phase', '') == phase:
            end += 1
        if 'hold' in phase:
            data = frames[start:end]
            record = dict(phase=phase, start=data[0]['t'], end=data[-1]['t'] + 0.001,
                          band_counts=band_counts, velocity_band=velocity_band,
                          dwell_seconds=dwell_seconds, torque_rms=rms(f['torque'] for f in data),
                          torque_delta_rms=rms(b['torque'] - a['torque']
                                              for a, b in zip(data, data[1:])))
            for label, key, velocity in (('motor', 'position', 'velocity'),
                                         ('load', 'load_position', 'load_velocity')):
                bad = [i for i, f in enumerate(data)
                       if abs(f['cmd_position'] - f[key]) > band_counts
                       or abs(f[velocity]) > velocity_band]
                first = bad[-1] + 1 if bad else 0
                enough = (len(data) - first) * 0.001 >= dwell_seconds
                record[label + '_settling_seconds'] = (
                    data[first]['t'] - data[0]['t'] if enough else None)
                record[label + '_final_error'] = data[-1]['cmd_position'] - data[-1][key]
            holds.append(record)
        start = end
    return holds


def tracking_metrics(frames, torque_limit=1000):
    if not frames:
        raise ValueError('cannot score an empty run')
    result = {}
    for label, key in (('motor', 'position'), ('load', 'load_position')):
        errors = [f['cmd_position'] - f[key] for f in frames]
        result[label + '_rms'] = rms(errors)
        result[label + '_peak'] = max(map(abs, errors))
    torque = [f['torque'] for f in frames]
    result.update(torque_rms=rms(torque),
                  torque_delta_rms=rms(b['torque'] - a['torque']
                                       for a, b in zip(frames, frames[1:])
                                       if b['t'] - a['t'] <= 0.001001),
                  saturation_ticks=sum(abs(t) >= torque_limit for t in torque))
    return result


def measure(frames, torque_limit=1000):
    result = tracking_metrics(frames, torque_limit)
    result['holds'] = hold_metrics(frames)
    moving = [f for f in frames if 'hold' not in f.get('phase', '')]
    result['moving'] = tracking_metrics(moving, torque_limit) if moving else None
    result['phases'] = []
    start = 0
    while start < len(frames):
        phase = frames[start].get('phase', '')
        end = start + 1
        while end < len(frames) and frames[end].get('phase', '') == phase:
            end += 1
        result['phases'].append(dict(phase=phase, start=frames[start]['t'],
                                     end=frames[end - 1]['t'] + 0.001,
                                     **tracking_metrics(frames[start:end], torque_limit)))
        start = end
    return result


def balanced_score(metrics):
    """Heuristic tracking/activity compromise, NOT an energy or hardware metric."""
    return (metrics['motor_rms'] + metrics['load_rms']
            + 0.8 * metrics['torque_delta_rms'] + 0.1 * metrics['torque_rms']
            + metrics['saturation_ticks'])
