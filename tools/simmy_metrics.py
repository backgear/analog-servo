"""Common synthetic-run metrics. No plotting or external dependencies."""
import math


def rms(values):
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def measure(frames, torque_limit=1000):
    if not frames:
        raise ValueError('cannot score an empty run')
    result = {}
    for label, key in (('motor', 'position'), ('load', 'load_position')):
        errors = [f['cmd_position'] - f[key] for f in frames]
        result[label + '_rms'] = rms(errors)
        result[label + '_peak'] = max(map(abs, errors))
    torque = [f['torque'] for f in frames]
    result.update(torque_rms=rms(torque),
                  torque_delta_rms=rms(b - a for a, b in zip(torque, torque[1:])),
                  saturation_ticks=sum(abs(t) >= torque_limit for t in torque))
    return result


def balanced_score(metrics):
    """Heuristic tracking/activity compromise, NOT an energy or hardware metric."""
    return (metrics['motor_rms'] + metrics['load_rms']
            + 0.8 * metrics['torque_delta_rms'] + 0.1 * metrics['torque_rms']
            + metrics['saturation_ticks'])
