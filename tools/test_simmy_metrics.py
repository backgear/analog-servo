"""Deterministic scoring tests without simmy or matplotlib."""
import unittest

from simmy_metrics import hold_metrics, measure


def frame(i, position=100, velocity=0):
    return dict(t=i / 1000, cmd_position=100, position=position, load_position=position,
                velocity=velocity, load_velocity=velocity, torque=0, phase='final-hold')


class MetricsTests(unittest.TestCase):
    def test_settling_requires_persistent_error_and_velocity_band(self):
        frames = [frame(i, 90 if i < 100 else 100) for i in range(1000)]
        frames[300]['velocity'] = 2
        metrics = hold_metrics(frames)[0]
        self.assertAlmostEqual(metrics['motor_settling_seconds'], 0.301)
        self.assertAlmostEqual(metrics['load_settling_seconds'], 0.100)

    def test_short_tail_or_unsettled_is_not_reported_as_success(self):
        frames = [frame(i) for i in range(1000)]
        frames[900]['load_position'] = 80
        self.assertIsNone(hold_metrics(frames)[0]['load_settling_seconds'])
        self.assertIsNone(hold_metrics([frame(i, 90) for i in range(1000)])[0]['motor_settling_seconds'])

    def test_excluded_holds_do_not_create_spurious_torque_increments(self):
        frames = [frame(0), frame(1), frame(1000), frame(1001)]
        for f in frames:
            f['phase'] = 'cruise'
        frames[2]['torque'] = frames[3]['torque'] = 100
        metrics = measure(frames)
        self.assertEqual(metrics['moving']['torque_delta_rms'], 0)

    def test_saturation_and_signed_error_metrics(self):
        frames = [frame(0, 97), frame(1, 104)]
        frames[0]['torque'] = 1000
        frames[1]['torque'] = -1000
        metrics = measure(frames)
        self.assertEqual(metrics['saturation_ticks'], 2)
        self.assertEqual(metrics['motor_peak'], 4)
        self.assertEqual(metrics['torque_delta_rms'], 2000)
        self.assertEqual(metrics['motor_rms'], (12.5)**0.5)


if __name__ == '__main__':
    unittest.main()
