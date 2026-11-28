import unittest
from compare_tunes import trial_metrics


def frames(errors):
    return [dict(seq=i, synthetic=True, cmd_position=64, position=64-e,
                 velocity=0, torque=0) for i, e in enumerate(errors)]


class StepMetricTests(unittest.TestCase):
    def test_masked_error_does_not_settle(self):
        metrics = trial_metrics(frames([64]*20+[12]*780))
        self.assertIsNone(metrics['settle_ms'])
        self.assertEqual(metrics['final_error_counts'], 12)

    def test_requires_remaining_observation_inside_band(self):
        metrics = trial_metrics(frames([64]*20+[0]*700+[10]*10+[0]*70))
        self.assertEqual(metrics['settle_ms'], 730)
        metrics = trial_metrics(frames([64]*760+[0]*40))
        self.assertIsNone(metrics['settle_ms'])

    def test_overshoot_and_missing_frames(self):
        data = frames([64]*20+[-3]*780)
        self.assertEqual(trial_metrics(data)['overshoot'], 3)
        with self.assertRaises(ValueError):
            trial_metrics(data[:-1])
        data[300]['seq'] += 1
        with self.assertRaises(ValueError):
            trial_metrics(data)


if __name__ == '__main__':
    unittest.main()
