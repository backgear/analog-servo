import unittest
import numpy as np
from analyze_trim_motion import matched_pair


class TrimMotionTests(unittest.TestCase):
    def data(self, trim=300, speed=1):
        pairs = []
        for direction in [1, -1]:
            rows = [dict(position=p, torque=333-trim+direction*200+100*np.sin(2*np.pi*p/4000))
                    for p in range(0, 8001, speed)]
            pairs.append(rows if direction > 0 else rows[::-1])
        return pairs

    def test_recovers_offset_despite_friction_and_position_variation(self):
        result = matched_pair(*self.data(), trim=300)
        self.assertAlmostEqual(result['estimated_trim'], 333)
        self.assertAlmostEqual(result['odd_effort_mean'], 200)
        self.assertGreater(result['spatial_even_std'], 60)

    def test_trim_adjustment_changes_residual_not_offset_estimate(self):
        result = matched_pair(*self.data(trim=330), trim=330)
        self.assertAlmostEqual(result['controller_midpoint'], 3)
        self.assertAlmostEqual(result['estimated_trim'], 333)

    def test_speed_and_sample_order_do_not_change_offset(self):
        result = matched_pair(*self.data(speed=2), trim=300)
        self.assertAlmostEqual(result['estimated_trim'], 333)
        self.assertEqual(result['turns'], 2)

    def test_rejects_less_than_one_turn_of_overlap(self):
        rows = [dict(position=p, torque=0) for p in range(1000)]
        with self.assertRaises(ValueError):
            matched_pair(rows, rows, trim=300)


if __name__ == '__main__':
    unittest.main()
