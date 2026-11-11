import unittest
from compare_move_captures import step_metrics, ramp_segment


class DemoMetricTests(unittest.TestCase):
    def data(self, direction=1):
        return [dict(seq=i, cmd_position=100*direction, position=min(i,100)*direction,
                     velocity=0, torque=0) for i in range(150)]

    def test_rise_and_settle_use_firmware_ticks(self):
        result = step_metrics(self.data())
        self.assertEqual(result['rise_10_to_90_ms'], 80)
        self.assertEqual(result['settle_5_counts_ms'], 95)
        self.assertEqual(result['overshoot_counts'], 0)

    def test_negative_direction_overshoot(self):
        data = self.data(-1)
        data[110]['position'] = -102
        self.assertEqual(step_metrics(data)['overshoot_counts'], 2)

    def test_ramp_selection_excludes_late_hold_retarget(self):
        data = [dict(seq=i, synthetic=False, cmd_velocity=v)
                for i, v in enumerate([0, 1, 2, 3, 2, 1, 0, 0, 1, 0])]
        self.assertEqual([r['seq'] for r in ramp_segment(data)], [1, 2, 3, 4, 5])

    def test_missing_tick_is_rejected(self):
        data = self.data()
        data[10]['seq'] += 1
        with self.assertRaises(ValueError):
            step_metrics(data)


if __name__ == '__main__':
    unittest.main()
