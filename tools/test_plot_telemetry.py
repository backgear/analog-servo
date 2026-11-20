"""Renderer tests: python3 -m unittest discover -s tools -p test_plot_telemetry.py"""
import unittest

import matplotlib.pyplot as plt

from plot_telemetry import following_error_panel, panel, BLUE


class FollowingErrorTests(unittest.TestCase):
    def test_signed_error_is_not_rebased_with_position_or_time(self):
        data = [dict(t=5.0, cmd_position=110, position=100),
                dict(t=5.1, cmd_position=112, position=115)]
        fig, ax = plt.subplots()
        try:
            panel(ax, data, 5.0, 'cmd_position', 'position', BLUE,
                  'commanded', 'actual', 'position', baseline=100)
            error_ax = following_error_panel(ax, data, 5.0)
            self.assertEqual(list(error_ax.lines[0].get_ydata()), [10, -3])
            self.assertAlmostEqual(error_ax.lines[0].get_xdata()[1], 0.1)
            self.assertEqual(list(ax.lines[1].get_ydata()), [10, 12])
            self.assertEqual(error_ax.get_ylabel(), 'following error (ct)')
            self.assertEqual(len(error_ax.get_legend().get_texts()), 3)
            self.assertIsNone(ax.get_legend())
        finally:
            plt.close(fig)

    def test_load_error_uses_load_not_feedback_position(self):
        data = [dict(t=0, cmd_position=100, position=99, load_position=80)]
        fig, ax = plt.subplots()
        try:
            panel(ax, data, 0, 'cmd_position', 'load_position', BLUE,
                  'commanded', 'load', 'load position', baseline=99)
            error_ax = following_error_panel(ax, data, 0, 'load_position')
            self.assertEqual(list(error_ax.lines[0].get_ydata()), [20])
            self.assertEqual(list(ax.lines[2].get_ydata()), [-19])
        finally:
            plt.close(fig)


if __name__ == '__main__':
    unittest.main()
