import unittest
import numpy as np
from scipy.signal import lfilter
from evaluate_tune import (plant_response, fit_plant, velocity_controller,
                           closed_loop_poles, simulate, margins, step_metrics, evaluate)


class ModelTests(unittest.TestCase):
    parameters = [.000186, .9913, .266]
    candidate = dict(name='test', velocity_kp=338630, velocity_ki=82, position_kp=21)

    def test_recovers_synthetic_model(self):
        f = np.linspace(8, 150, 100)
        fit = fit_plant(f, plant_response(f, self.parameters))
        np.testing.assert_allclose(fit, self.parameters, rtol=1e-5)

    def test_plant_delay_and_units(self):
        # Zero-pole special case: P=b*q*(1+q)/2.
        f = np.array([50., 100.])
        q = np.exp(-2j*np.pi*f/1000)
        np.testing.assert_allclose(plant_response(f, [.0002, 0, 0]), .0001*(q+q*q))

    def test_pi_is_current_tick_integral(self):
        f = np.array([10., 100.])
        q = np.exp(-2j*np.pi*f/1000)
        np.testing.assert_allclose(velocity_controller(f, 256, 128), 1+.5/(1-q))

    def test_time_and_frequency_closed_loops_agree(self):
        b, a, p = self.parameters
        c = self.candidate
        k = b*(1-p)/2
        numerator = np.convolve([(c['velocity_kp']+c['velocity_ki'])/256,
                                 -c['velocity_kp']/256], [0, k, k])
        velocity_den = np.convolve([1, -1], [1, -(a+p), a*p])+numerator
        expected_v = lfilter(numerator, velocity_den, np.ones(2000))
        np.testing.assert_allclose(simulate(self.parameters, c)['velocity'], expected_v, atol=1e-10)
        position_num = numerator*c['position_kp']/256
        position_den = np.convolve(velocity_den, [1, -1])+np.pad(position_num, (0, 1))
        expected_x = lfilter(position_num, position_den, np.ones(2000))
        np.testing.assert_allclose(simulate(self.parameters, c, position_loop=True)['position'], expected_x, atol=1e-9)
        vp, pp = closed_loop_poles(self.parameters, c)
        self.assertLess(max(abs(vp)), 1)
        self.assertLess(max(abs(pp)), 1)

    def test_unstable_candidate_is_flagged_without_simulation(self):
        candidate = self.candidate | dict(velocity_kp=100000000)
        result = evaluate(self.parameters, candidate)
        self.assertFalse(result['velocity_stable_in_model'])
        self.assertFalse(result['position_stable_in_model'])
        self.assertIsNone(result['velocity_step'])
        self.assertIsNone(result['position_step'])

    def test_known_margin(self):
        f = np.geomspace(.1, 100, 1000)
        result = margins(f, 10/(2j*np.pi*f))
        self.assertAlmostEqual(result['unity_crossings'][0]['frequency_hz'], 10/(2*np.pi), places=5)
        self.assertAlmostEqual(result['unity_crossings'][0]['phase_margin_deg'], 90)
        self.assertEqual(result['phase_crossings'], [])

    def test_unsettled_is_not_reported_as_success(self):
        self.assertIsNone(step_metrics(np.zeros(100))['settle_2_percent_ms'])


if __name__ == '__main__':
    unittest.main()
