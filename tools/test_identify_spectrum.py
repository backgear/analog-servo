"""Host-only tests: python3 -m unittest discover -s tools -p test_identify_spectrum.py."""
import unittest
import numpy as np
from identify_spectrum import contiguous_runs, estimate, estimate_closed_loop


class SpectrumTests(unittest.TestCase):
    def test_wrap_and_gap(self):
        frames = [{'seq': i} for i in [0xfffffffe, 0xffffffff, 0, 2, 2, 3]]
        self.assertEqual([len(r) for r in contiguous_runs(frames)], [3, 1, 2])

    def test_known_gain(self):
        rng = np.random.default_rng(17)
        frames = [dict(seq=i, torque=u, velocity=2*u, fault_latched=False)
                  for i, u in enumerate(rng.normal(size=8192))]
        _, h, coherence, windows = estimate(frames, tick_hz=1000, nperseg=512)
        self.assertEqual(windows, 31)
        np.testing.assert_allclose(abs(h[1:]), 2000, rtol=1e-10)
        np.testing.assert_allclose(coherence[1:], 1, atol=1e-10)

    def test_closed_loop_reference(self):
        rng = np.random.default_rng(31)
        frames = []
        # Algebraic feedback with independent output disturbance: y=2*u+n,
        # u=d-0.4*y. A naive y/u spectrum is biased; d is independent of n.
        for i in range(65536):
            d, noise = rng.normal(size=2)
            y = (2*d + noise) / 1.8
            u = d - .4*y
            frames.append(dict(seq=i, injected_torque=d, torque=u, velocity=y/1000,
                               excitation=True, fault_latched=False, enabled=True, hw_enable=True))
        r = estimate_closed_loop(frames)
        self.assertEqual(r['excited_frames'], 65536)
        self.assertEqual(r['runs'], 1)
        self.assertLess(np.nanmedian(abs(r['response'][1:] - 2)), .12)

    def test_closed_loop_rejects_unmarked(self):
        with self.assertRaises(ValueError):
            estimate_closed_loop([dict(seq=i) for i in range(2048)])

    def test_no_excitation(self):
        frames = [dict(seq=i, torque=393, velocity=0, fault_latched=False)
                  for i in range(2048)]
        with self.assertRaisesRegex(ValueError, 'excited'):
            estimate(frames)

    def test_does_not_bridge_gap(self):
        frames = [dict(seq=i*2, torque=i%2, velocity=i%2, fault_latched=False)
                  for i in range(4096)]
        with self.assertRaises(ValueError):
            estimate(frames)

    def test_fault_windows_excluded(self):
        frames = [dict(seq=i, torque=i%2, velocity=i%2, fault_latched=True)
                  for i in range(2048)]
        with self.assertRaises(ValueError):
            estimate(frames)


if __name__ == '__main__':
    unittest.main()
