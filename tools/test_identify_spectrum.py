"""Host-only tests: python3 -m unittest discover -s tools -p test_identify_spectrum.py."""
import unittest
import numpy as np
from identify_spectrum import contiguous_runs, estimate


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
