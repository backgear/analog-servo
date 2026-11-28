"""Host integration test for Control.c's soft velocity-error deadband
(ConfigStore's CONFIG_DEFAULT_VELOCITY_ERROR_DEADBAND_COUNTS /
Control.h's ControlGains.velocityErrorDeadbandCounts). Requires SIMMY_DIR
(default ~/simmy) + cc. Run:
    python3 -m unittest discover -s tools -p test_simmy_soft_velocity_deadband.py
Tests synthetic behavior against the real firmware Control.c, not hardware.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from simmy_plot import require_simmy
from simmy_metrics import measure

ROOT = Path(__file__).resolve().parent.parent
SIMMY = Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser()


class SoftVelocityDeadbandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_simmy(SIMMY)

    def run_simulation(self, out, deadband):
        subprocess.run([sys.executable, str(ROOT / 'tools/simmy_plot.py'),
                        '--simmy-dir', str(SIMMY), '--out-dir', str(out), '--no-plot',
                        '--tune', 'balanced', '--scenario', 'sequence', '--feedback', 'encoder',
                        '--velocity-deadband', str(deadband)], check=True,
                       capture_output=True, text=True)
        return json.loads((out / 'frames.json').read_text())

    def test_widening_deadband_quiets_holds_without_degrading_tracking(self):
        """The deadband only softens P's reaction to feedback quantization noise
        (see Control.c's VelocityTickQ8) -- it must not change the commanded
        trajectory, and I always sees the full error, so active tracking
        should not measurably suffer for a quieter standstill."""
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            raw = self.run_simulation(out, 0)
            softened = self.run_simulation(out, 3)
            self.assertEqual([f['cmd_position'] for f in raw],
                             [f['cmd_position'] for f in softened])
            raw_metrics, softened_metrics = measure(raw), measure(softened)
            self.assertEqual(len(raw_metrics['holds']), 2)
            for raw_hold, softened_hold in zip(raw_metrics['holds'], softened_metrics['holds']):
                self.assertLess(softened_hold['tail_torque_ripple_rms'],
                                 raw_hold['tail_torque_ripple_rms'])
            # Not just a global detune hiding as "quieter": tracking while
            # actually moving stays within 10% either way.
            self.assertLess(softened_metrics['moving']['torque_rms'],
                             raw_metrics['moving']['torque_rms'] * 1.1)


if __name__ == '__main__':
    unittest.main()
