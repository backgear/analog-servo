"""Host integration smoke tests; requires SIMMY_DIR (default ~/simmy) + cc.
Run: python3 -m unittest discover -s tools -p test_simmy_plot.py
These test synthetic behavior, not hardware performance.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent
SIMMY = Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser()


@unittest.skipUnless((SIMMY / 'include/simmy.h').is_file(), 'external simmy checkout not found')
class SimmyPlotTests(unittest.TestCase):
    def run_simulation(self, out, *extra):
        subprocess.run([sys.executable, str(ROOT / 'tools/simmy_plot.py'),
                        '--simmy-dir', str(SIMMY), '--out-dir', str(out),
                        '--no-plot', *extra], check=True, capture_output=True, text=True)
        return json.loads((out / 'frames.json').read_text())

    def test_calibrated_scenario_and_determinism(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            frames = self.run_simulation(out, '--tune', 'baseline')
            self.assertEqual(len(frames), 2000)
            self.assertEqual(frames[0]['t'], 0)
            self.assertEqual(frames[-1]['t'], 1.999)
            errors = [abs(f['cmd_position'] - f['position']) for f in frames]
            # Same synthetic model/gains and bounds as simmy's integration example.
            self.assertLessEqual(max(errors[:1000]), 450)
            self.assertLessEqual(max(errors[1000:]), 300)
            self.assertLessEqual(sum(abs(f['torque']) >= 999 for f in frames), 20)
            self.assertTrue(all(-1000 <= f['torque'] <= 1000 for f in frames))
            self.assertEqual(frames, self.run_simulation(out, '--tune', 'baseline'))
            self.assertTrue(json.loads((out / 'run.json').read_text())['simulated'])

    def test_optimized_tune_improves_motor_and_load_tracking(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            baseline = self.run_simulation(out, '--tune', 'baseline')
            tuned = self.run_simulation(out)
            for key in ('position', 'load_position'):
                baseline_mse = sum((f['cmd_position'] - f[key])**2 for f in baseline) / len(baseline)
                tuned_mse = sum((f['cmd_position'] - f[key])**2 for f in tuned) / len(tuned)
                self.assertLess(tuned_mse, baseline_mse / 9)  # >3x RMS improvement
            self.assertLessEqual(max(abs(f['cmd_position'] - f['position']) for f in tuned), 25)
            self.assertLessEqual(max(abs(f['cmd_position'] - f['load_position']) for f in tuned), 75)
            self.assertTrue(all(abs(f['torque']) < 999 for f in tuned))
            metadata = json.loads((out / 'run.json').read_text())
            self.assertEqual((metadata['pos_kp'], metadata['vel_kp'], metadata['vel_ki']),
                             (256, 3072, 32))
            self.assertEqual(tuned, self.run_simulation(out))

    def test_optimized_tune_longer_runs_and_other_rates(self):
        with tempfile.TemporaryDirectory() as tmp:
            for rate in (2500, -2500, 1000, 5000):
                with self.subTest(rate=rate):
                    frames = self.run_simulation(Path(tmp), '--ticks', '8000', '--rate', str(rate))
                    self.assertLessEqual(max(abs(f['cmd_position'] - f['position']) for f in frames), 35)
                    self.assertLessEqual(max(abs(f['cmd_position'] - f['load_position']) for f in frames), 80)
                    self.assertTrue(all(abs(f['torque']) < 999 for f in frames))

    def test_balanced_tune_reduces_torque_activity(self):
        from simmy_metrics import measure
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            tracking = measure(self.run_simulation(out))
            balanced = measure(self.run_simulation(out, '--tune', 'balanced'))
            self.assertLess(balanced['torque_delta_rms'], tracking['torque_delta_rms'] / 2)
            self.assertLess(balanced['torque_rms'], tracking['torque_rms'])
            self.assertLess(balanced['motor_rms'], 15)
            self.assertLess(balanced['load_rms'], 25)
            self.assertEqual(balanced['saturation_ticks'], 0)
            self.assertEqual(json.loads((out / 'run.json').read_text())['filter_shift'], 1)

    def test_negative_rate(self):
        with tempfile.TemporaryDirectory() as tmp:
            frames = self.run_simulation(Path(tmp), '--rate', '-2500')
            self.assertLess(frames[-1]['cmd_position'], 0)
            self.assertLess(frames[-1]['position'], 0)

    def test_invalid_numeric_input(self):
        result = subprocess.run([sys.executable, str(ROOT / 'tools/simmy_plot.py'),
                                 '--rate', 'nan', '--no-plot'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('must be finite', result.stderr)


if __name__ == '__main__':
    unittest.main()
