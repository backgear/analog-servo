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

from simmy_plot import require_simmy

ROOT = Path(__file__).resolve().parent.parent
SIMMY = Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser()


class SimmyPlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_simmy(SIMMY)

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

    def test_sequence_stops_holds_and_reverses_without_reset(self):
        from simmy_metrics import measure
        with tempfile.TemporaryDirectory() as tmp:
            frames = self.run_simulation(Path(tmp), '--scenario', 'sequence', '--tune', 'balanced')
            self.assertEqual(len(frames), 10000)
            self.assertEqual(frames[0]['phase'], 'forward-accel')
            self.assertEqual(frames[3000]['phase'], 'forward-hold')
            self.assertEqual(frames[5000]['phase'], 'reverse-accel')
            self.assertEqual(frames[8000]['phase'], 'final-hold')
            for start, end in ((3000, 5000), (8000, 10000)):
                self.assertEqual(len({f['cmd_position'] for f in frames[start:end]}), 1)
                self.assertTrue(all(f['target_rate'] == 0 for f in frames[start:end]))
            self.assertLess(frames[7999]['cmd_position'], frames[5000]['cmd_position'])
            self.assertEqual(frames[4999]['cmd_position'], frames[5000]['cmd_position'])
            metrics = measure(frames)
            self.assertEqual(len(metrics['holds']), 2)
            self.assertEqual(metrics['saturation_ticks'], 0)
            self.assertLess(metrics['motor_peak'], 100)
            self.assertLess(metrics['load_peak'], 150)
            self.assertEqual(frames, self.run_simulation(Path(tmp), '--scenario', 'sequence',
                                                       '--tune', 'balanced'))

    def test_feedforward_is_causal_and_acceleration_is_separate(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            velocity = self.run_simulation(out, '--tune', 'balanced', '--feedforward', 'velocity')
            prefix = self.run_simulation(out, '--tune', 'balanced', '--feedforward', 'velocity',
                                         '--ticks', '200')
            self.assertEqual(prefix, velocity[:200])
            reference = 0
            previous = 0
            for f in velocity:
                self.assertEqual(f['cmd_position'] - previous, f['pulse_delta'])
                previous = f['cmd_position']
                reference += int((f['pulse_delta'] * 256 - reference) / 8)
                self.assertEqual(reference, f['reference_velocity_q8'])
            accel_zero = self.run_simulation(out, '--tune', 'balanced', '--feedforward',
                                             'velocity-acceleration', '--accel-ff-q8', '0')
            self.assertEqual(velocity, accel_zero)
            accel = self.run_simulation(out, '--tune', 'balanced', '--feedforward',
                                        'velocity-acceleration')
            self.assertNotEqual(velocity, accel)
            self.assertTrue(all(abs(f['torque']) <= 1000 for f in accel))
            self.assertEqual(json.loads((out / 'run.json').read_text())['effective_accel_ff_q8'], 18432)

    def test_encoder_feedback_is_integer_position_difference(self):
        with tempfile.TemporaryDirectory() as tmp:
            frames = self.run_simulation(Path(tmp), '--feedback', 'encoder', '--rate', '-2500')
            previous_position = previous_load = 0
            for f in frames:
                self.assertEqual(f['velocity'], f['position'] - previous_position)
                self.assertEqual(f['load_velocity'], f['load_position'] - previous_load)
                previous_position, previous_load = f['position'], f['load_position']

    def test_idle_schedule_uses_only_counted_pulse_history_and_retains_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            baseline = self.run_simulation(out, '--scenario', 'sequence', '--tune', 'balanced',
                                            '--feedback', 'encoder')
            scheduled = self.run_simulation(out, '--scenario', 'sequence', '--tune', 'balanced',
                                             '--feedback', 'encoder', '--idle-gains', '64', '2048',
                                             '16', '0', '1')
            self.assertEqual([f['cmd_position'] for f in baseline],
                             [f['cmd_position'] for f in scheduled])
            idle_ticks = 0
            for f in scheduled:
                idle_ticks = idle_ticks + 1 if f['pulse_delta'] == 0 else 0
                self.assertEqual(f['idle_active'], idle_ticks >= 50)
            from simmy_metrics import measure
            before, after = measure(baseline), measure(scheduled)
            for old, new in zip(before['holds'], after['holds']):
                self.assertIsNone(old['motor_settling_seconds'])
                # Require a settled suffix through the hold's end (including
                # measure()'s minimum dwell), not a compiler-specific deadline.
                # Floating-point plant differences cross integer encoder edges
                # and change the later stiction/quantization trajectory.
                self.assertIsNotNone(new['motor_settling_seconds'])
                self.assertLess(new['tail_torque_ripple_rms'], old['tail_torque_ripple_rms'])
            # The load is NOT silently declared fixed: it remains off-center.
            self.assertIsNone(after['holds'][0]['load_settling_seconds'])
            identical = self.run_simulation(out, '--scenario', 'sequence', '--tune', 'balanced',
                                             '--feedback', 'encoder', '--idle-gains', '256', '1024',
                                             '16', '1', '0')
            # Reapplying the same gains must not reset/overwrite integral state.
            strip_flag = lambda frames: [{k: v for k, v in f.items() if k != 'idle_active'} for f in frames]
            self.assertEqual(strip_flag(baseline), strip_flag(identical))

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
