"""Nine-case report integration test. Requires an external simmy checkout."""
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


class ReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_simmy(SIMMY)

    def test_nine_case_report_and_provenance_without_plot_dependencies(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            subprocess.run([sys.executable, str(ROOT / 'tools/simmy_report.py'),
                            '--simmy-dir', str(SIMMY), '--out-dir', str(out), '--no-plots'],
                           check=True, capture_output=True, text=True)
            scores = json.loads((out / 'scores.json').read_text())
            self.assertEqual(len(scores), 9)
            self.assertEqual(len({row['name'] for row in scores}), 9)
            self.assertIn('Not settled', (out / 'report.md').read_text())
            self.assertIn('Motion only', (out / 'report.md').read_text())
            targets = None
            for row in scores:
                run = out / row['name']
                frames = json.loads((run / 'frames.json').read_text())
                self.assertEqual(len(frames), 10000)
                current = [f['cmd_position'] for f in frames]
                if targets is None:
                    targets = current
                self.assertEqual(current, targets)
                self.assertEqual(len(row['metrics']['holds']), 2)
                metadata = json.loads((run / 'run.json').read_text())
                self.assertTrue(metadata['simulated'])
                self.assertEqual(len(metadata['source_sha256']['firmware:firmware/src/Control.c']), 64)
                self.assertEqual(metadata['effective_accel_ff_q8'],
                                 18432 if row['feedforward'] == 'velocity-acceleration' else 0)
            self.assertFalse(list(out.glob('*.png')))

    def test_stopping_report_preserves_validation_failures(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            subprocess.run([sys.executable, str(ROOT / 'tools/simmy_stop_report.py'),
                            '--simmy-dir', str(SIMMY), '--out-dir', str(out), '--no-plots'],
                           check=True, capture_output=True, text=True)
            rows = {r['name']: r for r in json.loads((out / 'scores.json').read_text())}
            self.assertEqual(len(rows), 23)
            self.assertEqual(rows['balanced-encoder']['parameters']['feedback'], 'encoder')
            self.assertEqual(rows['balanced-model']['parameters']['feedback'], 'model')
            # Hold 0's load now settles -- the always-live integrator (see
            # test_simmy_plot.py's test_idle_schedule_...) closes the bias a
            # hard deadband used to freeze instead of correct.
            self.assertIsNotNone(rows['idle-schedule']['metrics']['holds'][0]['load_settling_seconds'])
            self.assertIsNone(rows['slow-idle']['metrics']['holds'][-1]['motor_settling_seconds'])
            self.assertGreater(rows['smooth-idle']['metrics']['moving']['motor_rms'], 100)
            text = (out / 'report.md').read_text()
            self.assertIn('not a robust fix', text)
            self.assertIn('Not settled', text)
            self.assertIn('Tail motor span', text)


if __name__ == '__main__':
    unittest.main()
