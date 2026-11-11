import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from pool_excitation import main
from identify_spectrum import contiguous_runs


class PoolTests(unittest.TestCase):
    def run_pool(self, second_amplitude=2000, second_complete=True, second_start=120):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index, start in enumerate([100, second_start]):
                path = str(Path(directory) / f'{index}.json')
                rows = [dict(seq=start+i, excitation=True, fault_latched=False) for i in range(8)]
                Path(path).write_text(json.dumps(rows))
                meta = dict(amplitude=2000 if index == 0 else second_amplitude,
                            duration_ticks=8, completed=True if index == 0 else second_complete)
                Path(path + '.meta.json').write_text(json.dumps(meta))
                paths.append(path)
            out = str(Path(directory) / 'pooled.json')
            with patch('sys.argv', ['pool_excitation', *paths, '--out', out]):
                main()
            return json.loads(Path(out).read_text())

    def test_retains_gap(self):
        frames = self.run_pool()
        self.assertEqual([len(r) for r in contiguous_runs(frames)], [8, 8])

    def test_rejects_parameter_mismatch(self):
        with self.assertRaises(ValueError):
            self.run_pool(second_amplitude=4000)

    def test_rejects_incomplete(self):
        with self.assertRaises(ValueError):
            self.run_pool(second_complete=False)

    def test_rejects_counter_restart(self):
        with self.assertRaises(ValueError):
            self.run_pool(second_start=0)


if __name__ == '__main__':
    unittest.main()
