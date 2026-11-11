import tempfile
from pathlib import Path
import unittest
from bench_report import read_run


class ReportTests(unittest.TestCase):
    def test_recorder_fault_names_are_not_parsed_as_numbers(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'monitor.csv').write_text(
                'seq,position,velocity,physical_torque,reference_position,ramped_velocity,enabled,fault\n'
                '1,64,0,12,64,0,1,none\n')
            run = read_run(directory)
            self.assertEqual(run['rows'][0]['fault'], 'none')
            self.assertEqual(run['rows'][0]['position'], 64)


if __name__ == '__main__':
    unittest.main()
