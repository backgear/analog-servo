"""Keep a missing simulation dependency from becoming a green test run."""
from pathlib import Path
import tempfile
import unittest

from simmy_plot import require_simmy


class SimmyDependencyTests(unittest.TestCase):
    def test_missing_checkout_gives_setup_instructions(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileNotFoundError) as error:
                require_simmy(Path(tmp))
        message = str(error.exception)
        self.assertIn('git clone https://github.com/backgear/simmy.git', message)
        self.assertIn('SIMMY_DIR=', message)

    def test_header_alone_is_not_a_complete_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'include').mkdir()
            (root / 'include/simmy.h').touch()
            with self.assertRaises(FileNotFoundError):
                require_simmy(root)
