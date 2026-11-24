"""Host integration smoke test for output.topology=two_phase_analog: builds
and runs firmware/test/support/simmy_foc_integration_main.c, which links
this branch's REAL Commutation.c/Output.c against simmy's PMSM FOC example
harness (examples/pmsm_foc.h/.c + examples/foc_transforms.h) through a
small documented "stand-in amp" -- see that driver's own top-of-file
comment for the full design and honest limitations.

Same skip-if-absent convention as test_simmy_plot.py: requires SIMMY_DIR
(default ~/simmy) + cc. Run: python3 -m unittest discover -s tools -p
test_simmy_foc_integration.py

PROVISIONAL, more so than test_simmy_plot.py: as of this writing,
examples/pmsm_foc.h/.c and examples/foc_transforms.h do not exist on
simmy's main branch at all -- they are UNCOMMITTED in a temporary simmy
worktree (see this analog-servo branch's own task notes / final report for
that worktree's path). Until that work lands in simmy proper, this test
SKIPS against an ordinary `~/simmy` checkout (the same graceful skip
test_simmy_plot.py already uses for a missing SIMMY_DIR, just triggered by
missing files instead of a missing directory) -- it only actually runs
when SIMMY_DIR is pointed at a checkout/worktree that already has those
three files. This test deliberately does NOT default SIMMY_DIR to that
worktree's own (ephemeral, session-specific) path the way it might be
tempting to -- that path will not exist for anyone else, or even for this
same session later, so baking it in here would make the test silently stop
running for everyone the moment it's cleaned up, with no signal beyond a
skip. Point SIMMY_DIR at it explicitly when running this file today; once
pmsm_foc.h/.c/foc_transforms.h land on simmy's main branch, this test
starts running against the SAME default SIMMY_DIR (~/simmy) every other
tools/test_simmy_*.py file already uses, with no change needed here.
"""
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent
SIMMY = Path(os.environ.get('SIMMY_DIR', '~/simmy')).expanduser()

_REQUIRED_SIMMY_FILES = (
    'include/simmy.h',
    'include/simmy_bldc.h',
    'src/simmy.c',
    'src/simmy_bldc.c',
    'examples/pmsm_foc.h',
    'examples/pmsm_foc.c',
    'examples/foc_transforms.h',
)


def _simmy_has_pmsm_foc():
    return all((SIMMY / name).is_file() for name in _REQUIRED_SIMMY_FILES)


_OUTPUT_RE = re.compile(
    r'finalSpeed=(?P<speed>[-\d.]+) rad/s\s+'
    r'maxAbsIq=(?P<iq>[-\d.]+) A\s+'
    r'maxAbsIdAfterSettling=(?P<id>[-\d.]+) A\s+'
    r'voltageLimited=(?P<vlim>[-\d.]+)%\s+'
    r'anyNonFinite=(?P<nonfinite>\d)')


@unittest.skipUnless(_simmy_has_pmsm_foc(),
                      'SIMMY_DIR has no examples/pmsm_foc.h -- point SIMMY_DIR at a checkout/'
                      'worktree with the provisional PMSM FOC harness (see this file\'s own '
                      'top-of-file comment); not yet true of an ordinary simmy checkout')
class SimmyFocIntegrationTest(unittest.TestCase):
    def build(self, build_dir):
        sources = [ROOT / 'firmware/test/support/simmy_foc_integration_main.c',
                   ROOT / 'firmware/src/Commutation.c',
                   ROOT / 'firmware/src/Output.c',
                   SIMMY / 'src/simmy.c', SIMMY / 'src/simmy_bldc.c']
        for path in sources:
            if not path.is_file():
                raise FileNotFoundError(f'missing {path}')
        binary = Path(build_dir) / 'simmy_foc_integration'
        cmd = [os.environ.get('CC', 'cc'), '-std=gnu11', '-O2', '-Wall', '-Wextra',
               # main.h now errors without an explicit MCU macro (see
               # firmware/include/main.h) -- STM32L152xE, not
               # ANALOG_SERVO_NATIVE_TEST, since Output.c's real two-phase
               # DAC path (OutputSetTwoPhaseCommand) only exists under
               # `#if defined(STM32L152xE)`; sim_foc_mocks/stm32l1xx_hal.h
               # shadows the real L1 HAL header via the -I order below.
               '-D', 'STM32L152xE',
               '-I', str(ROOT / 'firmware/test/support/sim_foc_mocks'),
               '-I', str(ROOT / 'firmware/include'),
               '-I', str(SIMMY / 'include'), '-I', str(SIMMY / 'examples'),
               *map(str, sources), '-lm', '-o', str(binary)]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        return binary

    def test_real_commutation_and_output_drive_the_pmsm_plant(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = self.build(tmp)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0,
                             f'simmy_foc_integration_main failed:\n{result.stdout}\n{result.stderr}')
            self.assertIn('PASS', result.stdout)
            match = _OUTPUT_RE.search(result.stdout)
            self.assertIsNotNone(match, f'unexpected driver output: {result.stdout!r}')
            speed = float(match['speed'])
            max_iq = float(match['iq'])
            max_id = float(match['id'])
            self.assertEqual(match['nonfinite'], '0')
            # Independent of (duplicates, on purpose) the driver's own internal
            # bounds -- see simmy_foc_integration_main.c's final `ok` expression
            # for the calibrated-scenario reasoning behind these specific numbers.
            self.assertGreater(speed, 50.0, 'motor did not spin up in the commanded direction')
            self.assertLess(max_id, 0.75, 'commutation did not stay aligned (Id too large vs. Iq)')
            self.assertLess(max_iq, 8.5, "current loop exceeded the stand-in amp's configured limit")


if __name__ == '__main__':
    unittest.main()
