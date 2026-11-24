"""Host integration smoke test for the OTHER half of output.topology=
two_phase_analog: the open-loop Hall-driven `commutation learn` maneuver
(main.c's AppCommutationLearn()/AppControlTick()'s sCommLearningActive
state machine), built and run against a real spinning simmy plant. See
firmware/test/support/simmy_commutation_learn_integration_main.c's own
top-of-file comment for the full design, what it proves/doesn't, and a
real Fault.c interaction this test surfaced.

Same skip-if-absent convention as test_simmy_plot.py/
test_simmy_foc_integration.py: requires SIMMY_DIR (default ~/simmy) + cc,
and (like test_simmy_foc_integration.py) the provisional PMSM FOC files
this test's Clarke+Park reconstruction borrows examples/foc_transforms.h's
FocPark()/FOC_PI from -- NOT examples/pmsm_foc.h's current-loop bundle,
which this test doesn't need, but gated on the SAME required-files check
as that other test anyway: all three files live in the same not-yet-merged
simmy worktree, so "is the provisional work available" is one question,
not two. See that file's own PROVISIONAL-SIMMY_DIR-path comment for how to
actually run this today (point SIMMY_DIR at that worktree -- it is NOT
the default here on purpose, see this file's own skip message).

Run: python3 -m unittest discover -s tools -p test_simmy_commutation_learn_integration.py

Unlike test_simmy_foc_integration.py's driver, this one links the REAL
main.c itself (the first native test to do so -- see the driver's own
top comment for why that needs a two-step compile: main.c compiled
separately with `-Dmain=UnusedFirmwareMain` so it doesn't collide with
this driver's own main(), then linked together with `-Wl,--gc-sections`
so main.c's own real main()/SystemClock_Config() -- never called, only
compiled -- don't need a full STM32 HAL behind them at link time, just
enough to type-check at compile time (see
firmware/test/support/sim_main_mocks/stm32l1xx_hal.h).
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


_CFLAGS = ['-std=gnu11', '-O0', '-g', '-Wall', '-Wextra',
           '-ffunction-sections', '-fdata-sections',
           '-D', 'SYSCLK_HZ=32000000', '-D', 'ANALOG_SERVO_TWO_PHASE_DRIVE',
           # main.h now errors without an explicit MCU macro (see
           # firmware/include/main.h) -- STM32L152xE, not
           # ANALOG_SERVO_NATIVE_TEST, since Output.c's real two-phase DAC
           # path (OutputSetTwoPhaseCommand) only exists under
           # `#if defined(STM32L152xE)`; sim_main_mocks/stm32l1xx_hal.h
           # shadows the real L1 HAL header via the -I order below.
           '-D', 'STM32L152xE']

_OUTPUT_RE = re.compile(
    r'ticks=(?P<ticks>\d+)\s+stillBusy=(?P<busy>\d)\s+failed=(?P<failed>\d)\s+'
    r'learnedOffsetDeg=(?P<learned>-?\d+)\s+trueOffsetDeg=(?P<truth>[-\d.]+)\s+'
    r'errorDeg=(?P<error>[-\d.]+)\s+finalEncoderCounts=(?P<counts>-?\d+)\s+'
    r'anyNonFinite=(?P<nonfinite>\d)')


@unittest.skipUnless(_simmy_has_pmsm_foc(),
                      'SIMMY_DIR has no examples/pmsm_foc.h -- point SIMMY_DIR at a checkout/'
                      'worktree with the provisional PMSM FOC harness (see this file\'s own '
                      'top-of-file comment); not yet true of an ordinary simmy checkout')
class SimmyCommutationLearnIntegrationTest(unittest.TestCase):
    def build(self, build_dir):
        cc = [os.environ.get('CC', 'cc')]
        include_dirs = [
            ROOT / 'firmware/test/support/sim_main_mocks',
            ROOT / 'firmware/test/mocks',
            ROOT / 'firmware/include',
            SIMMY / 'include',
            SIMMY / 'examples',
        ]
        cflags = list(_CFLAGS)
        for d in include_dirs:
            cflags += ['-I', str(d)]

        main_c = ROOT / 'firmware/src/main.c'
        other_sources = [
            ROOT / 'firmware/test/support/simmy_commutation_learn_integration_main.c',
            ROOT / 'firmware/src/Hall.c',
            ROOT / 'firmware/src/Commutation.c',
            ROOT / 'firmware/src/Output.c',
            ROOT / 'firmware/src/Fault.c',
            ROOT / 'firmware/src/Control.c',
            # Autotune.c: this driver's own main() calls AutotuneInit()
            # directly as part of its init sequence (mirroring main.c's real
            # one), so it's linked regardless of ANALOG_SERVO_HAS_AUTOTUNE.
            ROOT / 'firmware/src/Autotune.c',
            ROOT / 'firmware/src/Signal.c',
            ROOT / 'firmware/src/Polarity.c',
            # RLS.c/GainGlide.c deliberately NOT linked here (unlike
            # Autotune.c above): this driver doesn't define
            # ANALOG_SERVO_HAS_AUTOTUNE (see _CFLAGS above) and never calls
            # either directly itself, so main.c's own #ifdef compiles out
            # every call site into them (same convention as
            # ANALOG_SERVO_TWO_PHASE_DRIVE) with nothing left referencing
            # them.
            SIMMY / 'src/simmy.c',
            SIMMY / 'src/simmy_bldc.c',
        ]
        for path in [main_c, *other_sources]:
            if not path.is_file():
                raise FileNotFoundError(f'missing {path}')

        objects = []
        # main.c gets its own `main` renamed away so it doesn't collide with this
        # driver's real entry point -- see the driver's own top-of-file comment.
        main_obj = Path(build_dir) / 'main.o'
        subprocess.run(cc + cflags + ['-Dmain=UnusedFirmwareMain', '-c', str(main_c),
                                      '-o', str(main_obj)],
                       check=True, capture_output=True, text=True)
        objects.append(main_obj)
        for src in other_sources:
            obj = Path(build_dir) / (src.stem + '.o')
            subprocess.run(cc + cflags + ['-c', str(src), '-o', str(obj)],
                           check=True, capture_output=True, text=True)
            objects.append(obj)

        binary = Path(build_dir) / 'simmy_commutation_learn_integration'
        subprocess.run(cc + ['-Wl,--gc-sections', *map(str, objects), '-lm', '-o', str(binary)],
                       check=True, capture_output=True, text=True)
        return binary

    def test_real_commutation_learn_state_machine_recovers_injected_offset(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = self.build(tmp)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0,
                             f'simmy_commutation_learn_integration_main failed:\n'
                             f'{result.stdout}\n{result.stderr}')
            self.assertIn('PASS', result.stdout)
            match = _OUTPUT_RE.search(result.stdout)
            self.assertIsNotNone(match, f'unexpected driver output: {result.stdout!r}')
            self.assertEqual(match['busy'], '0', 'learn attempt never finished')
            self.assertEqual(match['failed'], '0', 'learn attempt finished but rejected '
                                                    '(too few Hall transitions observed)')
            self.assertEqual(match['nonfinite'], '0')
            error_deg = float(match['error'])
            # Independent of (duplicates, on purpose) the driver's own internal
            # bound -- see simmy_commutation_learn_integration_main.c's final `ok`
            # expression for why 15 degrees, not a tighter number: Commutation.c's
            # 60-degree-sector coarseness is the real error floor here, not
            # something to tighten arbitrarily.
            self.assertLessEqual(abs(error_deg), 15.0,
                                 f'learned offset off by {error_deg} degrees from the '
                                 f'injected truth ({match["truth"]} degrees)')


if __name__ == '__main__':
    unittest.main()
