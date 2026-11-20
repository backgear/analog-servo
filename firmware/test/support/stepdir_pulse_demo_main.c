/*
 * Host-only driver: runs Control.c's real P->PI cascade against a
 * synthetic step/dir pulse train (PulseTrain.h), same model and same
 * "reasonable" gains as test_stepdir_control.c, and prints one JSON
 * object per tick to stdout -- one line each, in exactly
 * tools/monitor_frame.py's decoded-frame shape, so tools/plot_telemetry.py
 * can render it completely unchanged.
 *
 * This is a synthetic/simulated run, not a bench capture: there is no
 * real StepDir.c/TIM2 edge-counting and no real electrical pulse jitter
 * here, only the quantized edge-count model PulseTrain.h documents. Never
 * present output from this program as bench-validated.
 *
 * Built directly with the host cc, not through PlatformIO -- Control.c has
 * no HAL dependency, the same reason pio test -e native can run it on the
 * host. See tools/stepdir_pulse_demo.py, which compiles, runs, and plots
 * this (that script is also the one-command way to regenerate every
 * assets/stepdir-*.png after a Control.c change).
 *
 * Usage: stepdir_pulse_demo --rate N --ticks N [--jitter F] [--t0 S]
 *                            [--ramp-seconds S] [--filter-shift N]
 * --ramp-seconds >0 ramps the pulse rate linearly from 0 to --rate over
 * that many seconds, then holds --rate for the remaining ticks (0, the
 * default, means --rate from tick 0 -- no ramp).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Control.h"
#include "PulseTrain.h"

static double NextDoubleArg(int argc, char **argv, int *i) {
  if (*i + 1 >= argc) { fprintf(stderr, "%s: missing value\n", argv[*i]); exit(1); }
  return atof(argv[++*i]);
}

int main(int argc, char **argv) {
  double rateEdgesPerSec = -1.0, jitterFraction = 0.0, t0Seconds = 0.0, rampSeconds = 0.0;
  int    ticks = -1;
  int32_t filterShift = 0;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--rate")) rateEdgesPerSec = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--jitter")) jitterFraction = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--ticks")) ticks = (int)NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--t0")) t0Seconds = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--ramp-seconds")) rampSeconds = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--filter-shift")) filterShift = (int32_t)NextDoubleArg(argc, argv, &i);
    else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 1; }
  }
  if (rateEdgesPerSec < 0.0 || ticks < 0) {
    fprintf(stderr, "usage: %s --rate N --ticks N [--jitter F] [--t0 S] "
                    "[--ramp-seconds S] [--filter-shift N]\n", argv[0]);
    return 1;
  }
  int rampTicks = (int)(rampSeconds * 1000.0);
  double accelRatePerSec2 = rampSeconds > 0.0 ? rateEdgesPerSec / rampSeconds : 0.0;

  /* Same tune test_stepdir_control.c uses -- a representative, already-
   * reasonable tune, not a claim of correctness for any real plant. */
  ControlGains gains = {
    .posGainQ8 = 256,
    .velGainKpQ8 = 512,
    .velGainKiQ8 = 26,
    .velocityClamp = 200000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
    .velocityFilterShift = filterShift,
  };
  ControlInit(&gains);

  PulseTrainState pt;
  PulseTrainInit(&pt, 12345u);   /* same fixed seed as the native test -- deterministic demo */

  int32_t posTargetAccum = 0, pos = 0, vel = 0;

  printf("[\n");
  for (int i = 0; i < ticks; i++) {
    double rate = (i < rampTicks) ? (accelRatePerSec2 * i / 1000.0) : rateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, 1000.0, jitterFraction);
    posTargetAccum += delta;
    int32_t torque = ControlTick(posTargetAccum, pos, vel, 32000);

    printf("%s{\"t\": %.6f, \"position\": %d, \"cmd_position\": %d, "
           "\"velocity\": %d, \"cmd_velocity\": %d, \"torque\": %d, "
           "\"fault_latched\": false}\n",
           i == 0 ? "" : ",", t0Seconds + i / 1000.0, pos, posTargetAccum,
           vel, delta, torque);

    vel += torque / 16;
    pos += vel;
  }
  printf("]\n");
  return 0;
}
