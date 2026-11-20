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
 * host. See tools/stepdir_pulse_demo.py, which compiles and runs this.
 *
 * Usage: stepdir_pulse_demo <rateEdgesPerSec> <jitterFraction> <ticks> <t0Seconds>
 */
#include <stdio.h>
#include <stdlib.h>
#include "Control.h"
#include "PulseTrain.h"

int main(int argc, char **argv) {
  if (argc != 5) {
    fprintf(stderr, "usage: %s <rateEdgesPerSec> <jitterFraction> <ticks> <t0Seconds>\n", argv[0]);
    return 1;
  }
  double rateEdgesPerSec = atof(argv[1]);
  double jitterFraction  = atof(argv[2]);
  int    ticks           = atoi(argv[3]);
  double t0Seconds        = atof(argv[4]);

  /* Same tune test_stepdir_control.c uses -- a representative, already-
   * reasonable tune, not a claim of correctness for any real plant. */
  ControlGains gains = {
    .posGainQ8 = 256,
    .velGainKpQ8 = 512,
    .velGainKiQ8 = 26,
    .velocityClamp = 100000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
  };
  ControlInit(&gains);

  PulseTrainState pt;
  PulseTrainInit(&pt, 12345u);   /* same fixed seed as the native test -- deterministic demo */

  int32_t posTargetAccum = 0, pos = 0, vel = 0;

  printf("[\n");
  for (int i = 0; i < ticks; i++) {
    int32_t delta = PulseTrainTickDelta(&pt, rateEdgesPerSec, 1000.0, jitterFraction);
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
