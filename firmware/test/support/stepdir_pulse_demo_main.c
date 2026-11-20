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
 *                            [--feedforward none|raw|smoothed] [--ff-shift N]
 * --ramp-seconds >0 ramps the pulse rate linearly from 0 to --rate over
 * that many seconds, then holds --rate for the remaining ticks (0, the
 * default, means --rate from tick 0 -- no ramp).
 *
 * --feedforward selects what's fed into Control.c's existing
 * ControlTickWithReferenceQ8() feedforward path (already used by Mode
 * B/C, never reached from Mode A today -- see test_stepdir_control.c's
 * PROTOTYPE section): "none" (default, current production Mode A: zero
 * feedforward), "raw" (this tick's pulse-train delta straight in,
 * unsmoothed), or "smoothed" (one-pole-smoothed at --ff-shift, default 3
 * -- the calibrated sweet spot in test_stepdir_control.c).
 *
 * --plant selects the toy plant (see test_stepdir_control.c's
 * "PROTOTYPE, realistic plant" section for the full story): "idealized"
 * (default -- the zero-lag `velocity += torque/16` plant used throughout
 * this suite, with the gains that section also uses) or "realistic"
 * (ToyPlant.h's inertia/amp-lag/friction model, with the gains hand-
 * tuned specifically for it -- NOT the idealized plant's gains, which go
 * unstable against this plant).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Control.h"
#include "PulseTrain.h"
#include "ToyPlant.h"

static double NextDoubleArg(int argc, char **argv, int *i) {
  if (*i + 1 >= argc) { fprintf(stderr, "%s: missing value\n", argv[*i]); exit(1); }
  return atof(argv[++*i]);
}

typedef enum { FF_NONE, FF_RAW, FF_SMOOTHED } FeedforwardMode;
typedef enum { PLANT_IDEALIZED, PLANT_REALISTIC } PlantChoice;

/* See test_stepdir_control.c's "PROTOTYPE, realistic plant" section --
 * hand-tuned via a step-response grid search specifically for
 * kRealisticPlant below, NOT the idealized plant's gains. */
static const ToyPlantParams kRealisticPlant = {.inertia = 64.0, .ampAlpha = 0.6, .friction = 0.05};
static const ControlGains kRealisticPlantGains = {
  .posGainQ8 = 8, .velGainKpQ8 = 512, .velGainKiQ8 = 1,
  .velocityClamp = 200000, .torqueClampMin = -1000, .torqueClampMax = 1000,
};

int main(int argc, char **argv) {
  double rateEdgesPerSec = -1.0, jitterFraction = 0.0, t0Seconds = 0.0, rampSeconds = 0.0;
  int    ticks = -1;
  int32_t filterShift = 0;
  FeedforwardMode ffMode = FF_NONE;
  PlantChoice plantChoice = PLANT_IDEALIZED;
  int32_t ffShift = 3;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--rate")) rateEdgesPerSec = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--jitter")) jitterFraction = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--ticks")) ticks = (int)NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--t0")) t0Seconds = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--ramp-seconds")) rampSeconds = NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--filter-shift")) filterShift = (int32_t)NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--ff-shift")) ffShift = (int32_t)NextDoubleArg(argc, argv, &i);
    else if (!strcmp(argv[i], "--feedforward")) {
      if (++i >= argc) { fprintf(stderr, "--feedforward: missing value\n"); return 1; }
      if (!strcmp(argv[i], "none")) ffMode = FF_NONE;
      else if (!strcmp(argv[i], "raw")) ffMode = FF_RAW;
      else if (!strcmp(argv[i], "smoothed")) ffMode = FF_SMOOTHED;
      else { fprintf(stderr, "--feedforward: must be none|raw|smoothed\n"); return 1; }
    }
    else if (!strcmp(argv[i], "--plant")) {
      if (++i >= argc) { fprintf(stderr, "--plant: missing value\n"); return 1; }
      if (!strcmp(argv[i], "idealized")) plantChoice = PLANT_IDEALIZED;
      else if (!strcmp(argv[i], "realistic")) plantChoice = PLANT_REALISTIC;
      else { fprintf(stderr, "--plant: must be idealized|realistic\n"); return 1; }
    }
    else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 1; }
  }
  if (rateEdgesPerSec < 0.0 || ticks < 0) {
    fprintf(stderr, "usage: %s --rate N --ticks N [--jitter F] [--t0 S] "
                    "[--ramp-seconds S] [--filter-shift N] "
                    "[--feedforward none|raw|smoothed] [--ff-shift N] "
                    "[--plant idealized|realistic]\n", argv[0]);
    return 1;
  }
  int rampTicks = (int)(rampSeconds * 1000.0);
  double accelRatePerSec2 = rampSeconds > 0.0 ? rateEdgesPerSec / rampSeconds : 0.0;

  /* Idealized plant's gains -- same representative tune test_stepdir_control.c
   * uses throughout, not a claim of correctness for any real plant. */
  ControlGains idealizedGains = {
    .posGainQ8 = 256,
    .velGainKpQ8 = 512,
    .velGainKiQ8 = 26,
    .velocityClamp = 200000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
    .velocityFilterShift = filterShift,
  };
  ControlInit(plantChoice == PLANT_REALISTIC ? &kRealisticPlantGains : &idealizedGains);

  PulseTrainState pt;
  PulseTrainInit(&pt, 12345u);   /* same fixed seed as the native test -- deterministic demo */
  ToyPlantState plant;
  ToyPlantInit(&plant);

  int32_t posTargetAccum = 0, pos = 0, vel = 0;
  int64_t filteredDeltaQ8 = 0;

  printf("[\n");
  for (int i = 0; i < ticks; i++) {
    double rate = (i < rampTicks) ? (accelRatePerSec2 * i / 1000.0) : rateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, 1000.0, jitterFraction);
    posTargetAccum += delta;
    int32_t posActual = plantChoice == PLANT_REALISTIC ? ToyPlantPositionCounts(&plant) : pos;
    int32_t velActual = plantChoice == PLANT_REALISTIC ? ToyPlantVelocityCounts(&plant) : vel;

    int32_t torque;
    if (ffMode == FF_NONE) {
      torque = ControlTick(posTargetAccum, posActual, velActual, 32000);
    } else {
      int64_t referenceVelocityQ8 = (int64_t)delta * 256;
      if (ffMode == FF_SMOOTHED) {
        filteredDeltaQ8 += (referenceVelocityQ8 - filteredDeltaQ8) >> ffShift;
        referenceVelocityQ8 = filteredDeltaQ8;
      }
      torque = ControlTickWithReferenceQ8(posTargetAccum, posActual, velActual, referenceVelocityQ8, 32000);
    }

    printf("%s{\"t\": %.6f, \"position\": %d, \"cmd_position\": %d, "
           "\"velocity\": %d, \"cmd_velocity\": %d, \"torque\": %d, "
           "\"fault_latched\": false}\n",
           i == 0 ? "" : ",", t0Seconds + i / 1000.0, posActual, posTargetAccum,
           velActual, delta, torque);

    if (plantChoice == PLANT_REALISTIC) {
      ToyPlantStep(&plant, &kRealisticPlant, torque);
    } else {
      vel += torque / 16;
      pos += vel;
    }
  }
  printf("]\n");
  return 0;
}
