#ifndef PULSE_TRAIN_H
#define PULSE_TRAIN_H

/*
 * Host/native-only synthetic step/dir pulse-train model, shared by
 * test_stepdir_control.c and stepdir_pulse_demo_main.c. Models the
 * QUANTIZED EDGE-COUNT result StepDirGetDeltaCounts() would report each
 * tick for an idealized periodic pulse train -- NOT a claim about real
 * electrical edge timing. StepDir.c's actual TIM2 edge-counting is
 * HAL-dependent and out of reach for a native test either way; see
 * StepDir.h's own "UNVERIFIED WITHOUT THE DATASHEET" note for what's
 * genuinely unverified there. Same spirit as test_control.c's RunPlant()
 * disclaimer: this is just enough fidelity to exercise the control loop's
 * response to quantized/jittered input, not a plant/hardware model.
 *
 * Double-precision math throughout -- this is host-only test/demo code,
 * not firmware, so none of the embedded fixed-point constraints apply
 * (env:native already links -lm for exactly this reason).
 */

#include <math.h>
#include <stdint.h>

typedef struct {
  double   phase;     /* accumulated fractional edge count, kept in [0, 1) between ticks */
  uint32_t rngState;  /* xorshift32 state */
} PulseTrainState;

static inline void PulseTrainInit(PulseTrainState *pt, uint32_t seed) {
  pt->phase = 0.0;
  pt->rngState = seed ? seed : 0x9e3779b9u;   /* 0 would stay 0 forever under xorshift */
}

/* xorshift32 -- deterministic and fast, not cryptographic; fine for
 * reproducible test/demo jitter. */
static inline uint32_t PulseTrainNextRng(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

/* Pseudorandom double, uniform in [-1, 1). */
static inline double PulseTrainUniformSigned(uint32_t *state) {
  return (double)PulseTrainNextRng(state) / 4294967296.0 * 2.0 - 1.0;
}

/* One tick's worth of simulated step/dir edge count for a pulse train
 * nominally running at rateEdgesPerSec, sampled at tickHz (1000 for the
 * real 1kHz control tick). jitterFraction (>=0, 0 disables) bounds
 * short-term timing noise as a fraction of one tick's nominal increment.
 *
 * The perturbed increment is clamped to be non-negative: this models
 * edges arriving slightly early/late, never edges un-happening or the
 * count running backward. A real direction reversal is a separate,
 * deliberate DIR-pin event (StepDirDirIsr()) -- "jitter" here must not be
 * able to fake one, or this would be testing something that can't
 * actually happen on the wire.
 *
 * Returns the integer delta count for this tick, matching
 * StepDirGetDeltaCounts()'s per-tick contract. */
static inline int32_t PulseTrainTickDelta(PulseTrainState *pt, double rateEdgesPerSec,
                                           double tickHz, double jitterFraction) {
  double nominalIncrement = rateEdgesPerSec / tickHz;
  double increment = nominalIncrement;
  if (jitterFraction > 0.0) {
    increment += nominalIncrement * jitterFraction * PulseTrainUniformSigned(&pt->rngState);
    if (increment < 0.0) increment = 0.0;
  }
  pt->phase += increment;
  double wholeEdges = floor(pt->phase);
  pt->phase -= wholeEdges;
  return (int32_t)wholeEdges;
}

#endif /* PULSE_TRAIN_H */
