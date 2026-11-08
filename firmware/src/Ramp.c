/*
 * Mode B accel/decel ramp generator. See Ramp.h.
 *
 * RAMP_DENOM is the core clock rate in Hz -- dtCycles (from main.c's DWT
 * cycle counter) is only convertible to seconds against the actual SYSCLK,
 * which differs per env (72 MHz F103 vs. 32 MHz L152, see platformio.ini's
 * -D SYSCLK_HZ and main.c's SystemClock_Config). A bare numeric literal
 * here instead of SYSCLK_HZ would silently mis-scale every Mode B
 * accel/decel on any env whose clock doesn't match it (same coupling
 * app/'s Ramps.c has to its own DWT-cycle clock rate).
 */
#include "Ramp.h"

#ifndef SYSCLK_HZ
#error "SYSCLK_HZ must be defined by the env's build_flags (see platformio.ini) -- RAMP_DENOM depends on it"
#endif
#define RAMP_DENOM SYSCLK_HZ

void RampInit(RampState *r) {
  r->currentVelocity = 0;
  r->targetVelocity = 0;
  r->accelCountsPerTickPerSec = 1000;   /* conservative tier-1 default, tune on the bench */
  r->error = 0;
  r->jerkCountsPerTickPerSec2 = 0;
  r->smoothVelocityQ16 = 0;
  r->smoothAccelerationQ16 = 0;
}

void RampSetTarget(RampState *r, int32_t targetVelocityCountsPerTick) {
  r->targetVelocity = targetVelocityCountsPerTick;
}

void RampSetAcceleration(RampState *r, int32_t accelCountsPerTickPerSec) {
  r->accelCountsPerTickPerSec = accelCountsPerTickPerSec;
}

/* Shared by RampTick() and RampPeekAccelPerTick() below -- the one place
 * that actually knows RAMP_DENOM's scaling. errorInOut is updated in
 * place on a real tick (RampTick()'s own r->error), or a throwaway local
 * copy for a non-mutating peek -- either way the caller decides what
 * "advancing" means, this function just does the math once. */
static int32_t ScaleAccelToTick(int32_t accelCountsPerTickPerSec, uint32_t dtCycles, int32_t *errorInOut) {
  int64_t numerator = (int64_t)accelCountsPerTickPerSec * (int64_t)dtCycles + *errorInOut;
  int32_t step = (int32_t)(numerator / RAMP_DENOM);
  *errorInOut = (int32_t)(numerator % RAMP_DENOM);
  if (step < 0) step = -step;   /* accelCountsPerTickPerSec is a magnitude; direction
                                    is the caller's problem, not this scaling step's */
  return step;
}

int RampShouldDecelerateNow(const RampState *r, uint32_t dtCycles,
                             int32_t velocityMagnitude, int32_t remainingDistanceMagnitude) {
  int64_t v2 = (int64_t)velocityMagnitude * (int64_t)velocityMagnitude;
  int64_t lhs = v2 * (int64_t)RAMP_DENOM;
  int64_t rhs = 2LL * (int64_t)r->accelCountsPerTickPerSec * (int64_t)dtCycles *
                (int64_t)remainingDistanceMagnitude;
  return lhs >= rhs;
}

int32_t RampPeekAccelPerTick(const RampState *r, uint32_t dtCycles) {
  int32_t errorCopy = r->error;   /* peek -- never written back, so this can't desync
                                       the real accumulator a subsequent real RampTick()
                                       depends on */
  return ScaleAccelToTick(r->accelCountsPerTickPerSec, dtCycles, &errorCopy);
}

int32_t RampTick(RampState *r, uint32_t dtCycles) {
  r->smoothVelocityQ16 = (int64_t)r->currentVelocity * 65536;
  r->smoothAccelerationQ16 = 0;
  if (r->currentVelocity == r->targetVelocity) return r->currentVelocity;

  int32_t step = ScaleAccelToTick(r->accelCountsPerTickPerSec, dtCycles, &r->error);

  if (r->currentVelocity < r->targetVelocity) {
    r->currentVelocity += step;
    if (r->currentVelocity > r->targetVelocity) r->currentVelocity = r->targetVelocity;
  } else {
    r->currentVelocity -= step;
    if (r->currentVelocity < r->targetVelocity) r->currentVelocity = r->targetVelocity;
  }
  return r->currentVelocity;
}

void RampSetJerk(RampState *r, int32_t jerk) {
  r->jerkCountsPerTickPerSec2 = jerk;
}

static int64_t Abs64(int64_t v) { return v < 0 ? -v : v; }
static int64_t Clamp64(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : v > hi ? hi : v;
}
static int32_t RoundedVelocity(int64_t q16) {
  return (int32_t)(q16 >= 0 ? (q16+32768)/65536 : -((-q16+32768)/65536));
}

/* Causal jerk-limited velocity jog. PositionMove continues using RampTick:
 * its trapezoidal stopping-distance calculation must not silently inherit
 * a different braking profile. No future position samples are used. */
int32_t RampSmoothTick(RampState *r, uint32_t dtCycles) {
  if (r->jerkCountsPerTickPerSec2 <= 0 || dtCycles == 0) return RampTick(r, dtCycles);
  if (RoundedVelocity(r->smoothVelocityQ16) != r->currentVelocity) {
    r->smoothVelocityQ16 = (int64_t)r->currentVelocity * 65536;
    r->smoothAccelerationQ16 = 0;
  }
  int64_t targetQ16 = (int64_t)r->targetVelocity * 65536;
  int64_t errorQ16 = targetQ16 - r->smoothVelocityQ16;
  int sign = errorQ16 > 0 ? 1 : errorQ16 < 0 ? -1 : 0;
  int64_t limitA = (int64_t)r->accelCountsPerTickPerSec * 65536;
  int64_t jerkStep = (int64_t)r->jerkCountsPerTickPerSec2 * dtCycles * 65536 / RAMP_DENOM;
  if (jerkStep < 1) jerkStep = 1;
  int64_t candidateA = Clamp64(r->smoothAccelerationQ16 + sign*jerkStep, -limitA, limitA);
  int64_t magnitudeA = Abs64(candidateA);
  /* Discrete velocity distance needed to bring candidate acceleration back
   * to zero, plus the candidate step itself. All are Q16 velocity units. */
  int64_t n = magnitudeA > 0 ? (magnitudeA-1)/jerkStep : 0;
  int64_t braking = (n*magnitudeA - jerkStep*n*(n+1)/2) * dtCycles / RAMP_DENOM;
  int64_t candidateStep = magnitudeA * dtCycles / RAMP_DENOM;
  int64_t desiredA = (candidateA*sign > 0 && Abs64(errorQ16) < braking+candidateStep) ? 0 : sign*limitA;
  int64_t nextA = r->smoothAccelerationQ16 +
                  Clamp64(desiredA-r->smoothAccelerationQ16, -jerkStep, jerkStep);
  /* Exact small final step, only if both entering and leaving it respect
   * the jerk bound. This prevents a fractional residual from stalling. */
  int64_t finalA = errorQ16 * RAMP_DENOM / dtCycles;
  if (Abs64(finalA) <= jerkStep && Abs64(finalA-r->smoothAccelerationQ16) <= jerkStep) {
    r->smoothVelocityQ16 = targetQ16;
    nextA = finalA;
  } else {
    r->smoothVelocityQ16 += nextA * dtCycles / RAMP_DENOM;
  }
  r->smoothAccelerationQ16 = nextA;
  r->currentVelocity = RoundedVelocity(r->smoothVelocityQ16);
  return r->currentVelocity;
}
