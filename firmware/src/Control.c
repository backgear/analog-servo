/*
 * Cascaded position(P) -> velocity(PI) control core. See Control.h.
 *
 * Anti-windup: conditional integration (freeze/undo this tick's integral
 * contribution only when saturating AND the error would drive further into
 * that same-direction clamp) -- chosen over back-calculation because it
 * needs no extra tuning gain and is the simplest correct choice for a single
 * output clamp, appropriate for a tier-1 pass.
 */
#include "Control.h"
#include <limits.h>
#include "Output.h"   /* TORQUE_FULL_SCALE only -- a plain #define, no HAL dependency,
                         so this stays natively testable. */

/* Struct-initializer fallback only -- inert in the real firmware, where
 * main.c's boot sequence always calls ApplyConfigToModules() (which calls
 * ControlSetGains() from ConfigStore.c's ConfigDataSetDefaults(), or a
 * loaded flash record) before ControlInit(NULL) ever runs, so this value is
 * never actually observed there. 1:1 here is just a convenient round number
 * for native tests that pass their own explicit ControlGains anyway -- NOT a
 * recommendation; see ConfigStore.c's ConfigDataSetDefaults() for the actual
 * real-hardware default and why 1:1 would be a bad choice there. */
static ControlGains sGains = {
  .posGainQ8 = 256,
  .velGainKpQ8 = 256,
  .velGainKiQ8 = 0,
  .velocityClamp = 10000,
  .torqueClampMin = -TORQUE_FULL_SCALE,
  .torqueClampMax = TORQUE_FULL_SCALE,
};

/* Keep velocity errors in Q8 and accumulated torque in Q16. No per-tick
 * truncation of the integral, and no whole-count position-loop deadband.
 * There IS a whole-count VELOCITY-error deadband (velocityErrorDeadbandCounts,
 * default 0/disabled) -- see Control.h's field comment; a position-loop
 * deadband and this one solve different problems (masking a real residual
 * position error vs. ignoring unavoidable quantization noise in the
 * velocity signal) and aren't interchangeable. */
static int64_t sVelIntegralQ16 = 0;
static int64_t sFilteredVelocityQ8 = 0;
static int sVelocityFilterInitialized = 0;
static int64_t sPreviousReferenceVelocityQ8 = 0;
static int64_t sReferenceAccelerationQ8 = 0;
static int32_t sLastVelocityTarget = 0;
static int32_t sLastPositionError = 0;
static int64_t sLastVelocityReferenceQ8, sLastVelocityErrorQ8, sLastControlErrorQ8;
static int32_t sTorqueScaleQ8 = 256;   /* 256 = 100%, no scaling -- see ControlSetTorqueScale() */

void ControlInit(const ControlGains *gains) {
  if (gains) sGains = *gains;
  sVelIntegralQ16 = 0;
  sFilteredVelocityQ8 = 0;
  sVelocityFilterInitialized = 0;
  sPreviousReferenceVelocityQ8 = 0;
  sReferenceAccelerationQ8 = 0;
  sLastVelocityTarget = 0;
  sLastPositionError = 0;
  sLastVelocityReferenceQ8 = sLastVelocityErrorQ8 = sLastControlErrorQ8 = 0;
  sTorqueScaleQ8 = 256;
}

void ControlSetGains(const ControlGains *gains) { if (gains) sGains = *gains; }
const ControlGains *ControlGetGains(void) { return &sGains; }

void ControlSetTorqueScale(int32_t scaleQ8) { sTorqueScaleQ8 = scaleQ8; }

static int64_t Clamp64(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

static int32_t RoundQ16(int64_t v) {
  return (int32_t)(v >= 0 ? (v + 32768) / 65536 : -((-v + 32768) / 65536));
}

static int32_t VelocityTickQ8(int64_t targetQ8, int32_t measuredVelocity, int64_t torqueFeedforwardQ16) {
  int64_t measuredQ8 = (int64_t)measuredVelocity * 256;
  int shift = sGains.velocityFilterShift;
  if (!sVelocityFilterInitialized || shift <= 0) {
    sFilteredVelocityQ8 = measuredQ8;
    sVelocityFilterInitialized = 1;
  } else {
    /* Causal one-pole estimator; shift=1 means alpha=1/2. Raw velocity
     * remains available to monitor/fault logic. Filtering adds delay, so
     * it is disabled by default and must earn its place on the bench. */
    if (shift > 4) shift = 4;
    sFilteredVelocityQ8 += (measuredQ8 - sFilteredVelocityQ8) / (1 << shift);
  }
  sLastVelocityTarget = (int32_t)(targetQ8 / 256);
  int64_t errorQ8 = targetQ8 - sFilteredVelocityQ8;
  sLastVelocityReferenceQ8 = targetQ8;
  sLastVelocityErrorQ8 = errorQ8;
  if (sGains.velocityErrorDeadbandCounts > 0) {
    int64_t deadbandQ8 = (int64_t)sGains.velocityErrorDeadbandCounts * 256;
    /* >=/<=, not strict -- velocityActual is always a whole count, so errorQ8 against a
     * whole-count target is always an exact multiple of 256; a strict < would never
     * actually catch deadbandCounts=1's own boundary (exactly 1 count), making it a no-op
     * for the one case it exists to suppress. */
    if (errorQ8 >= -deadbandQ8 && errorQ8 <= deadbandQ8) errorQ8 = 0;
  }
  sLastControlErrorQ8 = errorQ8;
  int64_t pQ16 = errorQ8 * sGains.velGainKpQ8;
  int64_t candidateQ16 = sVelIntegralQ16 + errorQ8 * sGains.velGainKiQ8;
  int64_t loQ16 = (int64_t)sGains.torqueClampMin * sTorqueScaleQ8 * 256;
  int64_t hiQ16 = (int64_t)sGains.torqueClampMax * sTorqueScaleQ8 * 256;
  /* Clamp integral state as well as output to prevent unbounded state on
   * runtime gain/limit changes. Include fractional bits in anti-windup. */
  candidateQ16 = Clamp64(candidateQ16, loQ16, hiQ16);
  int64_t rawQ16 = pQ16 + candidateQ16 + torqueFeedforwardQ16;
  if (!((rawQ16 > hiQ16 && errorQ8 > 0) || (rawQ16 < loQ16 && errorQ8 < 0)))
    sVelIntegralQ16 = candidateQ16;
  return RoundQ16(Clamp64(rawQ16, loQ16, hiQ16));
}

int32_t ControlTick(int32_t positionTarget, int32_t positionActual,
                     int32_t velocityActual, uint32_t dtCycles) {
  sPreviousReferenceVelocityQ8 = 0;
  sReferenceAccelerationQ8 = 0;
  return ControlTickWithVelocity(positionTarget, positionActual, velocityActual, 0, dtCycles);
}

int32_t ControlTickWithVelocity(int32_t positionTarget, int32_t positionActual,
                                int32_t velocityActual, int32_t referenceVelocity,
                                uint32_t dtCycles) {
  return ControlTickWithReferenceQ8(positionTarget, positionActual, velocityActual,
                                     (int64_t)referenceVelocity * 256, dtCycles);
}

int32_t ControlTickWithReferenceQ8(int32_t positionTarget, int32_t positionActual,
                                   int32_t velocityActual, int64_t referenceVelocityQ8,
                                   uint32_t dtCycles) {
  return ControlTickWithReferenceLimitQ8(positionTarget, positionActual, velocityActual,
      referenceVelocityQ8, sGains.velocityClamp, dtCycles);
}

int32_t ControlTickWithReferenceLimitQ8(int32_t positionTarget, int32_t positionActual,
                                        int32_t velocityActual, int64_t referenceVelocityQ8,
                                        int32_t velocityLimit, uint32_t dtCycles) {
  (void)dtCycles;
  if (velocityLimit < 0) velocityLimit = 0;
  if (velocityLimit > sGains.velocityClamp) velocityLimit = sGains.velocityClamp;
  int64_t error = (int64_t)positionTarget - positionActual;
  sLastPositionError = (int32_t)error;
  int64_t targetQ8 = error * sGains.posGainQ8 + referenceVelocityQ8;
  int64_t speedLimitQ8 = (int64_t)velocityLimit * 256;
  int64_t accelerationQ8 = referenceVelocityQ8 - sPreviousReferenceVelocityQ8;
  sPreviousReferenceVelocityQ8 = referenceVelocityQ8;
  /* The integer trajectory alternates 1/2-count velocity increments for
   * fractional acceleration. Average those causal increments to avoid
   * clipping a physically reasonable mean feedforward into torque spikes.
   * alpha=1/2 adds about one tick of lag; validated separately on the bench. */
  sReferenceAccelerationQ8 = (sReferenceAccelerationQ8 + accelerationQ8) / 2;
  int64_t torqueFFQ16 = sReferenceAccelerationQ8 * sGains.accelerationFeedforwardQ8;
  return VelocityTickQ8(Clamp64(targetQ8, -speedLimitQ8, speedLimitQ8), velocityActual, torqueFFQ16);
}

int32_t ControlVelocityTick(int32_t velocityTarget, int32_t velocityActual, uint32_t dtCycles) {
  (void)dtCycles;
  sPreviousReferenceVelocityQ8 = 0;
  sReferenceAccelerationQ8 = 0;
  return VelocityTickQ8((int64_t)velocityTarget * 256, velocityActual, 0);
}

void ControlReset(void) {
  sVelIntegralQ16 = 0;
  sFilteredVelocityQ8 = 0;
  sVelocityFilterInitialized = 0;
  sPreviousReferenceVelocityQ8 = 0;
  sReferenceAccelerationQ8 = 0;
  sLastVelocityTarget = 0;
  sLastPositionError = 0;
  sLastVelocityReferenceQ8 = sLastVelocityErrorQ8 = sLastControlErrorQ8 = 0;
  sTorqueScaleQ8 = 256;
}

int32_t ControlGetLastVelocityTarget(void) { return sLastVelocityTarget; }
int32_t ControlGetLastPositionError(void) { return sLastPositionError; }

static int32_t DiagnosticRoundQ8(int64_t value) {
  int64_t rounded = value >= 0 ? (value + 128) / 256 : -((-value + 128) / 256);
  return (int32_t)Clamp64(rounded, INT32_MIN, INT32_MAX);
}

void ControlGetVelocitySample(ControlVelocitySample *out) {
  out->reference = DiagnosticRoundQ8(sLastVelocityReferenceQ8);
  out->filtered = DiagnosticRoundQ8(sFilteredVelocityQ8);
  out->error = DiagnosticRoundQ8(sLastVelocityErrorQ8);
  out->controlError = DiagnosticRoundQ8(sLastControlErrorQ8);
}
