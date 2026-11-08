#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>

/* Mode B's own accel/decel ramp generator: takes a host-commanded velocity
 * target and ramps toward it, producing a velocity in the SAME units
 * Control.c/Encoder.c use everywhere else -- counts per 1kHz control tick --
 * so main.c can feed it straight into the position-target accumulator with
 * no further unit conversion (`positionTargetAccum += RampTick(...)`).
 *
 * Structurally modeled on app/src/Ramps.c's DWT-cycle-measured, fixed-point
 * error-accumulator integration ((rate*dtCycles+error)/DENOM, %DENOM
 * remainder) -- reused specifically because it needs no changes when tick
 * timing jitters (the same property docs/motion_planner_design.md calls
 * out), unlike a naive float-multiply-by-nominal-dt approach. DENOM is the
 * core clock rate in Hz (see Ramp.c) so dtCycles (elapsed DWT->CYCCNT ticks)
 * converts directly to elapsed seconds inside the accumulator. */

typedef struct {
  int32_t currentVelocity;   /* counts/tick, ramped */
  int32_t targetVelocity;    /* counts/tick, host-commanded */
  int32_t accelCountsPerTickPerSec;  /* ramp rate: counts/tick of velocity change, per second */
  int32_t jerkCountsPerTickPerSec2; /* 0 disables smoothing for velocity jogs */
  int64_t smoothVelocityQ16;
  int64_t smoothAccelerationQ16; /* counts/tick/sec, Q16 */
  int32_t error;             /* fixed-point remainder accumulator */
} RampState;

void RampInit(RampState *r);
/* Optional jerk-limited jog, with Q16 internal velocity and acceleration.
 * RampTick remains trapezoidal for PositionMove stopping-distance planning. */
void RampSetJerk(RampState *r, int32_t jerk);
int32_t RampSmoothTick(RampState *r, uint32_t dtCycles);

void RampSetTarget(RampState *r, int32_t targetVelocityCountsPerTick);
void RampSetAcceleration(RampState *r, int32_t accelCountsPerTickPerSec);

/* Advances currentVelocity toward targetVelocity by at most one tick's worth
 * of acceleration (given dtCycles elapsed since the last call), and returns
 * the new currentVelocity. Never overshoots targetVelocity. */
int32_t RampTick(RampState *r, uint32_t dtCycles);

/* Read-only: how much RampTick(r, dtCycles) would change |currentVelocity|
 * by THIS tick, without actually advancing r->currentVelocity/r->error --
 * same per-tick scaling RampTick() itself uses internally (same source,
 * not a reimplementation), just exposed so a caller can answer "how much
 * runway until I can no longer stop in time" (PositionMove.c's decel
 * trigger) using the exact same accel rate RampTick() will actually honor,
 * rather than a second, possibly-inconsistent conversion of
 * accelCountsPerTickPerSec into per-tick units. Always non-negative. */
int32_t RampPeekAccelPerTick(const RampState *r, uint32_t dtCycles);

/* The "can I still stop in time" check PositionMove.c's decel trigger
 * needs (v^2 >= 2*a*d), done WITHOUT first truncating the per-tick accel
 * to an integer the way RampPeekAccelPerTick() above does -- that
 * truncation is fine for its own purpose (a real per-tick velocity step
 * is genuinely 0 or more whole counts), but squaring/comparing against a
 * PREMATURELY truncated per-tick rate is a different, much easier
 * mistake to make: any accelCountsPerTickPerSec slower than
 * SYSCLK_HZ/1000 (0.3 counts/tick for a 1000 counts/tick/sec rate at a
 * real ~1kHz tick, say) truncates to exactly 0 per tick, and
 * `velocityMagnitude^2 >= 2*0*remaining` is trivially true for ANY
 * remaining distance the instant velocity is 0 -- i.e. this would
 * misfire as "must decelerate now" before ever accelerating at all,
 * regardless of how far away the target is. This function instead
 * cross-multiplies by RAMP_DENOM so the comparison itself carries full
 * precision, with no intermediate rounding step to go wrong. */
int RampShouldDecelerateNow(const RampState *r, uint32_t dtCycles,
                             int32_t velocityMagnitude, int32_t remainingDistanceMagnitude);

#endif /* RAMP_H */
