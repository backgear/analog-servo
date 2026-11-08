#ifndef POSITIONMOVE_H
#define POSITIONMOVE_H

#include <stdint.h>
#include "Ramp.h"

/* Point-to-point ramped position move -- the primitive behind
 * `position.command` (main.c). Unlike `test square position` (an
 * instant position-target jump, deliberately rattley -- see Signal.h)
 * this computes, every tick, the velocity needed to decelerate and land
 * exactly on targetPosition.
 *
 * Deliberately stateless with respect to the ramp itself: PositionMoveTick()
 * takes the SAME RampState main.c already drives directly for a plain
 * `velocity.command` jog (one accelCountsPerTickPerSec/trajectory.accel_limit, one
 * velocity, no separate duplicate ramp to keep in sync) -- it just calls
 * RampSetTarget() with whatever THIS tick's decel-aware target is before
 * deferring to RampTick() for the actual accel-limited change. That means
 * switching between velocity_cmd and position_cmd carries the current
 * velocity over with no discontinuity, same as retargeting a plain jog.
 *
 * Trapezoidal (constant acceleration limit), not jerk-limited S-curve --
 * same deliberate choice Ramp.c itself documents; a true S-curve would
 * need its own, considerably more involved planner on top of this one,
 * not a replacement for it.
 *
 * Recomputed from the caller's CURRENT planned position each tick. The
 * caller integrates the returned displacement into that same reference;
 * ControlTick separately closes the loop against measured position.
 * Feeding measured position here double-counts following lag and causes
 * the reference to overshoot the requested endpoint.
 *
 * Decel-trigger math: decelerate once (currentVelocity)^2 >=
 * 2 * accelPerTick * remainingDistance -- the standard "can I still stop
 * in time" check most embedded motion planners use (e.g. grbl's step
 * planner), avoiding a sqrt() every tick. RampShouldDecelerateNow()
 * uses the same acceleration scaling as RampTick() without prematurely
 * truncating fractional per-tick acceleration.
 *
 * maxVelocityCountsPerTick<=0 (e.g. velocity.limit set to 0) means
 * "never cruise" -- same as it already means for a plain velocity_cmd jog
 * today, not a new restriction this module adds; a move whose distance
 * needs more than one tick's accel to cover will simply never complete
 * in that case (stays active, commanding 0 velocity) rather than lurching
 * to the target in one oversized jump. */

typedef struct {
  int32_t targetPosition;
  int     active;   /* 0 once the target has been reached, or after PositionMoveStop() */
} PositionMoveState;

void PositionMoveInit(PositionMoveState *m);

/* Starts (or redirects, if already active) a move toward targetPosition,
 * in the same encoder-counts scale as position.raw. */
void PositionMoveSetTarget(PositionMoveState *m, int32_t targetPosition);

/* Immediately ends any in-progress move (no further velocity commanded)
 * without changing targetPosition/PositionMoveGetTarget()'s last value --
 * same "stop where you are, remember what you were asked" convention as
 * AppSignalStop(). Does NOT touch *ramp -- the caller (main.c) decides
 * what happens to the shared velocity state next, e.g. a following
 * velocity.command ramping from wherever this left off. */
void PositionMoveStop(PositionMoveState *m);

int     PositionMoveIsActive(const PositionMoveState *m);
int32_t PositionMoveGetTarget(const PositionMoveState *m);

/* Called once per 1kHz control tick while active; returns the planned
 * displacement for this tick (counts, signed). The last nonzero return
 * lands the reference exactly at the endpoint and clears active; the
 * shared ramp is then stopped. The caller must integrate the return even
 * on that final tick. Inactive calls return 0 and leave the ramp untouched.
 * positionPlanned is the CURRENT position-target accumulator, never the
 * measured encoder position. Completion means the reference has arrived;
 * the physical shaft may still be settling. */
int32_t PositionMoveTick(PositionMoveState *m, RampState *ramp, int32_t positionPlanned,
                          int32_t maxVelocityCountsPerTick, uint32_t dtCycles);

#endif /* POSITIONMOVE_H */
