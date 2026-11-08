/*
 * Point-to-point ramped position move. See PositionMove.h.
 */
#include "PositionMove.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

void PositionMoveInit(PositionMoveState *m) {
  m->targetPosition = 0;
  m->active = 0;
}

void PositionMoveSetTarget(PositionMoveState *m, int32_t targetPosition) {
  m->targetPosition = targetPosition;
  m->active = 1;
}

void PositionMoveStop(PositionMoveState *m) {
  m->active = 0;
}

int     PositionMoveIsActive(const PositionMoveState *m) { return m->active; }
int32_t PositionMoveGetTarget(const PositionMoveState *m) { return m->targetPosition; }

int32_t PositionMoveTick(PositionMoveState *m, RampState *ramp, int32_t positionPlanned,
                          int32_t maxVelocityCountsPerTick, uint32_t dtCycles) {
  if (!m->active) return 0;

  int32_t remaining = m->targetPosition - positionPlanned;
  if (remaining == 0) {
    m->active = 0;
    ramp->currentVelocity = 0;
    ramp->targetVelocity = 0;
    ramp->error = 0;
    return 0;
  }

  int32_t absRemaining = Abs32(remaining);
  int32_t dirSign = (remaining < 0) ? -1 : 1;
  int32_t absCurrentVel = Abs32(ramp->currentVelocity);

  /* Final-tick snap: once the remaining distance is small enough that
   * even one more tick at the current velocity would reach or pass the
   * target, finish EXACTLY now instead of letting the decel curve
   * asymptotically chase zero for a few extra ticks on discrete-rounding
   * leftovers. Commands `remaining` directly (not accel-limited) -- a
   * few-count correction is harmless, and this is what guarantees the
   * move actually lands on targetPosition rather than settling a
   * handful of counts short/long. (Deliberately NOT also gated on
   * RampPeekAccelPerTick(): that's a real per-tick step size, routinely 0
   * for a slow accel relative to the tick rate, which would make such a
   * gate nearly dead weight at low accel settings. The velocity-based
   * check alone already covers the cases that matter -- see
   * RampShouldDecelerateNow() below for the truncate-to-0 pitfall a
   * direct use of that value would hit.) */
  if (absRemaining <= absCurrentVel) {
    m->active = 0;
    /* The returned final displacement completes the reference trajectory.
     * Leave the shared ramp stopped so the next zero-velocity jog tick
     * cannot resume a stale cruise target or add residual displacement. */
    ramp->currentVelocity = 0;
    ramp->targetVelocity = 0;
    ramp->error = 0;
    return remaining;
  }

  int32_t desiredRampTarget = RampShouldDecelerateNow(ramp, dtCycles, absCurrentVel, absRemaining)
                               ? 0 : dirSign * maxVelocityCountsPerTick;

  RampSetTarget(ramp, desiredRampTarget);
  return RampTick(ramp, dtCycles);
}
