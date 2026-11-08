/*
 * Built-in open-loop torque test-signal generator. See Signal.h. Pure
 * fixed-point tick counting, no HAL dependency -- natively testable.
 */
#include "Signal.h"

static SignalType   sType = SIGNAL_NONE;
static SignalTarget sTarget = SIGNAL_TARGET_TORQUE;
static int32_t    sAmplitude = 0;
static uint32_t   sHalfPeriodTicks = 1;
static uint32_t   sDurationTicks = 0;   /* 0 == unlimited */
static uint32_t   sElapsedTicks = 0;
static uint32_t   sPhaseTicks = 0;
static int        sPolarityPositive = 1;
static int        sCycleStartPulse = 0;

void SignalInit(void) {
  sType = SIGNAL_NONE;
  sTarget = SIGNAL_TARGET_TORQUE;
  sElapsedTicks = 0;
  sPhaseTicks = 0;
  sPolarityPositive = 1;
  sCycleStartPulse = 0;
}

void SignalStart(SignalType type, SignalTarget target, int32_t amplitude,
                  uint32_t halfPeriodTicks, uint32_t durationTicks) {
  sType = type;
  sTarget = target;
  sAmplitude = amplitude;
  sHalfPeriodTicks = (halfPeriodTicks == 0U) ? 1U : halfPeriodTicks;
  sDurationTicks = durationTicks;
  sElapsedTicks = 0;
  sPhaseTicks = 0;
  sPolarityPositive = 1;
  sCycleStartPulse = 0;
}

void SignalStop(void) {
  sType = SIGNAL_NONE;
  sCycleStartPulse = 0;   /* a stop with no following SignalTick() call must not leave
                             this pulse stuck at whatever it was on the last active tick --
                             see SignalCycleStartPulse()'s comment: it's a one-tick pulse,
                             not sticky state, and a host reading it afterward (Monitor.c's
                             status bit7, every tick, active or not) would otherwise see a
                             permanently "stuck high" cycle-start marker. */
}
int  SignalIsActive(void) { return sType != SIGNAL_NONE; }
SignalType   SignalGetType(void) { return sType; }
SignalTarget SignalGetTarget(void) { return sTarget; }

int32_t SignalTick(void) {
  sCycleStartPulse = 0;
  if (sType == SIGNAL_NONE) return 0;

  if (sDurationTicks != 0U && sElapsedTicks >= sDurationTicks) {
    sType = SIGNAL_NONE;
    return 0;
  }

  int32_t out = 0;
  if (sType == SIGNAL_IMPULSE) {
    out = sAmplitude;
    sType = SIGNAL_NONE;   /* exactly one tick, then done */
  } else if (sType == SIGNAL_SQUARE) {
    /* phaseTicks==0 && positive is true on the first tick of every
     * +amplitude half -- including this signal's very first tick ever,
     * since sPhaseTicks/sPolarityPositive both start at their "beginning of
     * a positive half" values in SignalStart() -- so this doubles as both
     * "cycle just started" and "signal just started." */
    sCycleStartPulse = (sPhaseTicks == 0U && sPolarityPositive) ? 1 : 0;
    out = sPolarityPositive ? sAmplitude : -sAmplitude;
    sPhaseTicks++;
    if (sPhaseTicks >= sHalfPeriodTicks) {
      sPhaseTicks = 0U;
      sPolarityPositive = !sPolarityPositive;
    }
  }

  sElapsedTicks++;
  return out;
}

int SignalCycleStartPulse(void) { return sCycleStartPulse; }
