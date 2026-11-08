/*
 * Relay-feedback velocity-loop autotune. See Autotune.h for the algorithm
 * and for why it's the project's only autotune method.
 */
#include "Autotune.h"

#define AUTOTUNE_HYSTERESIS_COUNTS 2U     /* counts/tick deadband around zero velocity,
                                              avoids switching on measurement noise right
                                              at the crossing -- tune on the bench */
#define AUTOTUNE_SETTLE_SWITCHES   2U     /* discard the first N half-cycles (startup transient) */
#define AUTOTUNE_MEASURE_SWITCHES  6U     /* average the next N half-cycles (~3 full periods) */
#define AUTOTUNE_DEFAULT_MAX_TICKS 20000U /* ~20s @ 1kHz -- hard stop if it never oscillates enough */
#define AUTOTUNE_RELAY_PROGRESS_INTERVAL_TICKS  500U   /* ~0.5s @ 1kHz */

static AutotuneMethod sMethod;
static int             sActive;
static AutotuneResult  sResult;
static uint32_t        sTicksElapsed;   /* ticks into the current attempt */
static int32_t         sAmplitude;

static int32_t  sKpResultQ8, sKiResultQ8, sPosPResultQ8, sGainResultQ8;
static uint32_t sMeasuredPeriodTicks;

static uint32_t sMaxTicks;
static int      sRelayHigh;          /* 1 == currently commanding +amplitude */
static uint32_t sSwitchCount;
static uint32_t sLastSwitchTick[2];  /* [0]/[1] == last tick the relay switched TO -/+amplitude;
                                         0 == "not seen yet" (sTicksElapsed starts counting at 1) */
static int32_t  sPeakThisHalfCycle;
static uint32_t sPeriodSumTicks;
static uint32_t sPeriodSamples;
static int32_t  sPeakSumCounts;
static uint32_t sPeakSamples;

static uint32_t sRelayProgressTicks;   /* countdown to next snapshot, separate from sTicksElapsed */
static uint32_t sRelayProgressSeq;
static uint32_t sRelayProgressTicksElapsed;
static uint32_t sRelayProgressSwitchCount;
static int32_t  sRelayProgressPeakVelocity;

void AutotuneInit(void) {
  sActive = 0;
  sResult = AUTOTUNE_RESULT_NONE;
  sMethod = AUTOTUNE_METHOD_RELAY;
  sMeasuredPeriodTicks = 0U;
  sKpResultQ8 = 0;
  sKiResultQ8 = 0;
  sPosPResultQ8 = 0;
  sGainResultQ8 = 0;
  sRelayProgressTicks = 0U;
  sRelayProgressSeq = 0U;
  sRelayProgressTicksElapsed = 0U;
  sRelayProgressSwitchCount = 0U;
  sRelayProgressPeakVelocity = 0;
}

void AutotuneStartRelay(int32_t amplitude, uint32_t maxTicks) {
  sMethod = AUTOTUNE_METHOD_RELAY;
  sAmplitude = amplitude;
  sMaxTicks = (maxTicks != 0U) ? maxTicks : AUTOTUNE_DEFAULT_MAX_TICKS;
  sTicksElapsed = 0U;
  sRelayHigh = 1;
  sSwitchCount = 0U;
  sLastSwitchTick[0] = 0U;
  sLastSwitchTick[1] = 0U;
  sPeakThisHalfCycle = 0;
  sRelayProgressTicks = 0U;
  sPeriodSumTicks = 0U;
  sPeriodSamples = 0U;
  sPeakSumCounts = 0;
  sPeakSamples = 0U;
  sGainResultQ8 = 0;
  sResult = AUTOTUNE_RESULT_NONE;
  sActive = 1;
}

void AutotuneStop(void) {
  if (sActive && sResult == AUTOTUNE_RESULT_NONE) {
    sResult = AUTOTUNE_RESULT_NO_OSCILLATION;
  }
  sActive = 0;
}

int AutotuneIsActive(void) { return sActive; }
AutotuneMethod AutotuneGetMethod(void) { return sMethod; }
AutotuneResult AutotuneGetResult(void) { return sResult; }
int32_t  AutotuneGetKpResultQ8(void) { return sKpResultQ8; }
int32_t  AutotuneGetKiResultQ8(void) { return sKiResultQ8; }
int32_t  AutotuneGetPosPResultQ8(void) { return sPosPResultQ8; }
uint32_t AutotuneGetMeasuredPeriodTicks(void) { return sMeasuredPeriodTicks; }
int32_t  AutotuneGetMeasuredGainQ8(void) { return sGainResultQ8; }

uint32_t AutotuneGetRelayProgressSeq(void) { return sRelayProgressSeq; }
uint32_t AutotuneGetRelayProgressTicksElapsed(void) { return sRelayProgressTicksElapsed; }
uint32_t AutotuneGetRelayProgressMaxTicks(void) { return sMaxTicks; }
uint32_t AutotuneGetRelayProgressSwitchCount(void) { return sRelayProgressSwitchCount; }
uint32_t AutotuneGetRelayProgressSwitchesNeeded(void) {
  return AUTOTUNE_SETTLE_SWITCHES + AUTOTUNE_MEASURE_SWITCHES;
}
int32_t  AutotuneGetRelayProgressPeakVelocity(void) { return sRelayProgressPeakVelocity; }

/* Ku=4*|amplitude|/(pi*peak) describing-function analysis + tempered
 * Ziegler-Nichols PI -- see Autotune.h's file comment for the derivation.
 * Rejects (rather than reports) a non-physical result -- shouldn't normally
 * trigger if NO_OSCILLATION didn't already catch it first, but a wrong gain
 * is worse than a refused one. */
static void ComputeRelayResult(void) {
  sActive = 0;

  if (sPeriodSamples == 0U || sPeakSamples == 0U) {
    sResult = AUTOTUNE_RESULT_BAD_MEASUREMENT;
    return;
  }

  float periodTicks = (float)sPeriodSumTicks / (float)sPeriodSamples;
  float peakCounts   = (float)sPeakSumCounts  / (float)sPeakSamples;
  if (periodTicks <= 0.0f || peakCounts <= 0.0f) {
    sResult = AUTOTUNE_RESULT_BAD_MEASUREMENT;
    return;
  }

  int32_t absAmplitude = (sAmplitude < 0) ? -sAmplitude : sAmplitude;
  float d   = (float)absAmplitude;
  float Ku  = (4.0f * d) / (3.14159265f * peakCounts);
  float Kp  = 0.45f * Ku;
  float Ti  = periodTicks / 1.2f;
  float Ki  = Kp / Ti;
  float posP = (1.2f / periodTicks) / 5.0f;   /* outer loop ~5x slower than the tuned
                                                  velocity loop's Ti^-1 -- see Autotune.h */

  int32_t kpQ8   = (int32_t)(Kp   * 256.0f + 0.5f);
  int32_t kiQ8   = (int32_t)(Ki   * 256.0f + 0.5f);
  int32_t posPQ8 = (int32_t)(posP * 256.0f + 0.5f);
  if (kpQ8 <= 0 || kiQ8 <= 0 || posPQ8 <= 0) {
    sResult = AUTOTUNE_RESULT_BAD_MEASUREMENT;
    return;
  }

  sMeasuredPeriodTicks = (uint32_t)(periodTicks + 0.5f);
  sKpResultQ8 = kpQ8;
  sKiResultQ8 = kiQ8;
  sPosPResultQ8 = posPQ8;
  sResult = AUTOTUNE_RESULT_OK;
}

static void CaptureRelayProgress(void) {
  sRelayProgressTicks++;
  if (sRelayProgressTicks < AUTOTUNE_RELAY_PROGRESS_INTERVAL_TICKS) return;
  sRelayProgressTicks = 0U;
  sRelayProgressSeq++;
  sRelayProgressTicksElapsed = sTicksElapsed;
  sRelayProgressSwitchCount = sSwitchCount;
  sRelayProgressPeakVelocity = sPeakThisHalfCycle;
}

static int32_t RelayTick(int32_t velocityActual) {
  sTicksElapsed++;

  int32_t absVel = (velocityActual < 0) ? -velocityActual : velocityActual;
  if (absVel > sPeakThisHalfCycle) sPeakThisHalfCycle = absVel;

  CaptureRelayProgress();   /* after sPeakThisHalfCycle updated above, so a snapshot
                                 always reflects this tick's own peak, not last tick's */

  int switched = 0;
  if (sRelayHigh && velocityActual > (int32_t)AUTOTUNE_HYSTERESIS_COUNTS) {
    sRelayHigh = 0;
    switched = 1;
  } else if (!sRelayHigh && velocityActual < -(int32_t)AUTOTUNE_HYSTERESIS_COUNTS) {
    sRelayHigh = 1;
    switched = 1;
  }

  if (switched) {
    sSwitchCount++;
    uint32_t dir = (uint32_t)sRelayHigh;   /* direction just entered */
    int inWindow = (sSwitchCount > AUTOTUNE_SETTLE_SWITCHES) &&
                   (sSwitchCount <= AUTOTUNE_SETTLE_SWITCHES + AUTOTUNE_MEASURE_SWITCHES);
    if (inWindow) {
      sPeakSumCounts += sPeakThisHalfCycle;
      sPeakSamples++;
      if (sLastSwitchTick[dir] != 0U) {
        sPeriodSumTicks += (sTicksElapsed - sLastSwitchTick[dir]);
        sPeriodSamples++;
      }
    }
    sLastSwitchTick[dir] = sTicksElapsed;
    sPeakThisHalfCycle = 0;

    if (sSwitchCount == AUTOTUNE_SETTLE_SWITCHES + AUTOTUNE_MEASURE_SWITCHES) {
      ComputeRelayResult();
      return 0;
    }
  }

  if (sTicksElapsed >= sMaxTicks) {
    sActive = 0;
    sResult = AUTOTUNE_RESULT_NO_OSCILLATION;
    return 0;
  }

  return sRelayHigh ? sAmplitude : -sAmplitude;
}

int32_t AutotuneTick(int32_t velocityActual) {
  if (!sActive) return 0;
  return RelayTick(velocityActual);
}
