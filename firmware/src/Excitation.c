#include "Excitation.h"
#include <stdint.h>

static volatile int sActive;
static int sResult, sSample, sStopMotion, sTorqueMode;
static int Start(int32_t amplitude, uint32_t hold, uint32_t duration,
                 int32_t travel, int32_t speed, int32_t position,
                 int32_t commandedVelocity, int torqueMode);
static int32_t sExpectedVelocity;
static int32_t sAmplitude, sTravel, sSpeed, sStart, sLast, sSign;
static uint32_t sHold, sDuration, sElapsed, sRandom;

int ExcitationStart(int32_t amplitude, uint32_t hold, uint32_t duration,
                    int32_t travel, int32_t speed, int32_t position) {
  return ExcitationStartMoving(amplitude, hold, duration, travel, speed, position, 0);
}
int ExcitationStartMoving(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position,
                          int32_t commandedVelocity) {
  return Start(amplitude, hold, duration, travel, speed, position, commandedVelocity, 0);
}
int ExcitationStartTorque(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position) {
  return Start(amplitude, hold, duration, travel, speed, position, 0, 1);
}
int ExcitationTorqueMode(void) { return sTorqueMode; }
static int Start(int32_t amplitude, uint32_t hold, uint32_t duration,
                 int32_t travel, int32_t speed, int32_t position,
                 int32_t commandedVelocity, int torqueMode) {
  int moving = commandedVelocity != 0;
  if (sActive || sStopMotion || amplitude < 1 || amplitude > (torqueMode ? 10000 : moving ? 500 : 2000) ||
      hold < 1 || hold > 100 ||
      duration < 2 * hold || duration > (moving ? 10000U : 30000U) || duration % (2 * hold) ||
      travel < 1 || travel > (moving ? 250000 : 10000) || speed < 1 ||
      speed > (moving ? 30 : 100) || commandedVelocity < -20 || commandedVelocity > 20 ||
      (moving && (speed <= commandedVelocity || speed <= -(int64_t)commandedVelocity))) return -1;
  sAmplitude = amplitude; sHold = hold; sDuration = duration;
  sTravel = travel; sSpeed = speed; sStart = position;
  sElapsed = 0; sRandom = 0x6d2b79f5U; sLast = 0; sSample = 0;
  sResult = 1;
  sExpectedVelocity = commandedVelocity;
  sStopMotion = moving;
  sTorqueMode = torqueMode;
  sActive = 1; /* publish only after all parameters are ready */
  return 0;
}
void ExcitationStop(int result) { sActive = 0; sResult = result; }
int32_t ExcitationExpectedVelocity(void) { return sExpectedVelocity; }
int ExcitationTakeMotionStop(void) {
  if (sActive || !sStopMotion) return 0;
  sStopMotion = 0;
  return 1;
}
int ExcitationActive(void) { return sActive; }
int ExcitationResult(void) { return sResult; }
int32_t ExcitationLast(void) { return sLast; }
int ExcitationSample(void) { return sSample; }
void ExcitationRejectClipping(void) {
  ExcitationStop(7); sLast = 0; sSample = 0;
}
int32_t ExcitationTick(int32_t position, int32_t velocity, int interlock) {
  sLast = 0; sSample = 0;
  if (!sActive) return 0;
  int64_t displacement = (int64_t)position - sStart;
  int64_t v = velocity;
  if (!interlock) { ExcitationStop(4); return 0; }
  if (displacement > sTravel || displacement < -sTravel) {
    ExcitationStop(5); return 0;
  }
  if (v > sSpeed || v < -sSpeed) { ExcitationStop(6); return 0; }
  if (sElapsed >= sDuration) { ExcitationStop(2); return 0; }
  uint32_t phase = sElapsed % (2 * sHold);
  if (phase == 0) {
    sRandom ^= sRandom << 13;
    sRandom ^= sRandom >> 17;
    sRandom ^= sRandom << 5;
    sSign = (sRandom & 1U) ? 1 : -1;
  }
  sLast = sAmplitude * (phase < sHold ? sSign : -sSign);
  sSample = 1;
  sElapsed++;
  return sLast;
}
