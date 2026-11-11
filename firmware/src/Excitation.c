#include "Excitation.h"
#include <stdint.h>

static volatile int sActive;
static int sResult, sSample;
static int32_t sAmplitude, sTravel, sSpeed, sStart, sLast, sSign;
static uint32_t sHold, sDuration, sElapsed, sRandom;

int ExcitationStart(int32_t amplitude, uint32_t hold, uint32_t duration,
                    int32_t travel, int32_t speed, int32_t position) {
  if (sActive || amplitude < 1 || amplitude > 2000 || hold < 1 || hold > 100 ||
      duration < 2 * hold || duration > 30000 || duration % (2 * hold) ||
      travel < 1 || travel > 10000 || speed < 1 || speed > 100) return -1;
  sAmplitude = amplitude; sHold = hold; sDuration = duration;
  sTravel = travel; sSpeed = speed; sStart = position;
  sElapsed = 0; sRandom = 0x6d2b79f5U; sLast = 0; sSample = 0;
  sResult = 1;
  sActive = 1; /* publish only after all parameters are ready */
  return 0;
}
void ExcitationStop(int result) { sActive = 0; sResult = result; }
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
