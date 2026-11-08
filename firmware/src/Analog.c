/*
 * Mode C (analog velocity) scale/hysteresis math. See Analog.h.
 */
#include "Analog.h"

static int32_t sScaleQ8 = 0;
static int32_t sHysteresisCounts = 0;
static int32_t sLastActedOnRaw = 0;
static int     sHaveLast = 0;

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

void AnalogVelocitySetConfig(int32_t scaleQ8, int32_t hysteresisCounts) {
  sScaleQ8 = scaleQ8;
  sHysteresisCounts = hysteresisCounts;
}

void AnalogVelocityReset(void) {
  sHaveLast = 0;
}

int32_t AnalogVelocityTick(int32_t rawAdcCode, int reversed) {
  if (!sHaveLast || sHysteresisCounts <= 0 ||
      Abs32(rawAdcCode - sLastActedOnRaw) >= sHysteresisCounts) {
    sLastActedOnRaw = rawAdcCode;
    sHaveLast = 1;
  }

  int32_t magnitude = (int32_t)(((int64_t)sLastActedOnRaw * (int64_t)sScaleQ8) >> 8);
  return reversed ? -magnitude : magnitude;
}
