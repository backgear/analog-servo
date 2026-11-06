/*
 * Electrical-angle math, Ia/Ib sine synthesis, and the alignment-learning
 * accumulator. See Commutation.h.
 */
#include "Commutation.h"
#include "Output.h"   /* TORQUE_FULL_SCALE */

/* sin(0..90 degrees) * 32767, 1-degree steps -- the other 3 quadrants are
 * reconstructed by symmetry in SinQ15Deg() below. Generated once (not
 * computed at runtime -- no floating point in this file, same convention as
 * every other fixed-point module here), values cross-checked against
 * sin(90)=32767 exactly. */
static const int16_t kSinTableQ15[91] = {
      0,   572,  1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,
   5690,  6252,  6813,  7371,  7927,  8481,  9032,  9580, 10126, 10668,
  11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886,
  16383, 16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621,
  21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964, 24351, 24730,
  25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087,
  28377, 28659, 28932, 29196, 29451, 29697, 29934, 30162, 30381, 30591,
  30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
  32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762,
  32767,
};

static int32_t WrapDeg(int32_t deg) {
  deg %= 360;
  if (deg < 0) deg += 360;
  return deg;
}

static int32_t SinQ15Deg(int32_t deg) {
  deg = WrapDeg(deg);
  int32_t quadrant = deg / 90;
  int32_t r = deg % 90;
  switch (quadrant) {
    case 0:  return  kSinTableQ15[r];
    case 1:  return  kSinTableQ15[90 - r];
    case 2:  return -kSinTableQ15[r];
    default: return -kSinTableQ15[90 - r];
  }
}

int32_t CommutationElectricalAngleFromHallDeg(int hallSector) {
  int s = ((hallSector % 6) + 6) % 6;
  return s * 60;
}

int32_t CommutationElectricalAngleFromEncoderDeg(int32_t encoderPositionCounts,
                                                  int32_t countsPerElecRev,
                                                  int32_t offsetDeg) {
  if (countsPerElecRev <= 0) return 0;   /* guarded at the CLI; defensive here too */
  int32_t m = encoderPositionCounts % countsPerElecRev;
  if (m < 0) m += countsPerElecRev;
  int32_t angle = (int32_t)(((int64_t)m * 360) / countsPerElecRev);
  return WrapDeg(angle - offsetDeg);
}

void CommutationComputePhases(int32_t torqueCommandQ, int32_t electricalAngleDeg,
                               int32_t *ia, int32_t *ib) {
  if (torqueCommandQ > TORQUE_FULL_SCALE) torqueCommandQ = TORQUE_FULL_SCALE;
  if (torqueCommandQ < -TORQUE_FULL_SCALE) torqueCommandQ = -TORQUE_FULL_SCALE;
  *ia = (int32_t)(((int64_t)torqueCommandQ * SinQ15Deg(electricalAngleDeg)) / 32767);
  *ib = (int32_t)(((int64_t)torqueCommandQ * SinQ15Deg(electricalAngleDeg - 120)) / 32767);
}

/* ---- Alignment-learning accumulator --------------------------------------- */
static int32_t sLearnSumDeg = 0;
static int32_t sLearnCount = 0;

void CommutationLearnReset(void) {
  sLearnSumDeg = 0;
  sLearnCount = 0;
}

void CommutationLearnAccumulate(int32_t rawEncoderAngleDeg, int sectorEnteredNow) {
  int32_t expectedDeg = CommutationElectricalAngleFromHallDeg(sectorEnteredNow);
  int32_t errorDeg = rawEncoderAngleDeg - expectedDeg;   /* not yet wrapped near the
                                                              running mean -- done below */
  if (sLearnCount > 0) {
    int32_t runningMeanDeg = sLearnSumDeg / sLearnCount;
    while (errorDeg - runningMeanDeg > 180) errorDeg -= 360;
    while (errorDeg - runningMeanDeg < -180) errorDeg += 360;
  }
  sLearnSumDeg += errorDeg;
  sLearnCount++;
}

int CommutationLearnEvaluate(int32_t *outOffsetDeg) {
  if (sLearnCount < COMMUTATION_LEARN_MIN_TRANSITIONS) return 0;
  *outOffsetDeg = WrapDeg(sLearnSumDeg / sLearnCount);
  return 1;
}

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* ---- Pole-count detection: by Hall-sector spacing -------------------------- */
static int32_t sSpacingSumAbsDelta = 0;
static int32_t sSpacingTransitionCount = 0;
static int     sSpacingHaveLast = 0;
static int32_t sSpacingLastEncoderPos = 0;

void CommutationPoleLearnBySpacingReset(void) {
  sSpacingSumAbsDelta = 0;
  sSpacingTransitionCount = 0;
  sSpacingHaveLast = 0;
}

void CommutationPoleLearnBySpacingAccumulate(int32_t encoderPositionCounts, int sectorEnteredNow) {
  (void)sectorEnteredNow;   /* not needed here -- spacing doesn't care which
                                sector, only that one was entered */
  if (sSpacingHaveLast) {
    sSpacingSumAbsDelta += Abs32(encoderPositionCounts - sSpacingLastEncoderPos);
    sSpacingTransitionCount++;
  }
  sSpacingLastEncoderPos = encoderPositionCounts;
  sSpacingHaveLast = 1;
}

int CommutationPoleLearnBySpacingEvaluate(int32_t encoderCountsPerMechRev,
                                           int32_t *outCountsPerElecRev,
                                           int32_t *outPolePairs) {
  if (sSpacingTransitionCount < COMMUTATION_POLE_LEARN_MIN_TRANSITIONS) return 0;
  int32_t avgCountsPerTransition = sSpacingSumAbsDelta / sSpacingTransitionCount;
  int32_t countsPerElecRev = avgCountsPerTransition * 6;
  if (countsPerElecRev <= 0) return 0;   /* degenerate -- motor never actually turned */
  if (encoderCountsPerMechRev <= 0) return 0;   /* caller's known-CPR precondition */
  int32_t polePairs = encoderCountsPerMechRev / countsPerElecRev;
  if (polePairs < 1) polePairs = 1;
  *outCountsPerElecRev = countsPerElecRev;
  *outPolePairs = polePairs;
  return 1;
}
