/*
 * Polarity-learn pass/fail decision. See Polarity.h.
 */
#include "Polarity.h"

int PolarityLearnEvaluate(int32_t movedCounts, int8_t *outSign) {
  int32_t absMoved = (movedCounts < 0) ? -movedCounts : movedCounts;
  if (absMoved < POLARITY_LEARN_MIN_MOVEMENT_COUNTS) return 0;
  *outSign = (movedCounts >= 0) ? 1 : -1;
  return 1;
}
