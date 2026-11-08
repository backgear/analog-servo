#ifndef POLARITY_H
#define POLARITY_H

#include <stdint.h>

/* Pure decision logic for the polarity-learn commissioning step (see
 * main.c's AppPolarityLearn/AppControlTick and docs/README.md).
 *
 * Given the raw encoder movement observed during the test pulse, decides
 * whether it's large enough to trust as a genuine polarity determination --
 * rather than blindly reporting "learned" with the meaningless default sign
 * of +1 when nothing actually moved (a disconnected/seized motor or an
 * unplugged encoder would otherwise silently "succeed"). */

#define POLARITY_LEARN_MIN_MOVEMENT_COUNTS 8   /* tune on the bench */

/* Returns 1 (learned; *outSign set to +1 or -1) if |movedCounts| >=
 * POLARITY_LEARN_MIN_MOVEMENT_COUNTS, else 0 (*outSign left untouched --
 * the caller must not treat it as valid, and must not overwrite any
 * previously-learned polaritySign on a failed attempt). */
int PolarityLearnEvaluate(int32_t movedCounts, int8_t *outSign);

#endif /* POLARITY_H */
