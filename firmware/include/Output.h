#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>

/* Analog torque-command output: the real on-chip DAC1, two channels
 * (DAC_OUT1/DAC_OUT2, PA4/PA5) into a downstream differential-input
 * amplifier stage -- see docs/architecture.md.
 *
 * Bipolar torque is encoded as **two unipolar channels**, not one channel
 * biased around a mid-scale code: "positive" and "negative" each sweep
 * their own native range from zero (no output) up to full magnitude, and
 * the downstream analog stage takes their *difference* to recover a
 * bipolar result (single reference, no negative rail needed). */

/* Signed torque command, full scale +-TORQUE_FULL_SCALE (matches Control.c's
 * torqueClampMin/Max range) -- 0 maps to both channels at their own zero. */
#define TORQUE_FULL_SCALE 32767

void OutputInit(void);
void OutputSetTorqueCommand(int32_t torqueCommandQ);

/* Signed offset added (DAC codes) to the effective signed value before it's
 * split across the positive/negative channels, correcting residual
 * analog-stage zero error (DAC/differential-amp input offset) that the
 * digital math alone can't know about. Defaults to 0; persisted via
 * ConfigStore, set over the protocol as `output.trim`. */
void OutputSetZeroTrim(int32_t trimCounts);

/* Immediate zero/safe -- called by Fault.c the instant any hard fault
 * latches, and every tick thereafter while latched. */
void OutputForceSafe(void);

#endif /* OUTPUT_H */
