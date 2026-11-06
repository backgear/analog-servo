#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>

/* Analog torque-command output: the real on-chip DAC1 on L152 (DAC_OUT1/
 * DAC_OUT2, PA4/PA5), or RC-filtered PWM on F103 (no on-chip DAC on that
 * part -- see docs/rc-filter.md) -- either way into a downstream
 * differential-input amplifier stage. See docs/architecture.md.
 *
 * Bipolar torque is encoded as **two unipolar channels**, not one channel
 * biased around a mid-scale code: "positive" and "negative" each sweep
 * their own native range from zero (no output) up to full magnitude, and
 * the downstream analog stage takes their *difference* to recover a
 * bipolar result (single reference, no negative rail needed). */

/* Signed torque command, full scale +-TORQUE_FULL_SCALE (matches Control.c's
 * torqueClampMin/Max range) -- 0 maps to both channels at their own zero. */
#define TORQUE_FULL_SCALE 32767

/* output.topology -- see docs/two-phase-drive.md, ported wholesale on the
 * `two-phase-output` draft-PR branch (NOT compiled into the default
 * `analog_servo` env -- see platformio.ini's `analog_servo_two_phase` env
 * and the ANALOG_SERVO_TWO_PHASE_DRIVE flag that gates it). Affects ONLY
 * OutputForceSafe()'s "what is zero on this channel pair" semantics below;
 * callers (main.c) still pick OutputSetTorqueCommand() vs.
 * OutputSetTwoPhaseCommand() themselves based on the same setting. */
typedef enum {
  OUTPUT_TOPOLOGY_ANALOG_TORQUE = 0,       /* default -- the only topology in `main` */
  OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG = 1,    /* see OutputSetTwoPhaseCommand() */
} OutputTopology;
void OutputSetTopology(OutputTopology topology);

void OutputInit(void);
void OutputSetTorqueCommand(int32_t torqueCommandQ);

/* output.topology=two_phase_analog's write path: two independent
 * sinusoidal phase commands (Commutation.c's CommutationComputePhases()),
 * each +-TORQUE_FULL_SCALE, written to the SAME two DAC channels
 * OutputSetTorqueCommand() uses -- not a third/fourth channel. Each
 * channel carries pure unsigned magnitude; sign is written separately to
 * gpio.h's PHASE_A_SIGN_PIN/PHASE_B_SIGN_PIN, each driving an external
 * analog switch (a CD4053) that routes that phase's magnitude signal onto
 * the drive's + or - differential terminal (ground on the other) -- no
 * external reference needed at all. A real board modification, not yet
 * built or bench-verified on any board -- see docs/two-phase-drive.md for
 * the switch wiring and the alternatives it replaced. L152-only for now --
 * see docs/two-phase-drive.md's own scope note; F103's PWM+RC output stage
 * doesn't implement this topology. */
void OutputSetTwoPhaseCommand(int32_t ia, int32_t ib);

/* Signed offset, in the same raw torque units as OutputSetTorqueCommand()
 * (+-TORQUE_FULL_SCALE), added to the commanded torque before it's scaled to
 * DAC/PWM codes and split across the positive/negative channels -- correcting
 * residual analog-stage zero error (DAC/differential-amp input offset) that
 * the digital math alone can't know about. Torque units rather than raw
 * DAC/PWM codes so the same configured value means the same thing on either
 * board, despite L152's 12-bit DAC and F103's PWM having different native
 * ranges. Defaults to 0; persisted via ConfigStore, set over the protocol as
 * `output.trim`. */
void OutputSetZeroTrim(int32_t trimTorqueUnits);

/* Immediate zero/safe -- called by Fault.c the instant any hard fault
 * latches, and every tick thereafter while latched. */
void OutputForceSafe(void);

#endif /* OUTPUT_H */
