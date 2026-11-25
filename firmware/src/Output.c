/*
 * Torque-command analog output: the real on-chip DAC1 on L152 (two
 * channels), or RC-filtered PWM on F103, which has no on-chip DAC at all
 * (see docs/rc-filter.md) -- one path per board, not a runtime toggle.
 *
 * Bipolar torque is encoded as **two unipolar channels** either way
 * (OutputSplit() below), not one channel biased around a mid-scale code:
 * whichever channel matches the commanded sign carries the magnitude (0 =
 * no output, full-scale = max magnitude), the other channel sits at its
 * own zero. A downstream differential-input amplifier stage takes the
 * difference to recover a bipolar result.
 *
 * OutputForceSafe() additionally asserts gpio.c's hardware true-zero clamp
 * (an external analog switch shorting the op-amp's own differential input
 * pins together) -- a hardware-enforced zero independent of firmware
 * correctness, on top of (not instead of) still writing a software zero.
 * Asserted ONLY on a latched hard fault; normal operation (including a
 * commanded torque of exactly zero while merely disabled/idle) releases it
 * and relies on both channels' own commanded-zero.
 */
#include "Output.h"
#include "main.h"
#include "gpio.h"
#include "tim.h"

/* Signed offset added to the commanded torque before scaling -- see
 * OutputSetZeroTrim(). Raw torque units, same scale on both boards. */
static int32_t sZeroTrim = 0;

void OutputSetZeroTrim(int32_t trimTorqueUnits) { sZeroTrim = trimTorqueUnits; }

static OutputTopology sTopology = OUTPUT_TOPOLOGY_ANALOG_TORQUE;
void OutputSetTopology(OutputTopology topology) { sTopology = topology; }

/* Clamp a signed value to +-max, then split its sign onto two unipolar
 * outputs. */
static void OutputSplit(int32_t raw, int32_t max, int32_t *pos, int32_t *neg) {
  if (raw < -max) raw = -max;
  if (raw > max)  raw = max;
  if (raw >= 0) { *pos = raw;  *neg = 0;    }
  else          { *pos = 0;    *neg = -raw; }
}

#if defined(STM32L152xE)
/* output.topology=two_phase_analog's split: each of ia/ib becomes its own
 * channel's UNSIGNED magnitude code (0 at zero command, max at full
 * magnitude) plus a separate sign flag -- not a VCC/2-biased bipolar code.
 * See gpio.h's PHASE_A_SIGN_PIN/PHASE_B_SIGN_PIN -- each phase's sign
 * drives an external analog switch that routes that channel's magnitude
 * signal onto the drive's + or - differential terminal (ground on the
 * other), recovering a full +-max bipolar swing with no analog reference
 * anywhere in the path. Deliberately does NOT apply sZeroTrim -- that
 * mechanism corrects the differential-pair scheme's small near-ground
 * mismatch, a failure mode that doesn't exist here (each channel's own
 * code-0 is already true zero, no reference to drift). L152-only, same
 * as the rest of this #if block -- F103's PWM+RC output stage doesn't
 * implement this topology (see Output.h). */
static void OutputSplitTwoPhase(int32_t ia, int32_t ib, int32_t max,
                                 int32_t *codeA, int32_t *codeB,
                                 int *signA, int *signB) {
  int32_t magA, magB;
  if (ia > TORQUE_FULL_SCALE) ia = TORQUE_FULL_SCALE;
  if (ia < -TORQUE_FULL_SCALE) ia = -TORQUE_FULL_SCALE;
  if (ib > TORQUE_FULL_SCALE) ib = TORQUE_FULL_SCALE;
  if (ib < -TORQUE_FULL_SCALE) ib = -TORQUE_FULL_SCALE;
  *signA = (ia < 0);
  *signB = (ib < 0);
  magA = (ia < 0) ? -ia : ia;
  magB = (ib < 0) ? -ib : ib;
  *codeA = (magA * max) / TORQUE_FULL_SCALE;
  *codeB = (magB * max) / TORQUE_FULL_SCALE;
}

static DAC_HandleTypeDef sHdac;

static void OutputWriteDac(int32_t torqueCommandQ) {
  /* Full 12-bit range per channel (0..4095), unipolar -- see OutputSplit(). */
  int32_t raw = (torqueCommandQ * 4095) / TORQUE_FULL_SCALE;
  int32_t pos, neg;
  OutputSplit(raw, 4095, &pos, &neg);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)pos);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)neg);
}

static void OutputWriteDacTwoPhase(int32_t ia, int32_t ib) {
  int32_t codeA, codeB;
  int signA, signB;
  OutputSplitTwoPhase(ia, ib, 4095, &codeA, &codeB, &signA, &signB);
  /* Sign before magnitude -- on a sign flip this briefly presents (new
   * sign, old magnitude) rather than (old sign, new magnitude); either
   * ordering glitches for under a tick, not worth more than a comment. */
  PhaseSignWrite(0, signA);
  PhaseSignWrite(1, signB);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)codeA);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)codeB);
}

void OutputInit(void) {
  __HAL_RCC_DAC_CLK_ENABLE();
  sHdac.Instance = DAC;
  if (HAL_DAC_Init(&sHdac) != HAL_OK) Error_Handler();
  DAC_ChannelConfTypeDef dacCh = {0};
  dacCh.DAC_Trigger = DAC_TRIGGER_NONE;
  dacCh.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&sHdac, &dacCh, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_DAC_ConfigChannel(&sHdac, &dacCh, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
  HAL_DAC_Start(&sHdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&sHdac, DAC_CHANNEL_2);
  OutputWriteDac(0);
}

void OutputSetTorqueCommand(int32_t torqueCommandQ) {
  ZeroClampWrite(0);   /* normal operation -- release the hardware clamp */
  torqueCommandQ += sZeroTrim;
  if (torqueCommandQ > TORQUE_FULL_SCALE) torqueCommandQ = TORQUE_FULL_SCALE;
  if (torqueCommandQ < -TORQUE_FULL_SCALE) torqueCommandQ = -TORQUE_FULL_SCALE;
  OutputWriteDac(torqueCommandQ);
}

void OutputSetTwoPhaseCommand(int32_t ia, int32_t ib) {
  ZeroClampWrite(0);   /* normal operation -- release the hardware clamp */
  OutputWriteDacTwoPhase(ia, ib);
}

void OutputForceSafe(void) {
  ZeroClampWrite(1);   /* hardware-enforced zero, independent of firmware correctness */
  /* Both topologies agree "zero" is both channels at their own code-0
   * (OutputSplitTwoPhase() emits magnitude 0 for ia=ib=0, sign don't-care)
   * -- still dispatches on sTopology because the two schemes write
   * different channel pairs' worth of state (torqueCommand's pos/neg pair
   * vs. ia/ib's magnitude+sign pair), not because "zero" differs. */
  if (sTopology == OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) OutputWriteDacTwoPhase(0, 0);
  else OutputWriteDac(0);
}
#else
static void OutputWritePwm(int32_t torqueCommandQ) {
  /* Full [0, TIM3_PWM_ARR] range per channel, unipolar -- see OutputSplit(). */
  int32_t max = (int32_t)TIM3_PWM_ARR;
  int32_t raw = (torqueCommandQ * max) / TORQUE_FULL_SCALE;
  int32_t pos, neg;
  OutputSplit(raw, max, &pos, &neg);
  Tim3PwmSetDuty((uint32_t)pos, (uint32_t)neg);
}

void OutputInit(void) {
  Tim3PwmInit();
  OutputWritePwm(0);
}

void OutputSetTorqueCommand(int32_t torqueCommandQ) {
  ZeroClampWrite(0);   /* normal operation -- release the hardware clamp */
  torqueCommandQ += sZeroTrim;
  if (torqueCommandQ > TORQUE_FULL_SCALE) torqueCommandQ = TORQUE_FULL_SCALE;
  if (torqueCommandQ < -TORQUE_FULL_SCALE) torqueCommandQ = -TORQUE_FULL_SCALE;
  OutputWritePwm(torqueCommandQ);
}

void OutputForceSafe(void) {
  ZeroClampWrite(1);   /* hardware-enforced zero, independent of firmware correctness */
  OutputWritePwm(0);
}
#endif
