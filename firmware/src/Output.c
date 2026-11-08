/*
 * Torque-command analog output: the real on-chip DAC1, two channels.
 *
 * Bipolar torque is encoded as **two unipolar channels** (OutputSplit()
 * below), not one channel biased around a mid-scale code: whichever channel
 * matches the commanded sign carries the magnitude (0 = no output, full-
 * scale = max magnitude), the other channel sits at its own zero. A
 * downstream differential-input amplifier stage takes the difference to
 * recover a bipolar result.
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

/* Signed offset added before OutputSplit() -- see OutputSetZeroTrim(). DAC codes. */
static int32_t sZeroTrimCounts = 0;

void OutputSetZeroTrim(int32_t trimCounts) { sZeroTrimCounts = trimCounts; }

/* Clamp a signed value to +-max, then split its sign onto two unipolar
 * outputs. */
static void OutputSplit(int32_t raw, int32_t max, int32_t *pos, int32_t *neg) {
  if (raw < -max) raw = -max;
  if (raw > max)  raw = max;
  if (raw >= 0) { *pos = raw;  *neg = 0;    }
  else          { *pos = 0;    *neg = -raw; }
}

static DAC_HandleTypeDef sHdac;

static void OutputWriteDac(int32_t torqueCommandQ) {
  /* Full 12-bit range per channel (0..4095), unipolar -- see OutputSplit(). */
  int32_t raw = sZeroTrimCounts + (torqueCommandQ * 4095) / TORQUE_FULL_SCALE;
  int32_t pos, neg;
  OutputSplit(raw, 4095, &pos, &neg);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)pos);
  HAL_DAC_SetValue(&sHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)neg);
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
  if (torqueCommandQ > TORQUE_FULL_SCALE) torqueCommandQ = TORQUE_FULL_SCALE;
  if (torqueCommandQ < -TORQUE_FULL_SCALE) torqueCommandQ = -TORQUE_FULL_SCALE;
  OutputWriteDac(torqueCommandQ);
}

void OutputForceSafe(void) {
  ZeroClampWrite(1);   /* hardware-enforced zero, independent of firmware correctness */
  OutputWriteDac(0);
}
