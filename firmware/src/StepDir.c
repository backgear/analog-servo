/*
 * Mode A (step/dir) input logic -- TIM2 register setup itself lives in
 * tim.c (Tim2StepDirInit/Tim2GetCount/Tim2SetDirectionUp), matching this
 * repo's convention. See StepDir.h and docs/README.md.
 */
#include "StepDir.h"
#include "gpio.h"
#include "tim.h"

static int16_t sLastCnt = 0;

void StepDirInit(void) {
  Tim2StepDirInit();
  sLastCnt = (int16_t)Tim2GetCount();
}

int32_t StepDirGetDeltaCounts(void) {
  int16_t nowCnt = (int16_t)Tim2GetCount();
  int32_t delta = (int32_t)(int16_t)(nowCnt - sLastCnt);
  sLastCnt = nowCnt;
  return delta;
}

void StepDirDirIsr(void) {
  Tim2SetDirectionUp(HAL_GPIO_ReadPin(DIR_GPIO_PORT, DIR_PIN) == GPIO_PIN_SET);
}
