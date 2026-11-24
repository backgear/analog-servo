#ifndef TIM_H
#define TIM_H

#include <stdint.h>

/* Raw timer-peripheral setup. Timer roles:
 *  - Control-tick: TIM6 on L152 (1kHz base interrupt, main.c's
 *    AppControlTick) -- L152 has no TIM1/TIM8 (no advanced-control timer),
 *    so the control tick uses a basic timer instead. TIM1 on F103 (a real
 *    advanced-control timer, used here only for its update interrupt).
 *    Public API name (ControlTickTimerInit) is deliberately
 *    hardware-neutral so main.c's call site doesn't need an #ifdef.
 *  - TIM2: STEP (ETR, external clock mode 2) + DIR (CR1.DIR) -- Mode A input.
 *  - TIM3: torque-command PWM (poor-man's DAC), F103 only -- see
 *    Output.c/docs/rc-filter.md. L152 uses its real on-chip DAC1 instead
 *    (Output.c), not a timer.
 *  - TIM4: hardware quadrature encoder (A/B, TI12 4x decode). */

void ControlTickTimerInit(void);

void     Tim2StepDirInit(void);
uint16_t Tim2GetCount(void);
void     Tim2SetDirectionUp(int up);   /* writes CR1.DIR (1=up/0=down semantics per StepDir.c) */

#ifndef STM32L152xE
/* carrier = TIM3CLK/(ARR+1) -- F103's 72MHz TIM3CLK gives ARR=2249 (2250
 * steps) at exactly 32kHz (72e6/2250 = 32000, no rounding). Only built on
 * F103 -- L152's real DAC1 replaces this entirely, see Output.c. */
#define TIM3_PWM_ARR 2249U
void Tim3PwmInit(void);
void Tim3PwmSetDuty(uint32_t ccrPos, uint32_t ccrNeg);   /* CH1 (positive) / CH2 (negative) */
#endif

void     Tim4EncoderInit(void);
uint16_t Tim4GetCount(void);

#endif /* TIM_H */
