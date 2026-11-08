#ifndef TIM_H
#define TIM_H

#include <stdint.h>

/* Raw timer-peripheral setup. Three timer roles on the L152:
 *  - Control-tick: TIM6, 1kHz base interrupt (main.c's AppControlTick).
 *    L152 has no TIM1/TIM8 (no advanced-control timer), so the control
 *    tick uses a basic timer instead -- this project never used TIM1's
 *    advanced-control features anyway.
 *  - TIM2: STEP (ETR, external clock mode 2) + DIR (CR1.DIR) -- Mode A input.
 *  - TIM4: hardware quadrature encoder (A/B, TI12 4x decode).
 * Torque-command output uses the real on-chip DAC1 (Output.c), not a
 * timer -- see Output.h. */

void ControlTickTimerInit(void);

void     Tim2StepDirInit(void);
uint16_t Tim2GetCount(void);
void     Tim2SetDirectionUp(int up);   /* writes CR1.DIR (1=up/0=down semantics per StepDir.c) */

void     Tim4EncoderInit(void);
uint16_t Tim4GetCount(void);

#endif /* TIM_H */
