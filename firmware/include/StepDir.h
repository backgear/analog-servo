#ifndef STEPDIR_H
#define STEPDIR_H

#include <stdint.h>

/* Mode A (step/dir) input: TIM2 in external clock mode 2 -- the ETR pin
 * (PA0/STEP) is the hardware clock source, so TIM2's counter
 * increments/decrements in hardware on each STEP edge with NO software
 * involvement in the counting itself. Direction is TIM2->CR1's DIR bit,
 * flipped by StepDirDirIsr() (EXTI8 on PB8/DIR).
 *
 * UNVERIFIED WITHOUT THE DATASHEET IN HAND (see docs/boards.md
 * open points): whether CR1.DIR applies to the very next counted edge or is
 * shadow/preload-buffered until an update event (which, with a free-running
 * ARR=0xFFFFFFFF, would almost never occur on its own), and whether DIR is
 * writable while CEN=1. Cheap bench check before trusting this: hold DIR
 * high, pulse PA0 a known number of times while reading TIM2->CNT over SWD,
 * flip DIR, pulse again, confirm clean increment/decrement with no
 * missed/extra count at the transition. */

void StepDirInit(void);

/* Signed delta of TIM2->CNT since the last call -- meant to be called exactly
 * once per 1kHz control tick, same convention as
 * EncoderGetDeltaSinceLastTick(). */
int32_t StepDirGetDeltaCounts(void);

/* EXTI8 handler body: reads the DIR pin level, writes TIM2->CR1's DIR bit. */
void StepDirDirIsr(void);

#endif /* STEPDIR_H */
