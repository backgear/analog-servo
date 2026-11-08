#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* TIM4 hardware quadrature encoder (A=PB6/TI1, B=PB7/TI2, 4x TI12 decode) --
 * first hardware-encoder-mode use in any F103 sibling project (app/'s F411
 * TIM4 encoder is the reference; only the GPIO mode differs on F103, see
 * gpio.c). Z index (PB5) is a separate plain EXTI input, latched via
 * EncoderZIndexIsr. */

void EncoderInit(void);

/* Rebase position at the current hardware count; call with control IRQs
 * excluded. Does not reset edge totals, index latch, or hardware counting. */
void EncoderResetPosition(void);

/* Free-running signed position, TIM4->CNT sign-extended and unwrapped across
 * 16-bit rollovers. The accumulated position wraps modulo 32 bits; velocity
 * is derived from the timer delta, independently of that coordinate wrap. */
int32_t EncoderGetPositionCounts(void);

/* Naive velocity estimate for tier-1: delta since the last call, meant to be
 * called exactly once per 1kHz control tick. */
int32_t EncoderGetDeltaSinceLastTick(void);

/* Z index: latches on EncoderZIndexIsr (EXTI5), cleared by
 * EncoderClearZSeen() (e.g. on enable). */
void EncoderZIndexIsr(void);
int  EncoderZWasSeenSinceEnable(void);
void EncoderClearZSeen(void);

/* Monotonic total-edge counter (A+B edges, both directions) -- distinct from
 * EncoderGetPositionCounts(), which nets forward/backward motion to zero.
 * Feeds Fault.c's zero-pulse detector (needs "any edges at all", not "net
 * position change" -- a stalled motor buzzing against a hard limit should
 * not false-fault there). Incremented from the TIM4 CC interrupt (see
 * Encoder.c) since hardware encoder mode alone doesn't expose a raw edge
 * count independent of net position. */
uint32_t EncoderTotalEdgesSeen(void);

#endif /* ENCODER_H */
