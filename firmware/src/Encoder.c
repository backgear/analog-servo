/*
 * TIM4 hardware quadrature encoder logic -- register/HAL setup itself lives
 * in tim.c (Tim4EncoderInit/Tim4GetCount), matching this repo's convention.
 *
 * EncoderTotalEdgesSeen() is derived from the abs() of each tick's CNT delta
 * (summed), not a separate raw edge-interrupt counter -- TIM4's own CC1/CC2
 * channels are consumed by hardware encoder mode already, so a second,
 * software edge-counting ISR on the same pins would both fight the hardware
 * decoder and add ISR overhead at motor edge rates. Summing abs(delta) each
 * tick gets the same semantic (Fault.c's zero-pulse detector wants "did any
 * edges happen", distinct from net position, which a stalled-but-vibrating
 * motor could hold at zero) for free from data already read every tick.
 */
#include "Encoder.h"
#include "tim.h"

static int16_t  sLastCnt = 0;
static int32_t  sPositionCounts = 0;
static uint32_t sTotalEdges = 0;
static volatile int sZSeen = 0;

void EncoderInit(void) {
  Tim4EncoderInit();
  sLastCnt = (int16_t)Tim4GetCount();
  sPositionCounts = 0;
  sTotalEdges = 0;
  sZSeen = 0;
}

void EncoderResetPosition(void) {
  int16_t nowCnt = (int16_t)Tim4GetCount();
  int32_t pending = (int32_t)(int16_t)(nowCnt - sLastCnt);
  sTotalEdges += (uint32_t)(pending < 0 ? -pending : pending);
  sLastCnt = nowCnt;
  sPositionCounts = 0;
}

int32_t EncoderGetDeltaSinceLastTick(void) {
  int16_t nowCnt = (int16_t)Tim4GetCount();
  int32_t delta = (int32_t)(int16_t)(nowCnt - sLastCnt);   /* 16-bit wraparound-safe */
  sLastCnt = nowCnt;
  /* Define modulo-32-bit position rollover without signed-overflow UB.
   * Both casts below stay in range, including INT32_MIN's representation. */
  uint32_t positionBits = (uint32_t)sPositionCounts + (uint32_t)delta;
  sPositionCounts = positionBits <= INT32_MAX ? (int32_t)positionBits :
                    -1 - (int32_t)(UINT32_MAX - positionBits);
  sTotalEdges += (uint32_t)(delta < 0 ? -delta : delta);
  return delta;
}

int32_t EncoderGetPositionCounts(void) { return sPositionCounts; }

void EncoderZIndexIsr(void) { sZSeen = 1; }
int  EncoderZWasSeenSinceEnable(void) { return sZSeen; }
void EncoderClearZSeen(void) { sZSeen = 0; }

uint32_t EncoderTotalEdgesSeen(void) { return sTotalEdges; }
