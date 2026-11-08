#include "Indicator.h"
#include "main.h"
#include "gpio.h"

#define BLINK_SLOW_MS    500U   /* idle heartbeat: ~1 Hz */
#define BLINK_FAULT_MS   125U   /* fault heartbeat: ~4 Hz, clearly distinct from idle */
#define BLINK_OVERLAY_MS 120U   /* single/double-blink ack pulses */

typedef enum { OV_NONE = 0, OV_SINGLE = 1, OV_DOUBLE = 2 } OverlayKind;

static IndicatorState sBaseState = INDICATOR_IDLE;
static OverlayKind    sOverlay = OV_NONE;
static int            sOverlayBlinksRemaining = 0;
static int            sOn = 0;
static uint32_t       sNextToggle = 0;

static void SetOutput(int on) {
  sOn = on;
  LedWrite(on);
}

void IndicatorInit(void) {
  sBaseState = INDICATOR_IDLE;
  sOverlay = OV_NONE;
  sOverlayBlinksRemaining = 0;
  sNextToggle = 0;
  SetOutput(0);
}

void IndicatorSetState(IndicatorState state) {
  if (state == sBaseState) return;   /* only touch FAULT_OUT (and avoid redundant GPIO writes) on real transitions */
  sBaseState = state;
  FaultOutWrite(state == INDICATOR_FAULT);
}

static void StartOverlay(OverlayKind kind, int blinks) {
  sOverlay = kind;
  sOverlayBlinksRemaining = blinks;
  sOn = 0;                     /* forces the first poll to turn output ON (see IndicatorPoll) */
  sNextToggle = HAL_GetTick(); /* fire on the very next poll */
}

void IndicatorPulseSingle(void) { StartOverlay(OV_SINGLE, 1); }
void IndicatorPulseDouble(void) { StartOverlay(OV_DOUBLE, 2); }

void IndicatorPoll(void) {
  uint32_t now = HAL_GetTick();

  if (sOverlay != OV_NONE) {
    if ((int32_t)(now - sNextToggle) < 0) return;
    if (sOn) {
      SetOutput(0);
      sOverlayBlinksRemaining--;
      if (sOverlayBlinksRemaining <= 0) sOverlay = OV_NONE;
    } else {
      SetOutput(1);
    }
    sNextToggle = now + BLINK_OVERLAY_MS;
    return;
  }

  if (sBaseState == INDICATOR_RUNNING) {
    if (!sOn) SetOutput(1);
    return;
  }

  /* IDLE/FAULT: blink at a state-specific rate (fault noticeably faster, so
   * the two are never confusable at a glance). */
  if ((int32_t)(now - sNextToggle) < 0) return;
  SetOutput(!sOn);
  sNextToggle = now + ((sBaseState == INDICATOR_FAULT) ? BLINK_FAULT_MS : BLINK_SLOW_MS);
}
