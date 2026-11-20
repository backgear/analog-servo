#ifndef GAIN_GLIDE_H
#define GAIN_GLIDE_H

#include <stdint.h>
#include "Control.h"

/* Smoothly interpolates ControlGains from one value to another over a
 * fixed number of control ticks, instead of main.c's existing
 * AppSetGains()/ApplyConfigToModules() instant-apply path -- used by RLS's
 * AppRlsApply() so a freshly computed candidate doesn't jump the live
 * torque gains in one tick mid-move.
 *
 * Deliberately tick-count driven, not DWT-cycle driven like Ramp.c (which
 * needs real-elapsed-time accuracy to respect physical acceleration
 * limits). A gain glide has no such physical constraint -- it only needs a
 * smooth N-tick interpolation at whatever the live control tick rate is --
 * so it skips Ramp.c's SYSCLK_HZ/RAMP_DENOM machinery entirely. This is
 * deliberate, not an oversight: totalTicks must be computed by the caller
 * from a real time duration and the LIVE tick rate (see RLS.h's tick-rate
 * note) -- GainGlide.c itself never assumes what that rate is. */

typedef struct {
  ControlGains from;
  ControlGains to;
  uint32_t totalTicks;
  uint32_t ticksElapsed;
  int active;
} GainGlideState;

void GainGlideStart(GainGlideState *g, const ControlGains *from, const ControlGains *to,
                     uint32_t totalTicks);
int  GainGlideActive(const GainGlideState *g);

/* Call once per control tick while active. Writes the interpolated gains
 * into *out and advances ticksElapsed; returns 0 and leaves *out untouched
 * if !active (including once the glide has already reached `to`). */
int GainGlideTick(GainGlideState *g, ControlGains *out);

#endif /* GAIN_GLIDE_H */
