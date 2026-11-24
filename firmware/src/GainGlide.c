/* Tick-count-driven gain interpolation. See GainGlide.h. */
#include "GainGlide.h"

static int32_t Lerp32(int32_t from, int32_t to, uint32_t elapsed, uint32_t total) {
  int64_t span = (int64_t)to - (int64_t)from;
  return (int32_t)(from + (span * (int64_t)elapsed) / (int64_t)total);
}

void GainGlideStart(GainGlideState *g, const ControlGains *from, const ControlGains *to,
                     uint32_t totalTicks) {
  g->from = *from;
  g->to = *to;
  g->totalTicks = (totalTicks != 0U) ? totalTicks : 1U;
  g->ticksElapsed = 0U;
  g->active = 1;
}

int GainGlideActive(const GainGlideState *g) { return g->active; }

int GainGlideTick(GainGlideState *g, ControlGains *out) {
  if (!g->active) return 0;

  g->ticksElapsed++;
  if (g->ticksElapsed >= g->totalTicks) {
    *out = g->to;
    g->active = 0;
    return 1;
  }

  /* Every other field (clamps, filter shift, deadband, ...) is identical
   * between from/to by construction (only the three gains ever change
   * mid-glide, see AppRlsApply()) -- copy `to` wholesale, then overwrite
   * just the interpolated fields. */
  *out = g->to;
  out->posGainQ8   = Lerp32(g->from.posGainQ8,   g->to.posGainQ8,   g->ticksElapsed, g->totalTicks);
  out->velGainKpQ8 = Lerp32(g->from.velGainKpQ8, g->to.velGainKpQ8, g->ticksElapsed, g->totalTicks);
  out->velGainKiQ8 = Lerp32(g->from.velGainKiQ8, g->to.velGainKiQ8, g->ticksElapsed, g->totalTicks);
  return 1;
}
