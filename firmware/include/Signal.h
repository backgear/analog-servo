#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdint.h>

/* Built-in open-loop torque test-signal generator -- bypasses Control.c's
 * cascade entirely (same "raw test torque" bypass main.c's polarity-learn
 * state machine already does) so step/impulse/oscillation response can be
 * observed directly (scope on the analog output, or `torque.command`/
 * `position.raw` telemetry) with no external signal generator wired in.
 * Runs from the 1kHz control tick, same as everything else -- periods and
 * durations are expressed in ticks (1 tick = 1ms @ 1kHz), not seconds.
 *
 * Signal.c itself is target-agnostic: it just produces a bare +-amplitude
 * waveform each tick, with no opinion on what that value means. SignalTarget
 * records what the CALLER (main.c) should do with it:
 *  - SIGNAL_TARGET_TORQUE: amplitude is raw torque-command units (same scale
 *    as torque.min/max), applied directly to Output.c with no
 *    position feedback -- useful for characterizing the raw plant/output
 *    stage, but nothing holds either side in place, so a real mechanism can
 *    migrate/drift over a run if it isn't perfectly symmetric.
 *  - SIGNAL_TARGET_POSITION: amplitude is encoder counts, added to the
 *    position captured when the signal started, and driven through
 *    Control.c's normal P->PI cascade -- each half-cycle is actively held,
 *    so the average position doesn't drift. */

typedef enum {
  SIGNAL_NONE = 0,
  SIGNAL_IMPULSE,   /* one tick at +amplitude, then auto-stops */
  SIGNAL_SQUARE,    /* alternates +amplitude/-amplitude every halfPeriodTicks */
} SignalType;

typedef enum {
  SIGNAL_TARGET_TORQUE = 0,
  SIGNAL_TARGET_POSITION = 1,
} SignalTarget;

void SignalInit(void);

/* Starts a new signal, replacing any signal already running. halfPeriodTicks
 * is ignored for SIGNAL_IMPULSE. durationTicks caps the total run length
 * (0 = run until SignalStop() is called); ignored for SIGNAL_IMPULSE, which
 * always self-stops after its one tick. */
void SignalStart(SignalType type, SignalTarget target, int32_t amplitude,
                  uint32_t halfPeriodTicks, uint32_t durationTicks);
void SignalStop(void);
int  SignalIsActive(void);
SignalType   SignalGetType(void);
SignalTarget SignalGetTarget(void);

/* Called once per 1kHz control tick while active (main.c's AppControlTick);
 * returns the raw +-amplitude contribution for this tick (torque units or
 * position-offset counts, per SignalGetTarget()), 0 once finished/stopped. */
int32_t SignalTick(void);

/* 1 on exactly the tick marking the start of a new SIGNAL_SQUARE cycle --
 * the first tick of the +amplitude half, including the signal's very first
 * tick -- 0 every other tick (always 0 for SIGNAL_IMPULSE/SIGNAL_NONE).
 * Reflects the most recent SignalTick() call; query it right after, same
 * tick, same convention as SignalGetTarget(). Exists so a host-side
 * consumer (Monitor.c's binary frame, ultimately a scope/strip-chart
 * trigger) has a firmware-sourced "cycle start" marker instead of having to
 * infer one by guessing a level crossing on torqueCommand -- ambiguous,
 * since amplitude is caller-chosen and a SIGNAL_TARGET_POSITION signal's
 * torqueCommand is the closed-loop cascade's OUTPUT, not the raw
 * +-amplitude square wave itself. */
int SignalCycleStartPulse(void);

#endif /* SIGNAL_H */
