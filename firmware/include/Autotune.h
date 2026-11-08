#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <stdint.h>

/* Relay-feedback (Åström-Hägglund) velocity-loop autotune, picked at
 * `autotune start relay ...`. Pure fixed-point-in/float-out tick counting,
 * no HAL dependency -- natively testable, same shape as Signal.c's
 * tick-driven state machine. Doesn't auto-apply its result into the live
 * gains -- see main.c's AppAutotuneApply(), an explicit follow-up step
 * (torque gains are consequential enough to want a review, unlike
 * polarity-learn's sign, which auto-commits).
 *
 * This is the project's only autotune method. An open-loop step-response
 * method and a closed-loop local-search refine method are intentionally not
 * provided here: a step input's safety depends entirely on a mandatory
 * operator-supplied velocity ceiling (easy to get dangerously wrong on
 * unfamiliar hardware), and a local-search refine needs a well-defined
 * abort/convergence condition that's easy to get wrong in a way that never
 * terminates cleanly.
 *
 * Takes an `amplitude`/torque value in the same raw TORQUE_FULL_SCALE units
 * as torque.min/max -- a DIGITAL PWM duty-cycle code, not a
 * physical torque. Computed gains are only valid for whatever supply
 * voltage and output-stage gain were in effect during the run that produced
 * them: change either later (different bench supply, different amplifier)
 * and re-run autotune rather than trusting the old numbers.
 *
 * ---- AUTOTUNE_METHOD_RELAY (Åström-Hägglund) ------------------------------
 * Drives torque to +-amplitude based on the sign of velocity error against a
 * target of ZERO velocity (a hysteresis band around zero avoids switching on
 * measurement noise right at the crossing) -- the mechanism jitters in place
 * rather than running away, which is what makes this safe to self-terminate
 * unattended: the experiment is self-regulating by construction.
 *
 * NOT polarity-agnostic: RelayTick() commits to +amplitude first
 * (sRelayHigh=1, hardcoded, not derived from anything) and only flips once
 * velocity itself crosses the opposite hysteresis band, which assumes
 * +torque produces +velocity. On reversed wiring it never crosses and just
 * holds one sign until max_ticks times out, saturating at whatever terminal
 * velocity the plant settles to at that amplitude. Requires `output learn_polarity`
 * to have already run -- enforced by the caller (Protocol.c's
 * cmdAutotuneStart refuses to start otherwise) -- so main.c's polaritySign
 * correction fixes the sign before AutotuneTick()'s output ever reaches
 * Output.c. A wrong-but-learned polaritySign is harmless either way: it only
 * changes which direction the shaft moves first, not whether the relay
 * converges.
 *
 * The resulting oscillation's period and peak amplitude are averaged over
 * AUTOTUNE_MEASURE_SWITCHES half-cycles (discarding the first
 * AUTOTUNE_SETTLE_SWITCHES as startup transient), then converted to an
 * ultimate gain/period (Ku, Pu) via standard describing-function analysis
 * and a tempered Ziegler-Nichols PI rule (gentler than classic ZN, which
 * runs underdamped for a mechanism):
 *   Ku = 4*|amplitude| / (pi * peakVelocity)
 *   Kp = 0.45 * Ku
 *   Ti = Pu / 1.2          (ticks -- Pu comes out in ticks already, since the
 *                            control loop runs at a fixed 1kHz, so no time-
 *                            unit conversion is needed anywhere in this file)
 *   Ki = Kp / Ti            (per tick, matches Control.h's velGainKiQ8
 *                            "accumulates per tick, not per second" contract)
 * `pos_p` gets a conservative heuristic off the SAME Pu (see below) rather
 * than a genuinely separate experiment. */

typedef enum {
  AUTOTUNE_METHOD_RELAY = 0,
} AutotuneMethod;

typedef enum {
  AUTOTUNE_RESULT_NONE = 0,          /* no completed attempt yet (idle, or still running) */
  AUTOTUNE_RESULT_OK,
  AUTOTUNE_RESULT_NO_OSCILLATION,    /* didn't switch enough within maxTicks -- amplitude
                                         too small against friction/stiction, or a
                                         disconnected/seized motor */
  AUTOTUNE_RESULT_BAD_MEASUREMENT,   /* a computed gain came out non-physical --
                                         shouldn't normally trigger if NO_OSCILLATION
                                         didn't already catch it, but rejected
                                         defensively rather than reporting a nonsense
                                         gain */
} AutotuneResult;

void AutotuneInit(void);

/* amplitude: raw torque units. maxTicks: hard cap on the whole attempt,
 * 0 = use a built-in default (~20s @ 1kHz). */
void AutotuneStartRelay(int32_t amplitude, uint32_t maxTicks);

void AutotuneStop(void);   /* early abort -- result becomes NO_OSCILLATION unless a
                               result was already computed */
int  AutotuneIsActive(void);
AutotuneMethod AutotuneGetMethod(void);   /* always AUTOTUNE_METHOD_RELAY today -- kept so
                                              Protocol.c's `autotune.method` has a stable
                                              shape if a second method is ever reintroduced */
AutotuneResult AutotuneGetResult(void);

/* Only meaningful when AutotuneGetResult() == AUTOTUNE_RESULT_OK -- stale
 * (previous run's) otherwise. Gains are Q8.8 fixed point, same scale as
 * position.kp/velocity.kp/velocity.ki. */
int32_t  AutotuneGetKpResultQ8(void);
int32_t  AutotuneGetKiResultQ8(void);
int32_t  AutotuneGetPosPResultQ8(void);
uint32_t AutotuneGetMeasuredPeriodTicks(void);   /* diagnostic: Pu */
int32_t  AutotuneGetMeasuredGainQ8(void);        /* always 0 -- kept for Protocol.c's
                                                      `autotune.plant_gain` shape */

/* Progress snapshot, captured every ~0.5s (AUTOTUNE_RELAY_PROGRESS_INTERVAL_TICKS)
 * while a relay run is active -- a direct window into RelayTick()'s own state: how
 * far into the run, how many half-cycle switches it's counted against how many it
 * needs (AUTOTUNE_SETTLE_SWITCHES + AUTOTUNE_MEASURE_SWITCHES), and the peak
 * |velocity| reached so far this half-cycle -- the number to watch if a run is
 * heading for NO_OSCILLATION: a peak that never gets anywhere near
 * AUTOTUNE_HYSTERESIS_COUNTS means the mechanism isn't producing a velocity signal
 * relay can switch on at all, not that it just needs more time/a different
 * amplitude. Meaningless (all zero) outside an active relay run. ProgressSeq
 * increments each time a new snapshot is captured; compare against the last-seen
 * value to know whether there's something new to print rather than polling on a
 * wall-clock timer of its own (see main.c's AppAutotuneGetRelayProgressSeq() and
 * ProtocolPoll()). */
uint32_t AutotuneGetRelayProgressSeq(void);
uint32_t AutotuneGetRelayProgressTicksElapsed(void);
uint32_t AutotuneGetRelayProgressMaxTicks(void);
uint32_t AutotuneGetRelayProgressSwitchCount(void);
uint32_t AutotuneGetRelayProgressSwitchesNeeded(void);  /* AUTOTUNE_SETTLE_SWITCHES +
                                                             AUTOTUNE_MEASURE_SWITCHES, exposed
                                                             so a progress line can show
                                                             switches=N/needed without
                                                             duplicating that arithmetic */
int32_t  AutotuneGetRelayProgressPeakVelocity(void);    /* |velocity| peak so far THIS
                                                             half-cycle (resets at each switch,
                                                             same value ComputeRelayResult()
                                                             eventually averages) */

/* Called once per 1kHz control tick while active (main.c's AppControlTick,
 * BEFORE the normal enabled/signal dispatch -- same "bypasses the cascade"
 * precedence as polarity-learn/Signal.c). Returns the raw torque command for
 * this tick (0 once finished, whether by success, timeout, or explicit
 * stop). */
int32_t AutotuneTick(int32_t velocityActual);

#endif /* AUTOTUNE_H */
