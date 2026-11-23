#ifndef RLS_H
#define RLS_H

#include <stdint.h>
#include "Autotune.h"   /* reuses AutotuneResult (NONE/OK/BAD_MEASUREMENT) -- RLS and relay
                            report success/failure through the same vocabulary since
                            Protocol.c's `autotune apply` treats either engine's result
                            the same way. */

/* Online (continuous, in-move) velocity-loop self-tuning via forgetting-
 * factor recursive least squares (RLS) -- the second autotune method,
 * alongside the relay-feedback bootstrap in Autotune.c/Autotune.h.
 *
 * Where relay is a dedicated, exclusive bootstrap test (bypasses the normal
 * cascade, self-terminates in one bounded run), RLS is a near-passive
 * observer: RlsTick() is called every normal-cascade control tick with
 * whatever torque command and measured velocity the cascade already
 * produced. The ONE exception is RlsGetDitherTorque() (see "Torque dither"
 * below) -- a small, self-limiting perturbation added on top of that torque
 * command while still converging, not a cascade-bypassing takeover like
 * relay's. It can run across many moves, with pauses between them, updating
 * its estimate the whole time -- "start from nothing," converging over
 * however many moves it takes, rather than relay's single bounded
 * experiment.
 *
 * Model: a first-order per-tick velocity plant, v[n+1] = a*v[n] + b*u[n].
 * `a` is the plant's own per-tick velocity decay/persistence (friction's
 * effect); `b` is torque-command-to-velocity-step gain. Candidate Kp/Ki are
 * derived from (a, b) by proportional pole placement -- see RLS.c.
 *
 * Deliberately relaxed bar vs. the refine method Autotune.h's file comment
 * explains was kept OFF this project's device ("an abort/convergence
 * condition... easy to get wrong in a way that never terminates cleanly"):
 * RLS has no termination proof either. Instead of a proof, a physical clamp
 * on `a` (a passive mechanical system can't gain energy on its own, so
 * a <= 1.0 at ANY tick rate -- a hard, tick-rate-invariant bound), a
 * generous bench-agnostic clamp on `b`, AND an exact cascade-stability check
 * (RLS.c's CascadeStable(), which is why this file depends on Control.h --
 * a candidate's velocity-loop pole can be perfectly valid on its own and
 * still destabilize Control.c's existing, RLS-untouched position(P) loop
 * nested around it) stand in as the safety net: a candidate that clamps, or
 * fails that stability check, is rejected (AUTOTUNE_RESULT_BAD_MEASUREMENT),
 * never silently handed to the cascade.
 *
 * Tick-rate handling: `a`'s clamp is a physical constant (a<=1.0),
 * independent of tick rate. `b` and velocity are natively counts/tick in
 * this codebase (see Control.h); for a fixed physical hardware gain, `b`
 * scales with the SQUARE of the tick period (not linearly -- see RLS.c's
 * file comment on why), so RlsStart()'s tickHz argument is used once, at
 * start, to derive b's clamp bound from that relationship rather than
 * baking in a bound sized for whatever tick rate happens to be live today.
 * The pole-placement target is instead self-relative (derived from the
 * live `a` estimate itself, not an absolute time constant) and so needs no
 * tickHz conversion at all -- see MapThetaToGains() in RLS.c. lambdaQ8 is
 * taken as given: RLS.c does not rescale it for tickHz, so whatever picks
 * lambdaQ8 (today, a constant in main.c) is responsible for choosing a
 * value appropriate to the live tick rate if that ever varies.
 *
 * Torque dither: RlsTick() observes whatever torqueCommand Control.c's
 * cascade already computed from POSITION ERROR -- which makes torque and
 * velocity correlated through the control law itself, not independently
 * exciting. Ordinary least squares (RLS included) is well known to come out
 * BIASED on closed-loop data like that: found by this feature wrongly
 * accepting a lightly-damped plant's candidate as OK with `a` off by a
 * wide margin (0.934 estimated vs. the plant's true 0.999) and a corresponding
 * `b` three orders of magnitude too large -- a bias invisible to every
 * clamp above, since the biased estimate still looked perfectly plausible.
 * RlsGetDitherTorque() breaks that correlation with a small, zero-mean,
 * pseudo-random perturbation (RLS.c's DitherMix()) added to the torque
 * command on top of whatever the cascade computed -- it must reach the
 * REAL output (not just RLS's own math) to have any decorrelating effect,
 * so main.c adds it to torqueCommand before both OutputSetTorqueCommand()
 * and RlsTick() see it.
 *
 * Measured, not assumed: as little as 1% of torqueClampMax fully corrects
 * the bias in every case tried, and the result stays flat (same corrected
 * estimate, same tracking quality) all the way through 50% -- there is no
 * delicate threshold to tune. RLS.c's actual amplitude (~5%) picks
 * comfortable margin above that observed minimum, not a tightly-fit value.
 * Measured impact of leaving dither on indefinitely against an ALREADY-
 * converged, already-applied good tune (simmy's rls_integration harness,
 * identical move, same gains, 5% dither forced on vs. off): tracking itself
 * is within noise -- maxErrCruise 27 vs. 30 counts, RMS error 20.41 vs.
 * 20.22, pegTicks 0 either way -- so the documented argument that it would
 * be SAFE to leave on continuously (no settle-time/tracking regression) is
 * measured, not assumed. It is NOT free, though: RMS torque came out ~33%
 * higher with dither forced on (43 vs. 58 raw units) -- real extra
 * actuator effort/heat (and likely audible buzz) a user could notice, even
 * though tracking quality itself doesn't suffer. That cost, not merely
 * saving cycles, is the real reason RlsGetDitherTorque() turns itself off
 * once RlsGetResult()==AUTOTUNE_RESULT_OK, independent of whether
 * RlsIsActive() stays true -- and resumes on its own if a later tick's
 * estimate degrades back to BAD_MEASUREMENT. */

void RlsInit(void);

/* lambdaQ8: forgetting factor, Q8.8 (256 == 1.0, no forgetting -- values
 * below 256 forget older samples faster). tickHz: the live control tick
 * rate; used once here to scale every tick-rate-sensitive internal bound
 * (see file comment) -- pass the real rate, not a assumed 1000. */
void RlsStart(int32_t lambdaQ8, uint32_t tickHz);
void RlsStop(void);   /* freezes theta/P; candidate stays readable */
int  RlsIsActive(void);

/* Raises the target closed-loop pole past the default p_target=a^2 (see
 * RLS.c's file comment above RLS_POLE_AGGRESSIVENESS_MAX) -- each unit
 * squares the target once more (0=a^2 [default], 1=a^4, 2=a^8, 3=a^16,
 * 4=a^32, 5=a^64, 6=a^128; clamped to that range). Reach for this when the
 * plant's own open-loop pole estimate (RlsGetThetaAQ8()) sits close to 1
 * (lightly damped under the linear approximation) and the default a^2
 * target produces a candidate that converges (AUTOTUNE_RESULT_OK) but
 * tracks noticeably weaker than a known-good hand tune. Found and
 * measured against a real, weak-torque-authority axis model (see
 * simmy's examples/rls_axis_command_v2/): a full sweep traces a real
 * peak, not a plateau -- convergence rises from a^2 through a^64, then
 * CascadeStable's own margin starts rejecting overly aggressive targets
 * and convergence collapses past a^128. **5 (a^64) is the sweet spot**,
 * with one level of documented headroom below where it falls back off --
 * see RLS.c's file comment for the full per-level numbers. This is an
 * empirically-found knob, not a closed-form optimum: a different plant's
 * own peak could sit elsewhere, so treat 5 as a starting point to verify
 * against a given plant's own convergence/quality tradeoff, not a
 * universal constant. Costs nothing and changes nothing when left at the
 * default 0 -- every existing caller's behavior is unaffected unless it
 * calls this. Persists across RlsStart() (only RlsInit() resets it to 0),
 * since it's a property of the plant the caller already knows about, not
 * a per-attempt tuning value. */
void RlsSetPoleAggressiveness(int32_t extraSquarings);
int32_t RlsGetPoleAggressiveness(void);   /* returns the clamped value RlsSetPoleAggressiveness()
                                              actually stored, for telemetry/host readback */

/* Called once per normal-cascade control tick, regardless of RlsIsActive()
 * (cheap early-return when not running, so the feature costs ~nothing when
 * unused) -- torqueCommand/velocityActual are this tick's already-polarity-
 * corrected cascade output (ALREADY including RlsGetDitherTorque(), if the
 * caller added it as instructed below) and measured velocity. */
void RlsTick(int32_t torqueCommand, int32_t velocityActual);

/* Call once per control tick while RlsIsActive(), and add the result to
 * torqueCommand BEFORE both OutputSetTorqueCommand() and this tick's
 * RlsTick() call -- see "Torque dither" above for why it must reach the
 * real output, not just RLS's own bookkeeping. Returns exactly 0 whenever
 * dither isn't needed (not running, or already converged), so a caller that
 * always adds this in has nothing conditional to get wrong. */
int32_t RlsGetDitherTorque(void);

AutotuneResult RlsGetResult(void);

/* Only meaningful when RlsGetResult() == AUTOTUNE_RESULT_OK -- stale
 * (previous attempt's) otherwise. Q8.8, same scale as
 * position.kp/velocity.kp/velocity.ki. */
int32_t RlsGetKpResultQ8(void);
int32_t RlsGetKiResultQ8(void);

/* Diagnostics only -- not used by any control math. Q8.8. */
int32_t RlsGetThetaAQ8(void);
int32_t RlsGetThetaBQ8(void);
int64_t RlsGetPTrace(void);   /* p11+p22 (internal wide scale) -- cheap single-number
                                  "how converged" signal: large while uninformed,
                                  shrinks as the estimate locks on. */
uint32_t RlsGetTicksObserved(void);  /* ticks where the dead-zone guard did NOT skip
                                         the update -- i.e. ticks that actually taught
                                         the estimator something, not wall-clock ticks */
uint32_t RlsGetProgressSeq(void);    /* bumped each time a fresh OK candidate is
                                         computed, for progress polling (mirrors
                                         Autotune.h's relay progress sequence) */

#endif /* RLS_H */
