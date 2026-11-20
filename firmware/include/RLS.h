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
 * cascade, self-terminates in one bounded run), RLS is a passive observer:
 * RlsTick() is called every normal-cascade control tick with whatever torque
 * command and measured velocity the cascade already produced, and never
 * feeds back into that tick's torque output. It can run across many moves,
 * with pauses between them, updating its estimate the whole time -- "start
 * from nothing," converging over however many moves it takes, rather than
 * relay's single bounded experiment.
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
 * a <= 1.0 at ANY tick rate -- a hard, tick-rate-invariant bound) plus a
 * generous bench-agnostic clamp on `b` stand in as the safety net: a
 * candidate that clamps for a sustained window is rejected
 * (AUTOTUNE_RESULT_BAD_MEASUREMENT), never silently handed to the cascade.
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
 * value appropriate to the live tick rate if that ever varies. */

void RlsInit(void);

/* lambdaQ8: forgetting factor, Q8.8 (256 == 1.0, no forgetting -- values
 * below 256 forget older samples faster). tickHz: the live control tick
 * rate; used once here to scale every tick-rate-sensitive internal bound
 * (see file comment) -- pass the real rate, not a assumed 1000. */
void RlsStart(int32_t lambdaQ8, uint32_t tickHz);
void RlsStop(void);   /* freezes theta/P; candidate stays readable */
int  RlsIsActive(void);

/* Called once per normal-cascade control tick, regardless of RlsIsActive()
 * (cheap early-return when not running, so the feature costs ~nothing when
 * unused) -- torqueCommand/velocityActual are this tick's already-polarity-
 * corrected cascade output and measured velocity, the same values the
 * cascade itself just used; RLS never feeds back into them. */
void RlsTick(int32_t torqueCommand, int32_t velocityActual);

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
