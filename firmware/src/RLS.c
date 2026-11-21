/*
 * Online forgetting-factor RLS velocity-plant estimator. See RLS.h for the
 * model, the relaxed-safety-net rationale, and the tick-rate-handling
 * contract this file must honor.
 *
 * Internal fixed point: everything theta/P-shaped is carried in Q16.16
 * (RLS_SCALE), wider than the project's usual Q8.8, for covariance dynamic
 * range -- only the public Kp/Ki/theta getters truncate down to Q8.8 to
 * match velGainKpQ8/velGainKiQ8's existing scale. All arithmetic is
 * int64_t, same convention as Control.c's Clamp64/RoundQ16/sVelIntegralQ16.
 */
#include "RLS.h"
#include "Control.h"   /* ControlGetGains()->posGainQ8 only -- see CascadeStable() below.
                           Same portability class as GainGlide.c's existing Control.h
                           dependency: HAL-free, natively testable, no new link
                           dependency for [env:native] (Control.c is already linked
                           there for other test targets). */

#define RLS_SCALE        65536LL   /* Q16.16 */
#define RLS_Q8_SCALE     256LL

static int64_t Q8ToQ16(int32_t q8)  { return (int64_t)q8 * (RLS_SCALE / RLS_Q8_SCALE); }
static int32_t Q16ToQ8Round(int64_t q16) {
  int64_t scale = RLS_SCALE / RLS_Q8_SCALE;
  return (int32_t)(q16 >= 0 ? (q16 + scale / 2) / scale : -((-q16 + scale / 2) / scale));
}

static int64_t Clamp64(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/* Integer sqrt (Newton/Babylonian) -- used only to keep p12 PSD-consistent
 * with p11/p22 after clamping, see ClampCovariance() below. x is assumed
 * non-negative (callers only ever pass a clamped, already-non-negative
 * p11/p22). */
static int64_t ISqrt64(int64_t x) {
  if (x < 2) return x < 0 ? 0 : x;
  int64_t r = x;
  int64_t y = (r + 1) / 2;
  while (y < r) {
    r = y;
    y = (r + x / r) / 2;
  }
  return r;
}

/* a <= 1.0 is a hard physical bound (no passive mechanical system gains
 * energy on its own) at ANY tick rate -- a true constant, never derived
 * from tickHz. aMin is deliberately a loose "reject nonsense" floor, not a
 * tight bound either, so it's safe to leave as a constant too. */
#define RLS_THETA_A_MAX_Q16   (RLS_SCALE)            /* 1.0 */
#define RLS_THETA_A_MIN_Q16   (-(RLS_SCALE / 4))      /* -0.25, generous */

/* b (torque-command -> velocity-step gain) is NOT tick-rate invariant: for
 * a fixed physical torque-to-acceleration gain K, discretizing a tick of
 * period T gives b ~= K*T^2 (quadratic, not linear -- a single-integrator
 * plant's per-tick velocity step from a tick-long constant acceleration
 * scales with T once for the acceleration's duration and once more because
 * velocity itself is stored as counts/tick = counts/sec * T here, see
 * Control.h). Anchoring a placeholder bound at a reference rate and
 * scaling by (refHz/tickHz)^2 avoids needing to know K, tau, or any other
 * real SI hardware constant up front -- exactly the "start from nothing"
 * requirement. RLS_B_MAX_AT_REF_Q8 is a deliberately generous, bench-TBD
 * guess, not a measured value. */
#define RLS_B_MAX_REF_HZ       1000U
#define RLS_B_MAX_AT_REF_Q8    64     /* ~0.25 @ 1kHz -- placeholder, bench-TBD */
#define RLS_B_MIN_Q16          (RLS_SCALE / RLS_Q8_SCALE)  /* ~0.0039 -- numerical divide-by-zero
                                         guard AND outer sanity floor in one: after polarity
                                         correction b should be meaningfully positive, so
                                         anything this close to zero is untrustworthy either way */

/* Target closed-loop pole for proportional pole placement, p_target = a^2
 * -- relative to the just-measured open-loop pole `a` itself, NOT an
 * absolute time constant. An absolute target (e.g. a fixed tau in ms)
 * would need to be faster than whatever the real, unknown-in-advance plant
 * turns out to be, or Kp = (a - p_target)/b comes out negative and every
 * candidate gets rejected -- exactly backwards for "start from nothing."
 * Since 0 < a < 1 for any plant this clamp range accepts, a^2 < a always,
 * guaranteeing a faster (tighter) closed-loop pole than open-loop by
 * construction, with no absolute units and no tickHz dependence needed at
 * all here -- `a` is already tick-rate-correct by the time this runs. */

/* Dead-zone thresholds: U_MIN is a static torque AMPLITUDE (a DAC/PWM code)
 * -- not a rate, tick-rate invariant, stays a literal. V_MIN looks like it
 * should scale with tick rate (same "counts/tick is a rate" argument as b,
 * above) but doesn't in practice: it exists only to catch the
 * exactly/near-zero reading during a real pause between moves, the same
 * quantization-noise-floor role Autotune.c's own AUTOTUNE_HYSTERESIS_COUNTS
 * (=2, also NOT tick-rate-scaled) already plays -- "stopped" reads as ~0
 * counts/tick regardless of how fast ticks arrive, so a small literal
 * threshold is correct at any tick rate, not an oversight. */
#define RLS_U_MIN   50
#define RLS_V_MIN   2

#define RLS_PROGRESS_INTERVAL_MS 500U   /* same ~0.5s cadence as Autotune.c's relay progress */

/* Classic RLS covariance can still grow without bound under one-sided
 * excitation (u varies but v doesn't, or vice versa) even outside the
 * all-quiet dead zone above -- a known RLS wrinkle, not something the dead
 * zone alone catches. A generous ceiling here is pure insurance against
 * int64 overflow/numerical garbage over a long unattended run, not a tuned
 * bound -- same "clamp instead of a convergence proof" philosophy as the
 * theta clamps, applied to P too. */
#define RLS_P_MAX_Q16 (1000000LL * RLS_SCALE)

static int64_t sThetaAQ16, sThetaBQ16;
static int64_t sP11Q16, sP12Q16, sP22Q16;
static int64_t sLambdaQ16;
static int64_t sBMaxQ16;
static uint32_t sProgressIntervalTicks, sProgressCountdown;

static int32_t sVPrev, sUPrev;
static int sHavePrev;

static int sRunning;
static AutotuneResult sResult;
static int32_t sKpResultQ8, sKiResultQ8;
static uint32_t sTicksObserved;
static uint32_t sProgressSeq;
static uint32_t sDitherState;

/* ~5% of torqueClampMax, Q8.8 (13/256=0.0508) -- see RLS.h's "Torque
 * dither" section. Empirically, as little as 1% already fully corrects the
 * closed-loop identification bias this exists to fix, and the result is
 * flat from there through 50%, so this picks comfortable margin above the
 * observed minimum rather than a tightly-tuned value. */
#define RLS_DITHER_PCT_Q8 13

/* Murmur3-style 32-bit finalizer (xor-shift / multiply / xor-shift),
 * chained across calls by folding in a golden-ratio increment each time --
 * deterministic, non-cryptographic, just needs to scatter a running counter
 * into something that doesn't read as "a small number plus noise" from one
 * tick to the next, which is all a decorrelating dither signal needs. */
static uint32_t DitherMix(uint32_t x) {
  x ^= x >> 16;
  x *= 0x85EBCA6Bu;
  x ^= x >> 13;
  x *= 0xC2B2AE35u;
  x ^= x >> 16;
  return x;
}

static int32_t DitherSample(int32_t amplitude) {
  if (amplitude <= 0) return 0;
  sDitherState = DitherMix(sDitherState + 0x9E3779B9u);
  uint32_t span = (uint32_t)(2 * amplitude + 1);
  return (int32_t)(sDitherState % span) - amplitude;
}

void RlsInit(void) {
  sThetaAQ16 = sThetaBQ16 = 0;
  sP11Q16 = sP22Q16 = 0;
  sP12Q16 = 0;
  sLambdaQ16 = RLS_SCALE;
  sBMaxQ16 = 0;
  sProgressIntervalTicks = sProgressCountdown = 0U;
  sVPrev = sUPrev = 0;
  sHavePrev = 0;
  sRunning = 0;
  sResult = AUTOTUNE_RESULT_NONE;
  sKpResultQ8 = sKiResultQ8 = 0;
  sTicksObserved = 0U;
  sProgressSeq = 0U;
  sDitherState = 2463534242U;   /* fixed nonzero seed -- deterministic reproducibility is a
                                    feature here (tests, bug reports), not a shortcoming: the
                                    dither only needs to decorrelate from Control.c's feedback,
                                    never to be unpredictable across power cycles */
}

void RlsStart(int32_t lambdaQ8, uint32_t tickHz) {
  if (tickHz == 0U) tickHz = 1000U;   /* defensive -- never divide by zero below */

  /* Uninformed prior: start from nothing, as required. P0 is a generous
   * "we know nothing" covariance -- large enough to let the first real
   * excitation move theta quickly, not a bench-tuned value. */
  sThetaAQ16 = 0;
  sThetaBQ16 = 0;
  sP11Q16 = sP22Q16 = 1000LL * RLS_SCALE;
  sP12Q16 = 0;
  sLambdaQ16 = Q8ToQ16(lambdaQ8);

  /* b's outer sanity bound, anchored at RLS_B_MAX_REF_HZ and scaled by
   * (refHz/tickHz)^2 -- see the file-comment derivation above. */
  int64_t ratioQ16 = ((int64_t)RLS_B_MAX_REF_HZ * RLS_SCALE) / (int64_t)tickHz;
  int64_t ratioSqQ16 = (ratioQ16 * ratioQ16) / RLS_SCALE;
  sBMaxQ16 = (Q8ToQ16(RLS_B_MAX_AT_REF_Q8) * ratioSqQ16) / RLS_SCALE;

  sProgressIntervalTicks = ((uint64_t)RLS_PROGRESS_INTERVAL_MS * tickHz) / 1000U;
  if (sProgressIntervalTicks == 0U) sProgressIntervalTicks = 1U;
  sProgressCountdown = sProgressIntervalTicks;

  sVPrev = sUPrev = 0;
  sHavePrev = 0;
  sResult = AUTOTUNE_RESULT_NONE;
  sKpResultQ8 = sKiResultQ8 = 0;
  sTicksObserved = 0U;
  sRunning = 1;
}

void RlsStop(void) { sRunning = 0; }
int  RlsIsActive(void) { return sRunning; }
AutotuneResult RlsGetResult(void) { return sResult; }
int32_t  RlsGetKpResultQ8(void) { return sKpResultQ8; }
int32_t  RlsGetKiResultQ8(void) { return sKiResultQ8; }
int32_t  RlsGetThetaAQ8(void) { return Q16ToQ8Round(sThetaAQ16); }
int32_t  RlsGetThetaBQ8(void) { return Q16ToQ8Round(sThetaBQ16); }
int64_t  RlsGetPTrace(void) { return sP11Q16 + sP22Q16; }
uint32_t RlsGetTicksObserved(void) { return sTicksObserved; }

int32_t RlsGetDitherTorque(void) {
  /* Gated on whether the estimate is trustworthy (sResult==OK), NOT on
   * whether RLS is still running -- the two are deliberately independent.
   * A run that's still RlsIsActive() but has already converged doesn't
   * need to keep perturbing the real output; a run that later degrades
   * back to BAD_MEASUREMENT (e.g. the mechanism's own dynamics genuinely
   * changed) resumes dithering on its own, with no separate restart. This
   * turns off once unnecessary: not because leaving it on continuously
   * would cause any tracking/settle-time problem (measured -- it doesn't),
   * but because it does measurably cost real actuator effort/heat (~33%
   * higher RMS torque in that same measurement) for no further benefit
   * once converged -- see RLS.h's "Torque dither" section. */
  if (!sRunning || sResult == AUTOTUNE_RESULT_OK) return 0;
  int32_t torqueMax = ControlGetGains()->torqueClampMax;
  int64_t amplitude = ((int64_t)torqueMax * RLS_DITHER_PCT_Q8) / 256;
  return DitherSample((int32_t)amplitude);
}
uint32_t RlsGetProgressSeq(void) { return sProgressSeq; }

/* Cascade stability gate: p_target=a^2 places the VELOCITY loop's own pole,
 * but that loop runs nested inside Control.c's existing position(P) loop
 * (posGainQ8) -- a loop RLS never touches (see RLS.h). p_target alone says
 * nothing about whether that combination is safe: on a lightly-damped plant
 * (a close to 1), p_target stays close to 1 too, which can leave the outer
 * position loop driving a velocity loop that isn't meaningfully faster than
 * open-loop, producing a weakly-damped (or outright unstable) 3-state
 * cascade [position error, velocity, PI integrator] once actually applied --
 * found by exercising a real nested cascade (not this file's own simplified
 * single-loop view) against a near-frictionless simulated plant.
 *
 * The exact stability condition turns out to depend ONLY on the plant pole
 * `a` and Kpos=posGainQ8/256 -- NOT on b, which cancels out completely once
 * Kp/Ki are derived from p_target (substituting Kp=(a-q)/b, Ki=Kp*(1-q) into
 * the closed-loop state matrix and rescaling the integrator state by b
 * removes every occurrence of b -- verified both algebraically and
 * numerically). That leaves the Jury/Schur-Cohn stability test for the
 * resulting cubic characteristic polynomial z^3+c2*z^2+c1*z+c0:
 *   c2 = -a*q + a + q^2 - 2*q - 2
 *   c1 = -Kpos*a*q + 2*Kpos*a + Kpos*q^2 - 2*Kpos*q + a*q - a - q^2 + 3*q + 1
 *   c0 = -(Kpos*(a-q) + q)
 * stable iff: P(-1)=-1+c2-c1+c0 < 0  AND  |c0|<1  AND  |c1-c0*c2| < 1-c0^2
 * (verified against 20000 random (a,q,Kpos) samples with zero mismatches
 * against direct eigenvalue computation). A 4th Jury condition,
 * P(1)=Kpos*(1-q)*(a-q)>0, holds automatically given the 0<q<a<1 range
 * MapThetaToGains() already enforces below, so it isn't checked separately.
 * Solving this FOR p_target given an arbitrary Kpos is a quartic with no
 * tractable closed form -- not attempted here -- but CHECKING whether a
 * given (a, p_target, Kpos) is safe is just evaluating these three
 * polynomials and comparing signs, cheap enough to run every tick.
 *
 * Reads posGainQ8 LIVE via ControlGetGains() every call, not a value
 * snapshotted at RlsStart() time, since `set position.kp ...` can change it
 * while RLS is still converging -- the check must track whatever position
 * loop a candidate would actually be applied into. */
static int CascadeStable(int64_t aQ16, int64_t qQ16) {
  /* Clamped defensively for fixed-point product safety below, not as a
   * policy limit on position.kp itself (Protocol.c/ConfigStore already own
   * that limit) -- a Kpos this large is already far into "every candidate
   * gets rejected anyway" territory, so clamping here changes no real
   * outcome. */
  int64_t KposQ16 = Clamp64(Q8ToQ16(ControlGetGains()->posGainQ8), 0, 16LL * RLS_SCALE);

  int64_t aq = (aQ16 * qQ16) / RLS_SCALE;
  int64_t q2 = (qQ16 * qQ16) / RLS_SCALE;
  int64_t c2 = -aq + aQ16 + q2 - 2 * qQ16 - 2 * RLS_SCALE;

  int64_t kposA  = (KposQ16 * aQ16) / RLS_SCALE;
  int64_t kposQ  = (KposQ16 * qQ16) / RLS_SCALE;
  int64_t kposAq = (kposA * qQ16) / RLS_SCALE;
  int64_t kposQ2 = (kposQ * qQ16) / RLS_SCALE;
  int64_t c1 = -kposAq + 2 * kposA + kposQ2 - 2 * kposQ + aq - aQ16 - q2 + 3 * qQ16 + RLS_SCALE;

  int64_t c0 = -(kposA - kposQ + qQ16);

  int64_t pMinus1 = -RLS_SCALE + c2 - c1 + c0;
  if (pMinus1 >= 0) return 0;

  int64_t absC0 = c0 < 0 ? -c0 : c0;
  if (absC0 >= RLS_SCALE) return 0;

  int64_t c0c2 = (c0 * c2) / RLS_SCALE;
  int64_t c0sq = (c0 * c0) / RLS_SCALE;
  int64_t lhs = c1 - c0c2; if (lhs < 0) lhs = -lhs;
  int64_t rhs = RLS_SCALE - c0sq; if (rhs < 0) rhs = -rhs;
  return lhs < rhs;
}

/* Proportional pole placement: Kp = (a - p_target)/b, p_target = a^2 (see
 * the file comment above MapThetaToGains's p_target definition for why
 * it's relative, not absolute). Ki from a tempered rule analogous to
 * relay's Ki = Kp/Ti (see Autotune.c), using (1 - p_target) as the Ti
 * factor. Guards |b| against RLS_B_MIN_Q16 before dividing -- skips the
 * recompute (candidate stays stale) rather than ever dividing by something
 * too close to zero to trust. Returns 0 and leaves kpQ8/kiQ8 untouched if
 * any guard trips or either theta is out of its clamp range (caller is
 * responsible for then reporting BAD_MEASUREMENT). */
static int MapThetaToGains(int32_t *kpQ8, int32_t *kiQ8) {
  if (sThetaAQ16 <= 0 || sThetaAQ16 > RLS_THETA_A_MAX_Q16) return 0;   /* a<=0: not enough
                                         information yet to place a pole against, not a clamp
                                         violation in the RLS_THETA_A_MIN_Q16 sense -- see
                                         RLS_THETA_A_MIN_Q16's own comment for that distinction */
  if (sThetaBQ16 < RLS_B_MIN_Q16 || sThetaBQ16 > sBMaxQ16) return 0;

  int64_t pTargetQ16 = (sThetaAQ16 * sThetaAQ16) / RLS_SCALE;   /* a^2, Q16.16 */
  if (!CascadeStable(sThetaAQ16, pTargetQ16)) return 0;   /* see CascadeStable()'s comment --
                                                               depends only on a and posGainQ8,
                                                               checked before touching b at all */
  int64_t numQ16 = sThetaAQ16 - pTargetQ16;           /* a - a^2 = a*(1-a), Q16.16, > 0 here */
  int64_t kpQ16 = (numQ16 * RLS_SCALE) / sThetaBQ16;  /* (a - p_target)/b, Q16.16 */
  if (kpQ16 <= 0) return 0;                            /* non-physical candidate -- reject */

  int64_t tiFactorQ16 = RLS_SCALE - pTargetQ16;        /* 1 - a^2, Q16.16, always in (0,1] here */
  int64_t kiQ16 = (kpQ16 * tiFactorQ16) / RLS_SCALE;

  int32_t kp = Q16ToQ8Round(kpQ16);
  int32_t ki = Q16ToQ8Round(kiQ16);
  if (kp <= 0 || ki <= 0) return 0;

  *kpQ8 = kp;
  *kiQ8 = ki;
  return 1;
}

static void CaptureProgress(void) {
  if (sProgressCountdown == 0U) return;
  sProgressCountdown--;
  if (sProgressCountdown != 0U) return;
  sProgressCountdown = sProgressIntervalTicks;
  sProgressSeq++;
}

void RlsTick(int32_t torqueCommand, int32_t velocityActual) {
  if (!sRunning) return;

  int32_t u = torqueCommand;
  int32_t v = velocityActual;

  if (!sHavePrev) {
    /* First observed sample has no prior (v_prev, u_prev) to form a
     * regressor against -- just seed and wait for the next tick. */
    sVPrev = v;
    sUPrev = u;
    sHavePrev = 1;
    return;
  }

  int32_t absU = (sUPrev < 0) ? -sUPrev : sUPrev;
  int32_t absV = (sVPrev < 0) ? -sVPrev : sVPrev;
  int32_t absUNow = (u < 0) ? -u : u;
  int32_t absVNow = (v < 0) ? -v : v;
  int quiet = (absU < RLS_U_MIN && absV < RLS_V_MIN) && (absUNow < RLS_U_MIN && absVNow < RLS_V_MIN);

  if (quiet) {
    /* Covariance wind-up guard: skip the P/theta update entirely during an
     * extended pause (phi ~= 0) rather than let P grow unboundedly against
     * no real information -- see RLS.h's file comment. ticksObserved is
     * deliberately NOT incremented here: it counts ticks that actually
     * taught the estimator something. */
    sVPrev = v;
    sUPrev = u;
    return;
  }

  /* phi = [v_prev, u_prev]; predict this tick's v from the PREVIOUS
   * sample, matching the model v[n+1] = a*v[n] + b*u[n]. */
  int64_t predictedQ16 = sThetaAQ16 * sVPrev + sThetaBQ16 * sUPrev;
  int64_t yQ16 = (int64_t)v * RLS_SCALE;
  int64_t errQ16 = yQ16 - predictedQ16;

  /* P*phi */
  int64_t pPhi1Q16 = sP11Q16 * sVPrev + sP12Q16 * sUPrev;
  int64_t pPhi2Q16 = sP12Q16 * sVPrev + sP22Q16 * sUPrev;
  /* phi^T * P * phi */
  int64_t phiPPhiQ16 = sVPrev * pPhi1Q16 + sUPrev * pPhi2Q16;
  int64_t denomQ16 = sLambdaQ16 + phiPPhiQ16;
  if (denomQ16 <= 0) {
    /* Shouldn't happen (P is positive semi-definite by construction, lambda
     * > 0) -- defensively skip rather than divide by something non-positive. */
    sVPrev = v;
    sUPrev = u;
    return;
  }

  int64_t k1Q16 = (pPhi1Q16 * RLS_SCALE) / denomQ16;
  int64_t k2Q16 = (pPhi2Q16 * RLS_SCALE) / denomQ16;

  sThetaAQ16 += (k1Q16 * errQ16) / RLS_SCALE;
  sThetaBQ16 += (k2Q16 * errQ16) / RLS_SCALE;

  /* P = (P - K*phi^T*P) / lambda */
  int64_t newP11Q16 = (sP11Q16 - (k1Q16 * pPhi1Q16) / RLS_SCALE);
  int64_t newP12Q16 = (sP12Q16 - (k1Q16 * pPhi2Q16) / RLS_SCALE);
  int64_t newP22Q16 = (sP22Q16 - (k2Q16 * pPhi2Q16) / RLS_SCALE);
  /* p11/p22 (the diagonal) can never be negative in exact arithmetic -- a
   * variance is never negative. Fixed-point division truncation can still
   * drift one slightly below zero after enough ticks; left unclamped, a
   * negative P then divided by lambda<1 (below) runs away MORE negative
   * every subsequent tick -- the exact pathological growth the dead-zone
   * guard above exists to prevent, just reached by rounding error instead
   * of a quiet period. Floor at 0, not just the symmetric overflow
   * ceiling, to cut that off at the source. p12 has no such sign
   * constraint (it's a covariance, not a variance). */
  sP11Q16 = Clamp64((newP11Q16 * RLS_SCALE) / sLambdaQ16, 0, RLS_P_MAX_Q16);
  sP12Q16 = Clamp64((newP12Q16 * RLS_SCALE) / sLambdaQ16, -RLS_P_MAX_Q16, RLS_P_MAX_Q16);
  sP22Q16 = Clamp64((newP22Q16 * RLS_SCALE) / sLambdaQ16, 0, RLS_P_MAX_Q16);

  /* A valid 2x2 covariance requires p12^2 <= p11*p22 (positive
   * semi-definiteness) -- clamping p11/p22 to >=0 above without also
   * re-bounding p12 can leave an inconsistent matrix (e.g. p11=p22=0 but
   * p12 still large), which produces nonsensical K/theta updates on the
   * very next tick and was observed to blow theta up within tens of
   * ticks. ISqrt64(p11)*ISqrt64(p22) bounds |p12| the same as
   * sqrt(p11*p22) would, computed this way specifically to avoid
   * overflowing int64_t when p11/p22 are each up near RLS_P_MAX_Q16. */
  int64_t p12BoundQ16 = ISqrt64(sP11Q16) * ISqrt64(sP22Q16);
  sP12Q16 = Clamp64(sP12Q16, -p12BoundQ16, p12BoundQ16);

  sVPrev = v;
  sUPrev = u;
  sTicksObserved++;

  int32_t kpQ8, kiQ8;
  if (MapThetaToGains(&kpQ8, &kiQ8)) {
    sKpResultQ8 = kpQ8;
    sKiResultQ8 = kiQ8;
    sResult = AUTOTUNE_RESULT_OK;
    CaptureProgress();
  } else {
    sResult = AUTOTUNE_RESULT_BAD_MEASUREMENT;   /* current estimate untrustworthy right now --
                                                     not terminal, may recover to OK on a later
                                                     tick as more data arrives (see RLS.h) */
  }
}
