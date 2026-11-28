/*
 * Native unit tests for Mode A (step/dir) position tracking. Mode A has no
 * ramp of its own (main.c:466-472) -- it just adds the raw per-tick step
 * delta straight onto the position-target accumulator and feeds that into
 * Control.c's P->PI cascade, so "does it move smoothly at constant speed"
 * is really a question about how that cascade tracks a QUANTIZED target:
 * a real step/dir pulse train sampled at the 1kHz tick rate beats against
 * that rate (bursts of 0/1/2 counts/tick, not a smooth stream), and this
 * suite checks that a representative tune tracks that quantization -- and
 * short-term pulse-timing jitter on top of it -- with a bounded following
 * error and no sudden torque/velocity jumps or growing oscillation.
 *
 * What this can't reach: StepDir.c's actual TIM2 edge-counting (HAL-
 * dependent, see StepDir.h's own "UNVERIFIED WITHOUT THE DATASHEET" note)
 * and real electrical pulse jitter -- those need a real pulse source on
 * the bench, not a native test. PulseTrain.h (test/support) models only
 * the quantized edge-COUNT a real pulse train would produce.
 *
 * Pure fixed-point math plus a toy discrete plant, no HAL dependency --
 * natively testable. Run: pio test -e native
 */
#include <unity.h>
#include "Control.h"
#include "PulseTrain.h"
#include "ToyPlant.h"
#include "MachinePresets.h"
#include "main.h"
#include "Encoder.h"
#include "Output.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Fault.c and Protocol.c (and everything they call into) get
 * linked into every test/test_* executable, this one included. See
 * platformio.ini and test_control.c for the same pattern. */
void     OutputForceSafe(void) {}
ServoMode AppGetServoMode(void) { return SERVO_MODE_STEPDIR; }
int       AppSetServoMode(ServoMode m) { (void)m; return 0; }
int       AppGetEnabled(void) { return 0; }
void      AppSetEnabled(int e) { (void)e; }
int32_t   AppGetCommandedVelocity(void) { return 0; }
void      AppSetCommandedVelocity(int32_t v) { (void)v; }
int32_t   AppGetPositionCmd(void) { return 0; }
void      AppSetPositionCmd(int32_t t) { (void)t; }
static ConfigData sUnusedConfig;
const ConfigData *AppGetConfig(void) { return &sUnusedConfig; }
static FaultFreezeFrame sUnusedFreeze;
int                     AppFaultFreezeExists(void) { return 0; }
const FaultFreezeFrame *AppGetFaultFreeze(void) { return &sUnusedFreeze; }
void AppSetGains(int32_t p, int32_t kp, int32_t ki) { (void)p; (void)kp; (void)ki; }
void AppSetTorqueClamp(int32_t mn, int32_t mx) { (void)mn; (void)mx; }
void AppSetVelocityClamp(int32_t c) { (void)c; }
void AppSetRampAccel(int32_t a) { (void)a; }
void AppSetFollowingErrorConfig(int32_t limit, int hard) { (void)limit; (void)hard; }
void AppSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks) { (void)minTorque; (void)windowTicks; }
void AppSetZeroTrim(int32_t trim) { (void)trim; }
void    AppSetAnalogVelocityConfig(int32_t scaleQ8, int32_t hysteresisCounts) { (void)scaleQ8; (void)hysteresisCounts; }
int32_t AppGetAnalogRawAdc(void) { return 0; }
int     AppGetAnalogReversed(void) { return 0; }
int     AppGetHwEnableAsserted(void) { return 1; }
uint32_t AppGetLastTickCycles(void) { return 0; }
uint32_t AppGetMaxTickCycles(void) { return 0; }
int32_t  AppGetLastTickLoadPct(void) { return 0; }
uint32_t AppGetBuildId(const uint8_t **id) { (void)id; return 0; }
int  AppConfigSave(void) { return -1; }
int  AppConfigLoad(void) { return -1; }
void AppConfigDefaults(void) {}
void AppPolarityLearn(void) {}
int  AppPolarityLearnBusy(void) { return 0; }
int     AppGetPolarityLearnFailed(void) { return 0; }
int32_t AppGetPolarityLearnMovedCounts(void) { return 0; }
void    AppAutotuneStartRelay(int32_t a, uint32_t m) { (void)a; (void)m; }
void    AppAutotuneStop(void) {}
int     AppAutotuneBusy(void) { return 0; }
AutotuneMethod AppAutotuneGetMethod(void) { return AUTOTUNE_METHOD_RELAY; }
AutotuneResult AppAutotuneGetResult(void) { return AUTOTUNE_RESULT_NONE; }
int32_t  AppAutotuneGetKpResultQ8(void) { return 0; }
int32_t  AppAutotuneGetKiResultQ8(void) { return 0; }
int32_t  AppAutotuneGetPosPResultQ8(void) { return 0; }
uint32_t AppAutotuneGetMeasuredPeriodTicks(void) { return 0; }
int32_t  AppAutotuneGetMeasuredGainQ8(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressSeq(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressTicksElapsed(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressMaxTicks(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressSwitchCount(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressSwitchesNeeded(void) { return 0; }
int32_t  AppAutotuneGetRelayProgressPeakVelocity(void) { return 0; }
void    AppRlsStart(int32_t lambdaQ8, uint32_t glideMs) { (void)lambdaQ8; (void)glideMs; }
void    AppRlsStop(void) {}
int     AppRlsBusy(void) { return 0; }
AutotuneResult AppRlsGetResult(void) { return AUTOTUNE_RESULT_NONE; }
int32_t  AppRlsGetKpResultQ8(void) { return 0; }
int32_t  AppRlsGetKiResultQ8(void) { return 0; }
int32_t  AppRlsGetThetaAQ8(void) { return 0; }
int32_t  AppRlsGetThetaBQ8(void) { return 0; }
int64_t  AppRlsGetPTrace(void) { return 0; }
uint32_t AppRlsGetTicksObserved(void) { return 0; }
uint32_t AppRlsGetProgressSeq(void) { return 0; }
void     AppRlsSetAggressiveness(int32_t extraSquarings) { (void)extraSquarings; }
int32_t  AppRlsGetAggressiveness(void) { return 0; }
void     AppRlsApply(void) {}
void     AppAutotuneApply(void) {}
void       AppSignalStart(SignalType t, SignalTarget tg, int32_t amp, uint32_t hp, uint32_t dur) {
  (void)t; (void)tg; (void)amp; (void)hp; (void)dur;
}
void       AppSignalStop(void) {}
int        AppGetSignalActive(void) { return 0; }
SignalType   AppGetSignalType(void) { return SIGNAL_NONE; }
SignalTarget AppGetSignalTarget(void) { return SIGNAL_TARGET_TORQUE; }
int AppResetPosition(void) { return 0; }
void AppGetControlSample(AppControlSample *out) { *out = (AppControlSample){0}; }
int32_t AppGetLastVelocityActual(void) { return 0; }
int32_t AppGetLastTorqueCommand(void) { return 0; }
int32_t AppGetLastCommandedPosition(void) { return 0; }
int32_t AppGetLastCommandedVelocity(void) { return 0; }
int32_t AppGetLastElectricalAngleDeg(void) { return 0; }
int32_t AppGetLastHallSector(void) { return 0; }
int        AppGetSignalCycleStart(void) { return 0; }
int32_t EncoderGetPositionCounts(void) { return 0; }
int     EncoderZWasSeenSinceEnable(void) { return 0; }
/* output.topology / commutation.* stubs -- Protocol.c (linked into every
 * native test, see the comment above) calls these for its
 * output.topology/commutation.* CLI commands; this suite never exercises
 * two-phase output, so degenerate stand-ins are enough, same pattern as
 * test_hall.c/test_commutation.c. */
OutputTopology AppGetOutputTopology(void) { return OUTPUT_TOPOLOGY_ANALOG_TORQUE; }
int            AppSetOutputTopology(OutputTopology t) { (void)t; return 0; }
void    AppSetCommutationConfig(int32_t p, int32_t c) { (void)p; (void)c; }
void    AppCommutationLearn(void) {}
int     AppCommutationLearnBusy(void) { return 0; }
int     AppGetCommutationLearnFailed(void) { return 0; }
void    AppCommutationDetectPoles(int32_t encoderCpr) { (void)encoderCpr; }
int     AppCommutationDetectPolesBusy(void) { return 0; }
int     AppGetCommutationDetectPolesFailed(void) { return 0; }
void    UsartTransmit(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { (void)Buf; (void)Len; return USBD_OK; }
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel, uint8_t *pbuff, uint16_t length) { (void)pdev; (void)channel; (void)pbuff; (void)length; return USBD_OK; }
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) { (void)pdev; (void)channel; return USBD_OK; }

#ifndef SYSCLK_HZ
#error "SYSCLK_HZ must be defined (see platformio.ini's native env)"
#endif

/* A representative, already-reasonable tune -- same gains test_control.c's
 * own setUp() uses for its closed-loop convergence tests. Not a claim that
 * this is THE correct tune for any real plant; the point is to pin down
 * that a tune already accepted as reasonable keeps tracking a quantized/
 * jittered step/dir input the same way it tracks a plain step target. */
static void SetUpReasonableGains(void) {
  ControlGains g = {
    .posGainQ8 = 256,
    .velGainKpQ8 = 512,
    .velGainKiQ8 = 26,
    .velocityClamp = 100000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
  };
  ControlInit(&g);
}

void setUp(void) { SetUpReasonableGains(); }
void tearDown(void) {}

static const uint32_t kDt = SYSCLK_HZ / 1000U;   /* ~1ms, the real control-tick cadence */
static const double   kTickHz = 1000.0;
static const uint32_t kSeed = 12345u;

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

typedef struct {
  int32_t maxErrFullRun;      /* bounded-ness over the whole run, transient included */
  int32_t maxErrSteadyState;  /* last quarter only -- settled tracking */
  int32_t maxJumpSteadyState; /* last quarter only -- no sudden torque jumps once settled */
} PulseTrainTrialResult;

/* Drives Control.c's real cascade with a synthetic step/dir pulse train,
 * through the SAME accumulation main.c:470 does for real Mode A
 * (posTargetAccum += delta each tick), against the toy discrete plant
 * test_control.c's RunPlant() uses (velocity integrates torque, position
 * integrates velocity) -- just enough closed-loop dynamics to tell
 * "tracks" from "doesn't," not a claim about the real electromechanical
 * plant. */
static PulseTrainTrialResult RunPulseTrainTrial(double rateEdgesPerSec, double jitterFraction,
                                                 int ticks) {
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);

  int32_t posTargetAccum = 0, pos = 0, vel = 0, prevTorque = 0;
  PulseTrainTrialResult result = {0, 0, 0};

  for (int i = 0; i < ticks; i++) {
    int32_t delta = PulseTrainTickDelta(&pt, rateEdgesPerSec, kTickHz, jitterFraction);
    posTargetAccum += delta;
    int32_t torque = ControlTick(posTargetAccum, pos, vel, kDt);
    int32_t err = Abs32(posTargetAccum - pos);
    int32_t jump = Abs32(torque - prevTorque);
    prevTorque = torque;

    if (err > result.maxErrFullRun) result.maxErrFullRun = err;
    if (i >= ticks * 3 / 4) {
      if (err > result.maxErrSteadyState) result.maxErrSteadyState = err;
      if (jump > result.maxJumpSteadyState) result.maxJumpSteadyState = jump;
    }

    vel += torque / 16;
    pos += vel;
  }
  return result;
}

/* ---- clean pulse train -- tight bounds, two representative rates: one
 * well below the 1kHz tick rate (mostly 0/1-count ticks), one well above
 * it (mostly 2/3-count ticks) -- different quantization beat patterns,
 * same expected behavior. Thresholds calibrated against this exact
 * scenario (2000 ticks, seed 12345): observed max following error <=12,
 * max steady-state torque jump <=8 at both rates -- bounds below give
 * several times that margin while still catching a real regression (a
 * deliberately-too-aggressive tune on this same plant rings against the
 * torque clamp, producing steady-state jumps of ~2000, nowhere near these
 * bounds). ---------------------------------------------------------- */

static void test_clean_pulse_train_slow_rate_tracks_without_jumps(void) {
  PulseTrainTrialResult r = RunPulseTrainTrial(200.0, 0.0, 2000);
  TEST_ASSERT_TRUE(r.maxErrFullRun <= 30);
  TEST_ASSERT_TRUE(r.maxErrSteadyState <= 20);
  TEST_ASSERT_TRUE(r.maxJumpSteadyState <= 15);
}

static void test_clean_pulse_train_fast_rate_tracks_without_jumps(void) {
  PulseTrainTrialResult r = RunPulseTrainTrial(2500.0, 0.0, 2000);
  TEST_ASSERT_TRUE(r.maxErrFullRun <= 30);
  TEST_ASSERT_TRUE(r.maxErrSteadyState <= 20);
  TEST_ASSERT_TRUE(r.maxJumpSteadyState <= 15);
}

/* ---- jittered pulse train -- same two rates, short-term pulse-timing
 * noise injected (jitterFraction=0.3: each tick's edge count can arrive
 * up to 30% of a nominal tick's worth early/late). Looser bounds than the
 * clean case -- the point isn't "jitter changes nothing," it's that the
 * loop stays stable and bounded under it rather than ringing or
 * diverging. Observed at this jitter level: max following error <=12,
 * max steady-state jump <=8, same ballpark as clean -- still generous
 * margin below. ----------------------------------------------------- */

static void test_jittered_pulse_train_slow_rate_remains_bounded(void) {
  PulseTrainTrialResult r = RunPulseTrainTrial(200.0, 0.3, 2000);
  TEST_ASSERT_TRUE(r.maxErrFullRun <= 40);
  TEST_ASSERT_TRUE(r.maxErrSteadyState <= 30);
  TEST_ASSERT_TRUE(r.maxJumpSteadyState <= 20);
}

static void test_jittered_pulse_train_fast_rate_remains_bounded(void) {
  PulseTrainTrialResult r = RunPulseTrainTrial(2500.0, 0.3, 2000);
  TEST_ASSERT_TRUE(r.maxErrFullRun <= 40);
  TEST_ASSERT_TRUE(r.maxErrSteadyState <= 30);
  TEST_ASSERT_TRUE(r.maxJumpSteadyState <= 20);
}

/* Regression guard for the actual failure mode jitter could plausibly
 * cause: a slow divergence that only shows up well into a run, not in an
 * initial settle. Splits one longer run into first/second half and checks
 * the second half isn't tracking worse than the first -- same tune as
 * above, at the harder (fast-rate, jittered) scenario. */
static void test_jittered_pulse_train_does_not_degrade_over_a_longer_run(void) {
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);
  int32_t posTargetAccum = 0, pos = 0, vel = 0;
  int ticks = 6000;
  int32_t firstHalfMaxErr = 0, secondHalfMaxErr = 0;

  for (int i = 0; i < ticks; i++) {
    int32_t delta = PulseTrainTickDelta(&pt, 2500.0, kTickHz, 0.3);
    posTargetAccum += delta;
    int32_t torque = ControlTick(posTargetAccum, pos, vel, kDt);
    int32_t err = Abs32(posTargetAccum - pos);
    if (i < ticks / 2) {
      if (err > firstHalfMaxErr) firstHalfMaxErr = err;
    } else {
      if (err > secondHalfMaxErr) secondHalfMaxErr = err;
    }
    vel += torque / 16;
    pos += vel;
  }
  TEST_ASSERT_TRUE(secondHalfMaxErr <= firstHalfMaxErr + 10);   /* +10: noise margin, not growth */
}

/* ---- ramped pulse train -- does an ACCELERATING step/dir source (an
 * indexer ramping its own step rate up) get tracked without excess lag?
 * Mode A has no feedforward of its own to help with this: main.c's real
 * Mode A branch (main.c:466-472) always leaves Control.c's
 * referenceVelocityQ8 at 0 -- only Mode B/C ever set it -- so this is
 * exercising PURE reactive P->PI tracking of a ramping target, same as
 * production Mode A actually runs.
 *
 * Caveat that matters more here than anywhere else in this suite: the toy
 * plant (velocity integrates torque/16, same convention as
 * test_control.c's RunPlant()) has no real inertia or bandwidth limit, so
 * it may well be stiffer than a real motor and understate how much lag a
 * missing feedforward term would cause on the bench. Treat "stays
 * bounded here" as a regression guard for Control.c's own math, not as
 * evidence that real hardware would track an accelerating pulse train
 * equally well. ------------------------------------------------------ */

typedef struct {
  int32_t maxErrAccel;    /* worst following error while the rate is still ramping */
  int32_t maxErrCruise;   /* worst following error once the rate target is reached */
} RampedPulseTrainResult;

static RampedPulseTrainResult RunRampedPulseTrainTrial(double targetRateEdgesPerSec,
                                                        double rampSeconds, double jitterFraction,
                                                        int accelTicks, int cruiseTicks) {
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);
  double accelRatePerSec2 = targetRateEdgesPerSec / rampSeconds;
  int32_t posTargetAccum = 0, pos = 0, vel = 0;
  RampedPulseTrainResult result = {0, 0};

  int ticks = accelTicks + cruiseTicks;
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / kTickHz) : targetRateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, kTickHz, jitterFraction);
    posTargetAccum += delta;
    int32_t torque = ControlTick(posTargetAccum, pos, vel, kDt);
    int32_t err = Abs32(posTargetAccum - pos);
    if (i < accelTicks) {
      if (err > result.maxErrAccel) result.maxErrAccel = err;
    } else {
      if (err > result.maxErrCruise) result.maxErrCruise = err;
    }
    vel += torque / 16;
    pos += vel;
  }
  return result;
}

/* Moderate ramp: 0 -> 2500 edges/sec over 1s, then 1s cruise. Calibrated
 * (seed 12345): maxErrAccel/maxErrCruise observed 10/10 clean, 11/12
 * jittered -- bounds below give ~2.5x margin. */
static void test_ramped_pulse_train_moderate_accel_clean_tracks_without_excess_lag(void) {
  RampedPulseTrainResult r = RunRampedPulseTrainTrial(2500.0, 1.0, 0.0, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 25);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 25);
}

static void test_ramped_pulse_train_moderate_accel_jittered_remains_bounded(void) {
  RampedPulseTrainResult r = RunRampedPulseTrainTrial(2500.0, 1.0, 0.3, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 30);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 30);
}

/* Fast ramp: 0 -> 20000 edges/sec in 0.1s (an aggressive indexer
 * accel), then 1.9s cruise. Calibrated: maxErrAccel/maxErrCruise observed
 * 20/20 clean, 35/53 jittered. Error during the ramp itself is NOT larger
 * than at cruise in either case here -- see the toy-plant caveat above
 * before reading that as "feedforward wouldn't help on real hardware." */
static void test_ramped_pulse_train_fast_accel_clean_tracks_without_excess_lag(void) {
  RampedPulseTrainResult r = RunRampedPulseTrainTrial(20000.0, 0.1, 0.0, 100, 1900);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 45);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 45);
}

static void test_ramped_pulse_train_fast_accel_jittered_remains_bounded(void) {
  RampedPulseTrainResult r = RunRampedPulseTrainTrial(20000.0, 0.1, 0.3, 100, 1900);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 70);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 90);
}

/* ---- PROTOTYPE: velocity feedforward for Mode A -- NOT current production
 * behavior. The ramped-pulse-train tests above exercise what main.c
 * ACTUALLY does today (zero feedforward, pure reactive P->PI). This
 * section explores whether feeding Control.c's existing
 * ControlTickWithReferenceQ8() feedforward path -- already used by Mode
 * B/C, just never reached from Mode A -- would help Mode A track an
 * accelerating pulse train.
 *
 * Feeding this tick's raw step delta straight in as the Q8 reference
 * velocity cuts acceleration lag a lot, but injects the pulse train's own
 * quantization/jitter directly into the torque command every tick,
 * roughly doubling steady-state chatter (measured, moderate jittered
 * ramp: avgJumpCruise 1.86 -> 3.12). One-pole smoothing the delta first
 * -- same alpha=1/2^shift convention Control.c's own velocityFilterShift
 * already uses, just applied to the COMMANDED side instead of the
 * feedback side -- recovers most or all of that chatter cost while
 * keeping most of the lag improvement. Swept shift=1..4 against all four
 * ramped scenarios above; shift=3 is the best overall balance and is
 * what these tests pin down.
 *
 * These tests guard THIS PROTOTYPE'S measured behavior as a baseline for
 * a possible future main.c change -- they are not a regression guard on
 * anything currently shipped, and main.c does not call
 * ControlTickWithReferenceQ8() from Mode A today. ------------------- */

typedef struct {
  int32_t maxErrAccel;
  int32_t maxErrCruise;
  int32_t maxJumpCruise;
} FeedforwardTrialResult;

/* smoothShift < 0 means "raw, unsmoothed feedforward" (the naive first
 * attempt); smoothShift >= 0 one-pole-smooths the delta at that shift
 * before using it as the Q8 reference velocity. */
static FeedforwardTrialResult RunRampedPulseTrainTrialWithFeedforward(
    double targetRateEdgesPerSec, double rampSeconds, double jitterFraction,
    int accelTicks, int cruiseTicks, int32_t smoothShift) {
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);
  double accelRatePerSec2 = targetRateEdgesPerSec / rampSeconds;
  int32_t posTargetAccum = 0, pos = 0, vel = 0, prevTorque = 0;
  int64_t filteredDeltaQ8 = 0;
  FeedforwardTrialResult result = {0, 0, 0};

  int ticks = accelTicks + cruiseTicks;
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / kTickHz) : targetRateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, kTickHz, jitterFraction);
    posTargetAccum += delta;

    int64_t referenceVelocityQ8 = (int64_t)delta * 256;
    if (smoothShift >= 0) {
      filteredDeltaQ8 += (referenceVelocityQ8 - filteredDeltaQ8) >> smoothShift;
      referenceVelocityQ8 = filteredDeltaQ8;
    }
    int32_t torque = ControlTickWithReferenceQ8(posTargetAccum, pos, vel, referenceVelocityQ8, kDt);

    int32_t err = Abs32(posTargetAccum - pos);
    int32_t jump = Abs32(torque - prevTorque);
    prevTorque = torque;
    if (i < accelTicks) {
      if (err > result.maxErrAccel) result.maxErrAccel = err;
    } else {
      if (err > result.maxErrCruise) result.maxErrCruise = err;
      if (jump > result.maxJumpCruise) result.maxJumpCruise = jump;
    }
    vel += torque / 16;
    pos += vel;
  }
  return result;
}

/* The actual claim: smoothed feedforward (shift=3) beats production's
 * reactive-only tracking on lag, on both ramp scenarios above, without a
 * chatter blowup -- a real comparison against the already-measured
 * baseline, not just an absolute number. */
static void test_prototype_smoothed_feedforward_beats_reactive_only_on_moderate_ramp(void) {
  RampedPulseTrainResult reactive = RunRampedPulseTrainTrial(2500.0, 1.0, 0.3, 1000, 1000);
  FeedforwardTrialResult smoothed =
      RunRampedPulseTrainTrialWithFeedforward(2500.0, 1.0, 0.3, 1000, 1000, 3);
  TEST_ASSERT_TRUE(smoothed.maxErrAccel < reactive.maxErrAccel);    /* less lag ramping up */
  TEST_ASSERT_TRUE(smoothed.maxErrCruise <= reactive.maxErrCruise); /* at least as good settled */
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise <= 20);                   /* no chatter blowup */
}

static void test_prototype_smoothed_feedforward_beats_reactive_only_on_fast_ramp(void) {
  RampedPulseTrainResult reactive = RunRampedPulseTrainTrial(20000.0, 0.1, 0.3, 100, 1900);
  FeedforwardTrialResult smoothed =
      RunRampedPulseTrainTrialWithFeedforward(20000.0, 0.1, 0.3, 100, 1900, 3);
  TEST_ASSERT_TRUE(smoothed.maxErrAccel < reactive.maxErrAccel);
  TEST_ASSERT_TRUE(smoothed.maxErrCruise < reactive.maxErrCruise);
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise <= 55);   /* better than raw-ff's measured 57, see
                                                         the comment above -- this hardest
                                                         scenario doesn't fully recover to
                                                         reactive-only's chatter floor, unlike
                                                         the moderate-ramp case above */
}

/* Confirms smoothing is doing real work, not just a no-op: smoothed
 * (shift=3) must chatter meaningfully less than raw (unsmoothed)
 * feedforward on the same scenario. */
static void test_prototype_smoothing_reduces_chatter_versus_raw_feedforward(void) {
  FeedforwardTrialResult raw =
      RunRampedPulseTrainTrialWithFeedforward(2500.0, 1.0, 0.3, 1000, 1000, -1);
  FeedforwardTrialResult smoothed =
      RunRampedPulseTrainTrialWithFeedforward(2500.0, 1.0, 0.3, 1000, 1000, 3);
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise < raw.maxJumpCruise);
}

/* ---- PROTOTYPE, realistic plant: does the chatter penalty above survive
 * once the plant isn't instant/zero-lag? The feedforward section just
 * above uses the same zero-lag toy plant as the rest of this suite
 * (velocity += torque/16 every tick, no amplifier bandwidth limit, no
 * friction) -- fine for exercising Control.c's own math, but it means
 * EVERY bit of torque ripple shows up immediately and fully as velocity
 * ripple. A real motor+amplifier doesn't work that way: there's
 * electrical bandwidth lag and mechanical inertia between a commanded
 * torque and actual motion, both of which are themselves low-pass
 * filters.
 *
 * Using ToyPlant.h's model (inertia=64, one-pole amp lag alpha=0.6,
 * viscous friction=0.05 -- a deliberately "heavier and slower" plant than
 * the zero-lag default, NOT a claim about any specific real hardware) with
 * the SAME gains used above (posGainQ8=256, velGainKpQ8=512) goes
 * unstable outright -- following error in the thousands, torque pinned
 * against the clamp almost every tick, independent of feedforward mode.
 * That's not a finding about feedforward, it's a reminder that a gain set
 * is only ever valid for the plant it was matched to.
 *
 * Gains below (posGainQ8=8, velGainKpQ8=512, velGainKiQ8=1) were hand-
 * tuned against this specific plant via a step-response grid search
 * (target=5000, looking for no clamp-pegging and settling within ~1s) --
 * not an autotune, not a claim of optimality, just "stable and
 * reasonable" so the feedforward comparison below is fair rather than
 * measuring instability. With THIS plant and THESE gains, the story
 * changes a lot from the zero-lag section above: smoothed feedforward
 * keeps nearly all of raw feedforward's big lag improvement while
 * costing almost no extra chatter (maxJumpCruise ties or nearly ties the
 * no-feedforward baseline, instead of roughly doubling it). ---------- */

static const ToyPlantParams kRealisticPlant = {.inertia = 64.0, .ampAlpha = 0.6, .friction = 0.05};
static const ControlGains kRealisticPlantGains = {
  .posGainQ8 = 8,
  .velGainKpQ8 = 512,
  .velGainKiQ8 = 1,
  .velocityClamp = 200000,
  .torqueClampMin = -1000,
  .torqueClampMax = 1000,
};

static FeedforwardTrialResult RunRampedPulseTrainTrialWithRealisticPlant(
    double targetRateEdgesPerSec, double rampSeconds, double jitterFraction,
    int accelTicks, int cruiseTicks, int32_t smoothShift) {
  ControlGains g = kRealisticPlantGains;
  ControlInit(&g);
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);
  ToyPlantState plant;
  ToyPlantInit(&plant);
  double accelRatePerSec2 = targetRateEdgesPerSec / rampSeconds;
  int32_t posTargetAccum = 0, prevTorque = 0;
  int64_t filteredDeltaQ8 = 0;
  FeedforwardTrialResult result = {0, 0, 0};

  int ticks = accelTicks + cruiseTicks;
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / kTickHz) : targetRateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, kTickHz, jitterFraction);
    posTargetAccum += delta;
    int32_t posActual = ToyPlantPositionCounts(&plant);
    int32_t velActual = ToyPlantVelocityCounts(&plant);

    int32_t torque;
    if (smoothShift < -1) {   /* sentinel: no feedforward at all (current production) */
      torque = ControlTick(posTargetAccum, posActual, velActual, kDt);
    } else {
      int64_t referenceVelocityQ8 = (int64_t)delta * 256;
      if (smoothShift >= 0) {
        filteredDeltaQ8 += (referenceVelocityQ8 - filteredDeltaQ8) >> smoothShift;
        referenceVelocityQ8 = filteredDeltaQ8;
      }
      torque = ControlTickWithReferenceQ8(posTargetAccum, posActual, velActual, referenceVelocityQ8, kDt);
    }

    int32_t err = Abs32(posTargetAccum - posActual);
    int32_t jump = Abs32(torque - prevTorque);
    prevTorque = torque;
    if (i < accelTicks) {
      if (err > result.maxErrAccel) result.maxErrAccel = err;
    } else {
      if (err > result.maxErrCruise) result.maxErrCruise = err;
      if (jump > result.maxJumpCruise) result.maxJumpCruise = jump;
    }
    ToyPlantStep(&plant, &kRealisticPlant, torque);
  }
  return result;
}

static void test_prototype_realistic_plant_smoothed_feedforward_beats_reactive_only_on_moderate_ramp(void) {
  FeedforwardTrialResult noFF = RunRampedPulseTrainTrialWithRealisticPlant(2500.0, 1.0, 0.3, 1000, 1000, -2);
  FeedforwardTrialResult smoothed = RunRampedPulseTrainTrialWithRealisticPlant(2500.0, 1.0, 0.3, 1000, 1000, 3);
  TEST_ASSERT_TRUE(smoothed.maxErrAccel < noFF.maxErrAccel);
  TEST_ASSERT_TRUE(smoothed.maxErrCruise < noFF.maxErrCruise);
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise <= noFF.maxJumpCruise + 5);   /* essentially no chatter
                                                                             penalty, unlike the
                                                                             zero-lag plant above */
}

static void test_prototype_realistic_plant_smoothed_feedforward_beats_reactive_only_on_fast_ramp(void) {
  FeedforwardTrialResult noFF = RunRampedPulseTrainTrialWithRealisticPlant(20000.0, 0.1, 0.3, 100, 1900, -2);
  FeedforwardTrialResult smoothed = RunRampedPulseTrainTrialWithRealisticPlant(20000.0, 0.1, 0.3, 100, 1900, 3);
  TEST_ASSERT_TRUE(smoothed.maxErrAccel < noFF.maxErrAccel);
  TEST_ASSERT_TRUE(smoothed.maxErrCruise < noFF.maxErrCruise);
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise <= noFF.maxJumpCruise + 15);
}

static void test_prototype_realistic_plant_smoothed_feedforward_beats_raw_feedforward_chatter(void) {
  FeedforwardTrialResult raw = RunRampedPulseTrainTrialWithRealisticPlant(2500.0, 1.0, 0.3, 1000, 1000, -1);
  FeedforwardTrialResult smoothed = RunRampedPulseTrainTrialWithRealisticPlant(2500.0, 1.0, 0.3, 1000, 1000, 3);
  TEST_ASSERT_TRUE(smoothed.maxJumpCruise < raw.maxJumpCruise);

  FeedforwardTrialResult rawFast = RunRampedPulseTrainTrialWithRealisticPlant(20000.0, 0.1, 0.3, 100, 1900, -1);
  FeedforwardTrialResult smoothedFast = RunRampedPulseTrainTrialWithRealisticPlant(20000.0, 0.1, 0.3, 100, 1900, 3);
  TEST_ASSERT_TRUE(smoothedFast.maxJumpCruise < rawFast.maxJumpCruise);
}

/* ---- PROTOTYPE, named machine presets: MachinePresets.h bundles
 * ToyPlant.h's two-mass compliant-coupling model (plus the stiction/
 * cogging primitives added alongside it) into two recognizable
 * mechanisms -- a NEMA 23 lead-screw mill X-axis (Lovejoy coupling,
 * cogging, load-side stiction) and a treadmill motor spindle (belt
 * coupling, very large inertia, no cogging). Representative ratios, not
 * measurements of any specific real machine -- see MachinePresets.h's
 * own header comment for the reasoning behind each parameter.
 *
 * These are sanity/stability checks, not the full feedforward matrix
 * from the sections above: each preset's own hand-tuned gains (found via
 * the same step-response grid-search process as every other gain set in
 * this suite) should track a representative ramped pulse train, clean
 * and jittered, without sustained torque-clamp pegging and with a
 * following error that stays bounded. Calibrated (seed 12345, 2500
 * edges/sec, 1s ramp + 1s cruise): nema23 maxErr ~130-215, zero pegged
 * ticks; treadmill maxErr ~670-750, zero pegged ticks -- bounds below
 * give ~2x margin on each. ------------------------------------------- */

typedef struct {
  int32_t maxErrAccel;
  int32_t maxErrCruise;
  int32_t pegTicks;
} MachinePresetTrialResult;

static MachinePresetTrialResult RunMachinePresetTrial(
    const ToyPlantParams *motor, const ToyPlantParams *load, const CouplingParams *coupling,
    const ControlGains *gains, double targetRateEdgesPerSec, double rampSeconds,
    double jitterFraction, int accelTicks, int cruiseTicks) {
  ControlGains g = *gains;
  ControlInit(&g);
  PulseTrainState pt;
  PulseTrainInit(&pt, kSeed);
  CoupledPlantState plant;
  CoupledPlantInit(&plant);
  double accelRatePerSec2 = targetRateEdgesPerSec / rampSeconds;
  int32_t posTargetAccum = 0;
  MachinePresetTrialResult result = {0, 0, 0};

  int ticks = accelTicks + cruiseTicks;
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / kTickHz) : targetRateEdgesPerSec;
    int32_t delta = PulseTrainTickDelta(&pt, rate, kTickHz, jitterFraction);
    posTargetAccum += delta;
    int32_t posActual = CoupledPlantMotorPositionCounts(&plant);
    int32_t velActual = CoupledPlantMotorVelocityCounts(&plant);
    int32_t torque = ControlTick(posTargetAccum, posActual, velActual, kDt);
    int32_t err = Abs32(posTargetAccum - posActual);
    if (i < accelTicks) {
      if (err > result.maxErrAccel) result.maxErrAccel = err;
    } else {
      if (err > result.maxErrCruise) result.maxErrCruise = err;
    }
    if (Abs32(torque) >= g.torqueClampMax - 1) result.pegTicks++;
    CoupledPlantStep(&plant, motor, load, coupling, torque);
  }
  return result;
}

static void test_prototype_nema23_leadscrew_preset_clean_is_stable(void) {
  MachinePresetTrialResult r = RunMachinePresetTrial(
      &kNema23LeadscrewMotor, &kNema23LeadscrewLoad, &kNema23LeadscrewCoupling,
      &kNema23LeadscrewGains, 2500.0, 1.0, 0.0, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 450);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 300);
  TEST_ASSERT_TRUE(r.pegTicks <= 20);
}

static void test_prototype_nema23_leadscrew_preset_jittered_is_stable(void) {
  MachinePresetTrialResult r = RunMachinePresetTrial(
      &kNema23LeadscrewMotor, &kNema23LeadscrewLoad, &kNema23LeadscrewCoupling,
      &kNema23LeadscrewGains, 2500.0, 1.0, 0.3, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 450);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 300);
  TEST_ASSERT_TRUE(r.pegTicks <= 20);
}

/* Confirms the friction bump actually does something -- without it,
 * crossing the same stretch of travel produces visibly less following
 * error than with it (131 -> 275 at cruise, calibrated; see
 * MachinePresets.h). Not just "the preset is still stable with a bump
 * somewhere in it" (the test above already covers that) -- this pins
 * down that the bump is a real, measurable local disturbance, not a
 * no-op. */
static void test_prototype_nema23_friction_bump_increases_local_error(void) {
  ToyPlantParams loadWithoutBump = kNema23LeadscrewLoad;
  loadWithoutBump.frictionBumpHalfWidth = 0.0;   /* disables the bump, same base stiction/coulomb */

  MachinePresetTrialResult withoutBump = RunMachinePresetTrial(
      &kNema23LeadscrewMotor, &loadWithoutBump, &kNema23LeadscrewCoupling,
      &kNema23LeadscrewGains, 2500.0, 1.0, 0.0, 1000, 1000);
  MachinePresetTrialResult withBump = RunMachinePresetTrial(
      &kNema23LeadscrewMotor, &kNema23LeadscrewLoad, &kNema23LeadscrewCoupling,
      &kNema23LeadscrewGains, 2500.0, 1.0, 0.0, 1000, 1000);
  TEST_ASSERT_TRUE(withBump.maxErrCruise > withoutBump.maxErrCruise);
}

static void test_prototype_treadmill_spindle_preset_clean_is_stable(void) {
  MachinePresetTrialResult r = RunMachinePresetTrial(
      &kTreadmillSpindleMotor, &kTreadmillSpindleLoad, &kTreadmillSpindleCoupling,
      &kTreadmillSpindleGains, 2500.0, 1.0, 0.0, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 1500);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 1500);
  TEST_ASSERT_TRUE(r.pegTicks <= 20);
}

static void test_prototype_treadmill_spindle_preset_jittered_is_stable(void) {
  MachinePresetTrialResult r = RunMachinePresetTrial(
      &kTreadmillSpindleMotor, &kTreadmillSpindleLoad, &kTreadmillSpindleCoupling,
      &kTreadmillSpindleGains, 2500.0, 1.0, 0.3, 1000, 1000);
  TEST_ASSERT_TRUE(r.maxErrAccel <= 1500);
  TEST_ASSERT_TRUE(r.maxErrCruise <= 1500);
  TEST_ASSERT_TRUE(r.pegTicks <= 20);
}

/* ---- velocityFilterShift -- does smoothing the measured velocity cut
 * down the reverse-torque chatter from pulse-train quantization? Measured
 * (seed 12345, 2500 edges/sec, last-quarter-of-run window): shift=0
 * jittered maxErr=11/maxJump=8; shift=1 jittered maxErr=12/maxJump=8;
 * shift=1 clean maxErr=10/maxJump=3 (a real improvement over shift=0
 * clean's maxJump=4, but NOT a clear win under jitter -- marginal either
 * way, not the fix for this symptom).
 *
 * shift>=2 is a genuine trap with THIS gain set (velGainKpQ8=512, no
 * feedforward reaching Mode A): measured shift=3 jittered maxErr=577,
 * maxJump=477 -- the added filter lag destabilizes the loop outright,
 * roughly 50x worse than shift=0/1. Not asserted here (a test that must
 * stay broken is the wrong kind of test) -- noted so nobody reaches for a
 * bigger shift expecting a bigger improvement with these gains. */
static PulseTrainTrialResult RunPulseTrainTrialWithFilterShift(double rateEdgesPerSec,
                                                                double jitterFraction, int ticks,
                                                                int32_t filterShift) {
  ControlGains g = {
    .posGainQ8 = 256,
    .velGainKpQ8 = 512,
    .velGainKiQ8 = 26,
    .velocityClamp = 100000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
    .velocityFilterShift = filterShift,
  };
  ControlInit(&g);
  return RunPulseTrainTrial(rateEdgesPerSec, jitterFraction, ticks);
}

static void test_filter_shift_one_stays_bounded_under_jitter(void) {
  PulseTrainTrialResult r = RunPulseTrainTrialWithFilterShift(2500.0, 0.3, 2000, 1);
  TEST_ASSERT_TRUE(r.maxErrSteadyState <= 30);
  TEST_ASSERT_TRUE(r.maxJumpSteadyState <= 25);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_clean_pulse_train_slow_rate_tracks_without_jumps);
  RUN_TEST(test_clean_pulse_train_fast_rate_tracks_without_jumps);
  RUN_TEST(test_jittered_pulse_train_slow_rate_remains_bounded);
  RUN_TEST(test_jittered_pulse_train_fast_rate_remains_bounded);
  RUN_TEST(test_jittered_pulse_train_does_not_degrade_over_a_longer_run);
  RUN_TEST(test_ramped_pulse_train_moderate_accel_clean_tracks_without_excess_lag);
  RUN_TEST(test_ramped_pulse_train_moderate_accel_jittered_remains_bounded);
  RUN_TEST(test_ramped_pulse_train_fast_accel_clean_tracks_without_excess_lag);
  RUN_TEST(test_ramped_pulse_train_fast_accel_jittered_remains_bounded);
  RUN_TEST(test_prototype_smoothed_feedforward_beats_reactive_only_on_moderate_ramp);
  RUN_TEST(test_prototype_smoothed_feedforward_beats_reactive_only_on_fast_ramp);
  RUN_TEST(test_prototype_smoothing_reduces_chatter_versus_raw_feedforward);
  RUN_TEST(test_prototype_realistic_plant_smoothed_feedforward_beats_reactive_only_on_moderate_ramp);
  RUN_TEST(test_prototype_realistic_plant_smoothed_feedforward_beats_reactive_only_on_fast_ramp);
  RUN_TEST(test_prototype_realistic_plant_smoothed_feedforward_beats_raw_feedforward_chatter);
  RUN_TEST(test_prototype_nema23_leadscrew_preset_clean_is_stable);
  RUN_TEST(test_prototype_nema23_leadscrew_preset_jittered_is_stable);
  RUN_TEST(test_prototype_nema23_friction_bump_increases_local_error);
  RUN_TEST(test_prototype_treadmill_spindle_preset_clean_is_stable);
  RUN_TEST(test_prototype_treadmill_spindle_preset_jittered_is_stable);
  RUN_TEST(test_filter_shift_one_stays_bounded_under_jitter);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
