/*
 * Native unit tests for the online RLS velocity-plant estimator (RLS.c) and
 * its gain-interpolation primitive (GainGlide.c). Same
 * full-env-link-requires-every-App*-stub situation as test_autotune.c (see
 * its own file comment) -- [env:native]'s build_src_filter links the whole
 * env, Protocol.c included, into every test/test_* executable.
 * Run: pio test -e native
 */
#include <unity.h>
#include <math.h>
#include "RLS.h"
#include "GainGlide.h"
#include "Autotune.h"
#include "Control.h"
#include "main.h"
#include "Encoder.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

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
int  AppConfigSave(void) { return -1; }
int  AppConfigLoad(void) { return -1; }
void AppConfigDefaults(void) {}
void AppPolarityLearn(void) {}
int  AppPolarityLearnBusy(void) { return 0; }
int     AppGetPolarityLearnFailed(void) { return 0; }
int32_t AppGetPolarityLearnMovedCounts(void) { return 0; }
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
int        AppGetSignalCycleStart(void) { return 0; }
static int32_t sPosition = 0;
int32_t EncoderGetPositionCounts(void) { return sPosition; }
int     EncoderZWasSeenSinceEnable(void) { return 0; }
void    OutputForceSafe(void) {}
void    UsartTransmit(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { (void)Buf; (void)Len; return USBD_OK; }
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel, uint8_t *pbuff, uint16_t length) { (void)pdev; (void)channel; (void)pbuff; (void)length; return USBD_OK; }
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) { (void)pdev; (void)channel; return USBD_OK; }
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
void     AppAutotuneApply(void) {}
uint32_t AppAutotuneGetRelayProgressSeq(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressTicksElapsed(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressMaxTicks(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressSwitchCount(void) { return 0; }
uint32_t AppAutotuneGetRelayProgressSwitchesNeeded(void) { return 0; }
int32_t  AppAutotuneGetRelayProgressPeakVelocity(void) { return 0; }
void AppSetCommandVelocityClamp(int32_t c) { (void)c; }
void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }
void AppSetAccelerationFeedforward(int32_t c) { (void)c; }
void AppSetVelocityJerk(int32_t c) { (void)c; }
void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }

void setUp(void) { RlsInit(); }
void tearDown(void) {}

/* ---- synthetic plant: v[n+1] = trueA*v[n] + trueB*u[n], driven through
 * RlsTick() the same way main.c's AppControlTick() would -- RlsTick(u[n],
 * v[n]) called in sequence, v[n] advanced by hand between calls. Double
 * precision here is the reference; RLS.c's own fixed point is what's under
 * test, same "reference math at higher precision than the firmware uses"
 * approach as test_autotune.c's hand-computed Ku/Kp/Ki. */
#define SIM_TRUE_A 0.90
#define SIM_TRUE_B 0.10

static int32_t sSimV;

static void SimReset(void) { sSimV = 0; }

static void SimStepAndTick(int32_t u) {
  RlsTick(u, sSimV);
  double vNext = SIM_TRUE_A * (double)sSimV + SIM_TRUE_B * (double)u;
  sSimV = (int32_t)(vNext >= 0.0 ? vNext + 0.5 : vNext - 0.5);
}

/* Deterministic pseudo-random excitation -- rich enough to identify both
 * plant parameters, reproducible across platforms/runs (no libc rand()). */
static uint32_t sLcg = 12345U;
static int32_t RandRange(int32_t lo, int32_t hi) {
  sLcg = sLcg * 1664525U + 1013904223U;
  uint32_t span = (uint32_t)(hi - lo + 1);
  return lo + (int32_t)((sLcg >> 8) % span);
}

/* One "move": numTicks of randomized excitation, holding each random u for
 * holdTicks ticks (coarser than a fresh random value every single tick,
 * closer to how a real move's torque command actually behaves). */
static void RunMove(uint32_t numTicks, uint32_t holdTicks) {
  int32_t u = 0;
  for (uint32_t t = 0; t < numTicks; t++) {
    if (t % holdTicks == 0U) u = RandRange(-6000, 6000);
    SimStepAndTick(u);
  }
}

/* A pause between moves: torque held at exactly 0 long enough for the
 * plant's own decay (trueA<1) to coast velocity down into the dead zone --
 * exercises the covariance wind-up guard under the actual "pauses between
 * moves" scenario RLS.h's file comment calls out. */
static void RunPause(uint32_t numTicks) {
  for (uint32_t t = 0; t < numTicks; t++) SimStepAndTick(0);
}

static void test_idle_tick_is_a_noop(void) {
  TEST_ASSERT_FALSE(RlsIsActive());
  RlsTick(1234, 5678);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NONE, RlsGetResult());
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaAQ8());
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaBQ8());
}

static void test_start_goes_active_with_zero_prior(void) {
  RlsStart(256, 1000U);
  TEST_ASSERT_TRUE(RlsIsActive());
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaAQ8());
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaBQ8());
  TEST_ASSERT_EQUAL_UINT32(0U, RlsGetTicksObserved());
}

static void test_convergence_within_tolerance(void) {
  SimReset();
  RlsStart(256, 1000U);   /* lambda=1.0 (no forgetting) -- most stable choice for a clean,
                              noiseless convergence check */
  RunMove(1500U, 10U);

  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, RlsGetResult());
  /* trueA=0.90 -> Q8.8 230, trueB=0.10 -> Q8.8 26. */
  TEST_ASSERT_INT32_WITHIN(5, 230, RlsGetThetaAQ8());
  TEST_ASSERT_INT32_WITHIN(5, 26, RlsGetThetaBQ8());
  TEST_ASSERT_TRUE(RlsGetKpResultQ8() > 0);
  TEST_ASSERT_TRUE(RlsGetKiResultQ8() > 0);
}

static void test_covariance_does_not_wind_up_during_pause(void) {
  SimReset();
  RlsStart(255, 1000U);   /* lambda < 1.0 -- forgetting enabled, so the 1/lambda growth term
                              this test is actually checking exists in the first place */
  RunMove(600U, 10U);                 /* real excitation -- P should shrink from its P0 prior */
  RunPause(200U);                     /* settle fully into the dead zone first -- with trueA=0.90
                                           this coasts below the velocity threshold well inside
                                           200 ticks, so this stretch's OWN updates (legitimate,
                                           not yet in the dead zone) don't contaminate the
                                           comparison below */
  int64_t pBeforePause = RlsGetPTrace();

  RunPause(800U);                     /* once truly quiet, further pause ticks must be no-ops */
  int64_t pAfterPause = RlsGetPTrace();

  TEST_ASSERT_EQUAL_INT64(pBeforePause, pAfterPause);   /* dead zone must have skipped every
                                                             update once settled, not just slowed
                                                             growth */

  /* And a fresh move afterward still converges normally -- the guard didn't
   * leave the estimator in some stuck state. */
  RunMove(200U, 10U);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, RlsGetResult());
  TEST_ASSERT_INT32_WITHIN(8, 230, RlsGetThetaAQ8());
  TEST_ASSERT_INT32_WITHIN(8, 26, RlsGetThetaBQ8());
}

static void test_bad_measurement_rejected_not_silently_applied(void) {
  /* Drive RlsTick() directly (bypassing the plant simulation) with a
   * sequence designed to push thetaB toward ~0 -- u varies while v stays
   * pinned at 0, so the regression has no information relating u to any
   * velocity response at all, and b should never climb out of its
   * near-zero divide-by-zero guard. */
  RlsStart(256, 1000U);
  for (int i = 0; i < 200; i++) {
    RlsTick(RandRange(-6000, 6000), 0);
  }
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_BAD_MEASUREMENT, RlsGetResult());
}

static void test_stop_freezes_state(void) {
  SimReset();
  RlsStart(256, 1000U);
  RunMove(600U, 10U);
  int32_t kpBefore = RlsGetKpResultQ8();
  int32_t kiBefore = RlsGetKiResultQ8();
  RlsStop();
  TEST_ASSERT_FALSE(RlsIsActive());
  RlsTick(1234, 5678);   /* must be a no-op now -- sRunning gates RlsTick() */
  TEST_ASSERT_EQUAL_INT32(kpBefore, RlsGetKpResultQ8());
  TEST_ASSERT_EQUAL_INT32(kiBefore, RlsGetKiResultQ8());
}

static void test_restart_clears_previous_estimate(void) {
  SimReset();
  RlsStart(256, 1000U);
  RunMove(600U, 10U);
  TEST_ASSERT_TRUE(RlsGetThetaAQ8() != 0 || RlsGetThetaBQ8() != 0);

  RlsStart(256, 1000U);   /* fresh attempt */
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaAQ8());
  TEST_ASSERT_EQUAL_INT32(0, RlsGetThetaBQ8());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NONE, RlsGetResult());
}

static void test_tick_rate_independence(void) {
  /* Same physical plant, discretized at two different tick rates: at
   * tickHz=4000 the per-tick 'a' is closer to 1 and 'b' is smaller than at
   * tickHz=1000 for the SAME physical mechanism (see RLS.h/RLS.c's file
   * comments on why b ~ K*T^2) -- simulate each rate's own discrete
   * plant and confirm both runs still land on an OK result with a
   * physically sensible (positive, non-clamped) candidate. This is the
   * regression test for RLS.c not baking in a 1kHz assumption. */
  double periods[2] = { 1.0 / 1000.0, 1.0 / 4000.0 };
  uint32_t tickHz[2] = { 1000U, 4000U };
  double trueTau = 0.05;        /* 50ms friction time constant, same physical plant both runs */
  double trueKPhys = 1.0e5;     /* counts/sec^2 per raw torque unit, same physical plant both runs --
                                    chosen so the resulting b lands comfortably inside RLS.c's
                                    generous placeholder clamp at both tick rates (b~0.10 @ 1kHz,
                                    b~0.006 @ 4kHz, see the T^2 scaling this is testing) */

  for (int run = 0; run < 2; run++) {
    double T = periods[run];
    double a = exp(-T / trueTau);
    double b = trueKPhys * trueTau * (1.0 - a) * T;   /* exact discretization, see RLS.h comment */

    RlsInit();
    RlsStart(256, tickHz[run]);
    sSimV = 0;
    sLcg = 12345U;   /* identical excitation sequence both runs */
    int32_t u = 0;
    for (uint32_t t = 0; t < 3000U; t++) {
      if (t % 10U == 0U) u = RandRange(-6000, 6000);
      RlsTick(u, sSimV);
      double vNext = a * (double)sSimV + b * (double)u;
      sSimV = (int32_t)(vNext >= 0.0 ? vNext + 0.5 : vNext - 0.5);
    }

    TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, RlsGetResult());
    TEST_ASSERT_TRUE(RlsGetKpResultQ8() > 0);
    TEST_ASSERT_TRUE(RlsGetKiResultQ8() > 0);
  }
}

/* ---- GainGlide ---- */

static ControlGains MakeGains(int32_t pos, int32_t kp, int32_t ki) {
  ControlGains g = {0};
  g.posGainQ8 = pos;
  g.velGainKpQ8 = kp;
  g.velGainKiQ8 = ki;
  g.torqueClampMin = -32767;
  g.torqueClampMax = 32767;
  return g;
}

static void test_glide_reaches_target_exactly_at_total_ticks(void) {
  GainGlideState glide;
  ControlGains from = MakeGains(256, 256, 0);
  ControlGains to = MakeGains(256, 1000, 50);
  GainGlideStart(&glide, &from, &to, 10U);

  ControlGains out;
  int result = 0;
  for (int i = 0; i < 10; i++) result = GainGlideTick(&glide, &out);

  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_INT32(1000, out.velGainKpQ8);
  TEST_ASSERT_EQUAL_INT32(50, out.velGainKiQ8);
  TEST_ASSERT_FALSE(GainGlideActive(&glide));
}

static void test_glide_is_monotonic_between_endpoints(void) {
  GainGlideState glide;
  ControlGains from = MakeGains(256, 256, 0);
  ControlGains to = MakeGains(256, 1000, 100);
  GainGlideStart(&glide, &from, &to, 20U);

  ControlGains out;
  int32_t lastKp = from.velGainKpQ8;
  for (int i = 0; i < 20; i++) {
    GainGlideTick(&glide, &out);
    TEST_ASSERT_TRUE(out.velGainKpQ8 >= lastKp);
    lastKp = out.velGainKpQ8;
  }
}

static void test_glide_inactive_is_a_noop(void) {
  GainGlideState glide = {0};
  ControlGains out = MakeGains(1, 2, 3);
  int result = GainGlideTick(&glide, &out);
  TEST_ASSERT_FALSE(result);
  TEST_ASSERT_EQUAL_INT32(1, out.posGainQ8);   /* untouched */
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_idle_tick_is_a_noop);
  RUN_TEST(test_start_goes_active_with_zero_prior);
  RUN_TEST(test_convergence_within_tolerance);
  RUN_TEST(test_covariance_does_not_wind_up_during_pause);
  RUN_TEST(test_bad_measurement_rejected_not_silently_applied);
  RUN_TEST(test_stop_freezes_state);
  RUN_TEST(test_restart_clears_previous_estimate);
  RUN_TEST(test_tick_rate_independence);
  RUN_TEST(test_glide_reaches_target_exactly_at_total_ticks);
  RUN_TEST(test_glide_is_monotonic_between_endpoints);
  RUN_TEST(test_glide_inactive_is_a_noop);
  return UNITY_END();
}
