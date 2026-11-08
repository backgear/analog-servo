/*
 * Native unit tests for Mode B's accel/decel ramp generator (Ramp.c).
 * Pure fixed-point tick counting, no HAL dependency -- natively testable.
 * Run: pio test -e native
 */
#include <unity.h>
#include "Ramp.h"
#include "Control.h"
#include "main.h"
#include "Encoder.h"
#include "Output.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Fault.c/Protocol.c (and everything they call
 * into) get linked into every test/test_* executable, this one included.
 * See platformio.ini and test_signal.c for the same pattern. */
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
void     AppAutotuneApply(void) {}
void       AppSignalStart(SignalType t, SignalTarget tg, int32_t amp, uint32_t hp, uint32_t dur) {
  (void)t; (void)tg; (void)amp; (void)hp; (void)dur;
}
void       AppSignalStop(void) {}
int        AppGetSignalActive(void) { return 0; }
SignalType   AppGetSignalType(void) { return SIGNAL_NONE; }
SignalTarget AppGetSignalTarget(void) { return SIGNAL_TARGET_TORQUE; }
int        AppGetSignalCycleStart(void) { return 0; }
int AppResetPosition(void) { return 0; }
void AppGetControlSample(AppControlSample *out) { *out = (AppControlSample){0}; }
int32_t AppGetLastVelocityActual(void) { return 0; }
int32_t AppGetLastTorqueCommand(void) { return 0; }
int32_t AppGetLastCommandedPosition(void) { return 0; }
int32_t AppGetLastCommandedVelocity(void) { return 0; }
int32_t EncoderGetPositionCounts(void) { return 0; }
int     EncoderZWasSeenSinceEnable(void) { return 0; }
void    UsartTransmit(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { (void)Buf; (void)Len; return USBD_OK; }
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel, uint8_t *pbuff, uint16_t length) { (void)pdev; (void)channel; (void)pbuff; (void)length; return USBD_OK; }
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) { (void)pdev; (void)channel; return USBD_OK; }

/* SYSCLK_HZ is whatever the native env's build_flags define (see
 * platformio.ini) -- not asserted against a specific value here, these
 * tests work out the expected step size FROM it instead of hardcoding a
 * number that'd silently go stale if that define ever changes. */
#ifndef SYSCLK_HZ
#error "SYSCLK_HZ must be defined (see platformio.ini's native env)"
#endif

static RampState r;

void setUp(void) { RampInit(&r); }
void tearDown(void) {}

static void test_init_defaults(void) {
  TEST_ASSERT_EQUAL_INT32(0, r.currentVelocity);
  TEST_ASSERT_EQUAL_INT32(0, r.targetVelocity);
  TEST_ASSERT_EQUAL_INT32(1000, r.accelCountsPerTickPerSec);
}

static void test_tick_with_no_target_change_stays_zero(void) {
  TEST_ASSERT_EQUAL_INT32(0, RampTick(&r, SYSCLK_HZ));
}

static void test_tick_ramps_toward_positive_target_not_instantly(void) {
  RampSetTarget(&r, 100000);   /* far enough that one tick's accel can't reach it */
  int32_t v = RampTick(&r, SYSCLK_HZ);   /* exactly 1 second's worth of accel */
  TEST_ASSERT_EQUAL_INT32(1000, v);   /* accelCountsPerTickPerSec's default -- one tick's
                                          worth of ramp, not an instant jump to the target */
}

static void test_tick_never_overshoots_target(void) {
  RampSetTarget(&r, 500);
  int32_t v = RampTick(&r, SYSCLK_HZ);   /* 1000 counts/tick of accel available, target is 500 */
  TEST_ASSERT_EQUAL_INT32(500, v);   /* clamped, not 1000 */
}

static void test_tick_ramps_toward_negative_target(void) {
  RampSetTarget(&r, -500);
  int32_t v = RampTick(&r, SYSCLK_HZ);
  TEST_ASSERT_EQUAL_INT32(-500, v);
}

static void test_reaching_target_then_holds(void) {
  RampSetTarget(&r, 100);
  RampTick(&r, SYSCLK_HZ);   /* reaches 100 (clamped, see above) */
  TEST_ASSERT_EQUAL_INT32(100, RampTick(&r, SYSCLK_HZ));   /* already there -- early-return path */
}

static void test_small_dt_takes_multiple_ticks_to_arrive(void) {
  RampSetAcceleration(&r, 1000);
  RampSetTarget(&r, 10);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;   /* ~1ms, the real control-tick cadence */
  int32_t v = RampTick(&r, dtPerTick);
  TEST_ASSERT_TRUE(v > 0);
  TEST_ASSERT_TRUE(v < 10);   /* one tick's worth of accel is much less than the target --
                                  proves this isn't silently jumping straight there */
}

static void test_retargeting_mid_ramp_reverses_direction(void) {
  RampSetTarget(&r, 10000);
  RampTick(&r, SYSCLK_HZ / 1000U);   /* small positive step underway */
  int32_t midVelocity = r.currentVelocity;
  TEST_ASSERT_TRUE(midVelocity > 0);

  RampSetTarget(&r, -10000);   /* retarget before arriving -- e.g. a jog reversed */
  int32_t v = RampTick(&r, SYSCLK_HZ / 1000U);
  TEST_ASSERT_TRUE(v < midVelocity);   /* immediately decelerating/reversing, not still
                                           climbing toward the old target */
}

static void test_accel_scales_with_configured_rate(void) {
  RampSetAcceleration(&r, 2000);
  RampSetTarget(&r, 100000);
  TEST_ASSERT_EQUAL_INT32(2000, RampTick(&r, SYSCLK_HZ));
}

static void test_error_accumulator_prevents_rate_starvation(void) {
  /* A tiny accel at a tiny dt would round `step` to exactly 0 every tick
   * without the fixed-point error accumulator carrying the remainder
   * forward -- this is the same "don't silently starve the ramp at low
   * dt/low accel" property Ramp.h's header comment calls out. */
  RampSetAcceleration(&r, 1);   /* 1 count/tick per second -- tiny */
  RampSetTarget(&r, 100000);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;
  int32_t total = 0;
  for (int i = 0; i < 2000; i++) total = RampTick(&r, dtPerTick);   /* ~2 real seconds */
  TEST_ASSERT_TRUE(total > 0);   /* accumulated SOME motion over 2s at 1 count/tick/sec,
                                     not stuck at 0 the whole time from per-tick rounding */
}

/* ---- RampPeekAccelPerTick ---------------------------------------------- */

static void test_peek_matches_what_tick_would_apply(void) {
  RampSetAcceleration(&r, 1000);
  RampSetTarget(&r, 100000);   /* far enough that RampTick won't clamp to the target */
  int32_t peeked = RampPeekAccelPerTick(&r, SYSCLK_HZ);
  int32_t actualVelocity = RampTick(&r, SYSCLK_HZ);
  TEST_ASSERT_EQUAL_INT32(peeked, actualVelocity);   /* started at 0, so the step IS the
                                                          resulting velocity */
}

static void test_peek_does_not_mutate_state(void) {
  RampSetAcceleration(&r, 1);   /* tiny -- any peek-caused error-accumulator drift would be
                                    visible in whether this starves exactly like the real
                                    test_error_accumulator_prevents_rate_starvation case */
  RampSetTarget(&r, 100000);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;

  for (int i = 0; i < 50; i++) RampPeekAccelPerTick(&r, dtPerTick);   /* peek repeatedly,
                                                                          same dt each time */
  TEST_ASSERT_EQUAL_INT32(0, r.currentVelocity);   /* never advanced -- peek is read-only */
  TEST_ASSERT_EQUAL_INT32(0, r.error);              /* and never perturbed the real
                                                         accumulator a later REAL tick
                                                         depends on */

  int32_t total = 0;
  for (int i = 0; i < 2000; i++) total = RampTick(&r, dtPerTick);
  TEST_ASSERT_TRUE(total > 0);   /* real ticks still accumulate normally afterward --
                                     the 50 peeks above left no residue to desync this */
}

static void test_peek_is_nonnegative_for_a_negative_target(void) {
  RampSetTarget(&r, -100000);
  int32_t peeked = RampPeekAccelPerTick(&r, SYSCLK_HZ);
  TEST_ASSERT_TRUE(peeked >= 0);   /* a magnitude, same convention as RampTick()'s internal
                                       step -- direction is the caller's problem */
}

/* ---- RampShouldDecelerateNow -------------------------------------------- */

static void test_should_not_decelerate_from_standstill_far_from_target(void) {
  RampSetAcceleration(&r, 1000);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;
  TEST_ASSERT_FALSE(RampShouldDecelerateNow(&r, dtPerTick, 0, 100000));
}

static void test_should_decelerate_when_close_at_speed(void) {
  RampSetAcceleration(&r, 1000);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;
  /* velocity=500, 1 tick's accel=1 (accel=1000/sec @ 1ms) -- stopping
   * distance = v^2/(2*a) = 250000, far more than remaining=10. */
  TEST_ASSERT_TRUE(RampShouldDecelerateNow(&r, dtPerTick, 500, 10));
}

static void test_slow_accel_relative_to_tick_rate_does_not_misfire(void) {
  /* The exact bug this function exists to fix: accel=300 counts/tick/sec
   * at a real ~1ms tick is only 0.3 counts/tick -- an integer per-tick
   * step truncates that to 0, and a naive v^2 >= 2*0*d check degenerates
   * to "0 >= 0," always true the instant velocity is 0, regardless of
   * how far away the target is. Confirmed on real hardware: a
   * PositionMove.c built on that naive check never moved at all. This
   * function must say "don't decelerate yet" here, not "decelerate." */
  RampSetAcceleration(&r, 300);
  uint32_t dtPerTick = SYSCLK_HZ / 1000U;
  TEST_ASSERT_FALSE(RampShouldDecelerateNow(&r, dtPerTick, 0, 600));
}

static void test_jerk_limited_jog_reaches_targets_and_bounds_acceleration(void) {
  RampSetAcceleration(&r,1200); RampSetJerk(&r,120000);
  int targets[]={40,0,-40,0,1,0};
  int64_t priorA=0;
  for (unsigned t=0;t<sizeof(targets)/sizeof(targets[0]);t++) {
    RampSetTarget(&r,targets[t]);
    for (int i=0;i<300;i++) {
      RampSmoothTick(&r,SYSCLK_HZ/1000U);
      int64_t a=r.smoothAccelerationQ16;
      int64_t delta=a-priorA;if(delta<0)delta=-delta;
      TEST_ASSERT_TRUE(delta<=120*65536LL+16);
      TEST_ASSERT_TRUE(a<=1200*65536LL && a>=-1200*65536LL);
      priorA=a;
    }
    TEST_ASSERT_EQUAL_INT32(targets[t],r.currentVelocity);
    TEST_ASSERT_TRUE(r.smoothAccelerationQ16==0);
    TEST_ASSERT_TRUE(r.smoothVelocityQ16==(int64_t)targets[t]*65536);
  }
}

static void test_zero_jerk_retains_existing_acceleration_ramp(void) {
  RampSetAcceleration(&r,1200); RampSetJerk(&r,0); RampSetTarget(&r,40);
  RampState copy=r;
  for(int i=0;i<100;i++) TEST_ASSERT_EQUAL_INT32(RampTick(&copy,SYSCLK_HZ/1000U),RampSmoothTick(&r,SYSCLK_HZ/1000U));
}

static void test_jerk_ramp_can_reverse_during_acceleration(void) {
  RampSetAcceleration(&r,1200); RampSetJerk(&r,120000); RampSetTarget(&r,40);
  for(int i=0;i<5;i++) RampSmoothTick(&r,SYSCLK_HZ/1000U);
  RampSetTarget(&r,-40);
  for(int i=0;i<400;i++) RampSmoothTick(&r,SYSCLK_HZ/1000U);
  TEST_ASSERT_EQUAL_INT32(-40,r.currentVelocity);
  TEST_ASSERT_TRUE(r.smoothAccelerationQ16==0);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_jerk_ramp_can_reverse_during_acceleration);
  RUN_TEST(test_jerk_limited_jog_reaches_targets_and_bounds_acceleration);
  RUN_TEST(test_zero_jerk_retains_existing_acceleration_ramp);
  RUN_TEST(test_init_defaults);
  RUN_TEST(test_tick_with_no_target_change_stays_zero);
  RUN_TEST(test_tick_ramps_toward_positive_target_not_instantly);
  RUN_TEST(test_tick_never_overshoots_target);
  RUN_TEST(test_tick_ramps_toward_negative_target);
  RUN_TEST(test_reaching_target_then_holds);
  RUN_TEST(test_small_dt_takes_multiple_ticks_to_arrive);
  RUN_TEST(test_retargeting_mid_ramp_reverses_direction);
  RUN_TEST(test_accel_scales_with_configured_rate);
  RUN_TEST(test_error_accumulator_prevents_rate_starvation);
  RUN_TEST(test_peek_matches_what_tick_would_apply);
  RUN_TEST(test_peek_does_not_mutate_state);
  RUN_TEST(test_peek_is_nonnegative_for_a_negative_target);
  RUN_TEST(test_should_not_decelerate_from_standstill_far_from_target);
  RUN_TEST(test_should_decelerate_when_close_at_speed);
  RUN_TEST(test_slow_accel_relative_to_tick_rate_does_not_misfire);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
