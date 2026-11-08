/*
 * Native unit tests for the point-to-point ramped position move
 * (PositionMove.c) -- the primitive behind `position.command`. Pure
 * fixed-point tick counting, no HAL dependency -- natively testable.
 * Run: pio test -e native
 */
#include <unity.h>
#include "PositionMove.h"
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
 * See platformio.ini and test_signal.c/test_ramp.c for the same pattern. */
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

#ifndef SYSCLK_HZ
#error "SYSCLK_HZ must be defined (see platformio.ini's native env)"
#endif

static PositionMoveState m;
static RampState ramp;

void setUp(void) {
  PositionMoveInit(&m);
  RampInit(&ramp);
  RampSetAcceleration(&ramp, 1000);
}
void tearDown(void) {}

static const uint32_t kDt = SYSCLK_HZ / 1000U;   /* ~1ms, the real control-tick cadence */

static void test_init_inactive(void) {
  TEST_ASSERT_FALSE(PositionMoveIsActive(&m));
  TEST_ASSERT_EQUAL_INT32(0, PositionMoveGetTarget(&m));
}

static void test_set_target_activates(void) {
  PositionMoveSetTarget(&m, 5000);
  TEST_ASSERT_TRUE(PositionMoveIsActive(&m));
  TEST_ASSERT_EQUAL_INT32(5000, PositionMoveGetTarget(&m));
}

static void test_tick_while_inactive_is_noop(void) {
  int32_t v = PositionMoveTick(&m, &ramp, 0, 10000, kDt);
  TEST_ASSERT_EQUAL_INT32(0, v);
  TEST_ASSERT_EQUAL_INT32(0, ramp.currentVelocity);   /* untouched */
}

static void test_already_at_target_completes_immediately(void) {
  PositionMoveSetTarget(&m, 1234);
  int32_t v = PositionMoveTick(&m, &ramp, 1234, 10000, kDt);
  TEST_ASSERT_EQUAL_INT32(0, v);
  TEST_ASSERT_FALSE(PositionMoveIsActive(&m));
}

/* ---- full-trajectory invariants -- the properties that actually matter
 * for something eventually driving a real indexer: never pass the
 * target, never exceed the configured cruise cap, always finish, and
 * land EXACTLY on the requested count, not "close enough." ---------- */

static void test_long_positive_move_never_overshoots_and_lands_exactly(void) {
  int32_t target = 100000;
  int32_t maxVel = 5000;
  PositionMoveSetTarget(&m, target);
  int32_t pos = 0;
  int guard = 0;
  while (PositionMoveIsActive(&m)) {
    int32_t v = PositionMoveTick(&m, &ramp, pos, maxVel, kDt);
    pos += v;
    TEST_ASSERT_TRUE(pos <= target);
    int32_t absV = (v < 0) ? -v : v;
    TEST_ASSERT_TRUE(absV <= maxVel);
    guard++;
    TEST_ASSERT_TRUE(guard < 100000);   /* sanity bound -- must actually finish */
  }
  TEST_ASSERT_EQUAL_INT32(target, pos);
}

static void test_long_negative_move_never_overshoots_and_lands_exactly(void) {
  int32_t target = -75000;
  int32_t maxVel = 3000;
  PositionMoveSetTarget(&m, target);
  int32_t pos = 0;
  int guard = 0;
  while (PositionMoveIsActive(&m)) {
    int32_t v = PositionMoveTick(&m, &ramp, pos, maxVel, kDt);
    pos += v;
    TEST_ASSERT_TRUE(pos >= target);
    int32_t absV = (v < 0) ? -v : v;
    TEST_ASSERT_TRUE(absV <= maxVel);
    guard++;
    TEST_ASSERT_TRUE(guard < 100000);
  }
  TEST_ASSERT_EQUAL_INT32(target, pos);
}

static void test_short_move_shorter_than_one_accel_step_lands_exactly(void) {
  /* accelPerTick at 1000 counts/tick/sec, 1ms dt is ~1 count/tick -- a
   * 3-count move is well under that, so this exercises the "final-tick
   * snap" path almost immediately rather than ever really ramping. */
  PositionMoveSetTarget(&m, 3);
  int32_t pos = 0;
  int guard = 0;
  while (PositionMoveIsActive(&m)) {
    pos += PositionMoveTick(&m, &ramp, pos, 5000, kDt);
    guard++;
    TEST_ASSERT_TRUE(guard < 1000);
  }
  TEST_ASSERT_EQUAL_INT32(3, pos);
}

static void test_moderate_move_lands_exactly(void) {
  /* Big enough to actually ramp up and back down, small enough to never
   * reach maxVel -- a pure triangular (accel-then-decel, no cruise)
   * profile, the other half of the trapezoidal/triangular distinction. */
  PositionMoveSetTarget(&m, 500);
  int32_t pos = 0;
  int guard = 0;
  int32_t maxAbsVelSeen = 0;
  while (PositionMoveIsActive(&m)) {
    int32_t v = PositionMoveTick(&m, &ramp, pos, 5000, kDt);
    pos += v;
    int32_t absV = (v < 0) ? -v : v;
    if (absV > maxAbsVelSeen) maxAbsVelSeen = absV;
    TEST_ASSERT_TRUE(pos <= 500);
    guard++;
    TEST_ASSERT_TRUE(guard < 10000);
  }
  TEST_ASSERT_EQUAL_INT32(500, pos);
  TEST_ASSERT_TRUE(maxAbsVelSeen < 5000);   /* never needed to reach the cap for a move
                                                this short -- confirms it's actually a
                                                triangular profile, not clipped cruise */
}

static void test_slow_accel_relative_to_tick_rate_still_moves(void) {
  /* Regression guard: trajectory.accel_limit=300 (counts/tick/sec) at a real ~1kHz tick is only
   * 0.3 counts/tick of raw per-tick accel -- TRUNCATED TO EXACTLY 0 if read back out as a
   * one-shot integer (RampPeekAccelPerTick()'s own correct, documented behavior for ITS
   * purpose). The v^2>=2*a*d decel check must NOT use that truncated value directly: doing
   * so degenerates to 0>=0 (always true) the instant velocity is 0, which decides "must
   * already be decelerating" before ever accelerating at all, for any target distance, so
   * the move never moves a single count. RampShouldDecelerateNow() avoids this by never
   * truncating before the comparison -- this test uses the exact accel value (300) that
   * triggers the failure mode if that guarantee ever regresses. */
  RampSetAcceleration(&ramp, 300);
  PositionMoveSetTarget(&m, 600);
  int32_t pos = 0;
  int guard = 0;
  int32_t sawNonzeroVelocity = 0;
  while (PositionMoveIsActive(&m)) {
    int32_t v = PositionMoveTick(&m, &ramp, pos, 150, kDt);
    pos += v;
    if (v != 0) sawNonzeroVelocity = 1;
    TEST_ASSERT_TRUE(pos <= 600);
    guard++;
    TEST_ASSERT_TRUE(guard < 50000);   /* must actually finish, not stall forever */
  }
  TEST_ASSERT_TRUE(sawNonzeroVelocity);   /* the bug: this would be false -- stuck at
                                              v=0 the whole time, loop only exiting via
                                              the guard bound above, never reaching 600 */
  TEST_ASSERT_EQUAL_INT32(600, pos);
}

static void test_stop_mid_move_deactivates_without_changing_target(void) {
  PositionMoveSetTarget(&m, 9999);
  PositionMoveTick(&m, &ramp, 0, 5000, kDt);
  TEST_ASSERT_TRUE(PositionMoveIsActive(&m));

  PositionMoveStop(&m);
  TEST_ASSERT_FALSE(PositionMoveIsActive(&m));
  TEST_ASSERT_EQUAL_INT32(9999, PositionMoveGetTarget(&m));   /* remembers what it was
                                                                   asked, same convention
                                                                   as AppSignalStop() */

  int32_t v = PositionMoveTick(&m, &ramp, 500, 5000, kDt);
  TEST_ASSERT_EQUAL_INT32(0, v);   /* inactive -- no further commands */
}

static void test_retarget_mid_move_lands_on_new_target(void) {
  PositionMoveSetTarget(&m, 100000);
  int32_t pos = 0;
  int32_t maxVel = 5000;
  for (int i = 0; i < 200; i++) pos += PositionMoveTick(&m, &ramp, pos, maxVel, kDt);
  TEST_ASSERT_TRUE(pos > 0);   /* genuinely underway */

  int32_t newTarget = pos - 10000;   /* reverse -- target now behind current position */
  PositionMoveSetTarget(&m, newTarget);
  int guard = 0;
  while (PositionMoveIsActive(&m)) {
    pos += PositionMoveTick(&m, &ramp, pos, maxVel, kDt);
    guard++;
    TEST_ASSERT_TRUE(guard < 100000);
  }
  TEST_ASSERT_EQUAL_INT32(newTarget, pos);
}

static void test_shares_ramp_state_with_an_existing_jog_no_sudden_jump(void) {
  /* Simulates PositionMoveSetTarget() being issued while the SAME
   * RampState is already mid-jog from a plain velocity.command (the
   * whole reason PositionMoveTick() takes the caller's RampState instead
   * of owning its own) -- the first position-move tick must not command
   * a velocity wildly different from what the ramp was already doing. */
  RampSetTarget(&ramp, 50000);
  for (int i = 0; i < 500; i++) RampTick(&ramp, kDt);
  int32_t priorVelocity = ramp.currentVelocity;
  TEST_ASSERT_TRUE(priorVelocity > 0);

  int32_t pos = 100000;
  PositionMoveSetTarget(&m, 150000);
  int32_t firstV = PositionMoveTick(&m, &ramp, pos, 5000, kDt);

  int32_t accelPerTick = RampPeekAccelPerTick(&ramp, kDt);
  int32_t delta = firstV - priorVelocity;
  if (delta < 0) delta = -delta;
  TEST_ASSERT_TRUE(delta <= accelPerTick + 1);   /* at most one tick's worth of change,
                                                      whichever direction -- no discontinuity */
}

/* Regression for the live bench failure: after completing a reference
 * move, the ordinary zero-jog path must hold that endpoint indefinitely. */
static void test_completed_move_holds_reference_through_jog_handoff(void) {
  RampSetAcceleration(&ramp, 300);
  for (int direction = -1; direction <= 1; direction += 2) {
    PositionMoveInit(&m);
    RampInit(&ramp);
    RampSetAcceleration(&ramp, 300);
    int32_t reference = 1000;
    int32_t target = reference + direction * 512;
    PositionMoveSetTarget(&m, target);
    int ticks = 0;
    while (PositionMoveIsActive(&m)) {
      reference += PositionMoveTick(&m, &ramp, reference, 10, kDt);
      TEST_ASSERT_TRUE(++ticks < 10000);
    }
    TEST_ASSERT_EQUAL_INT32(target, reference);
    TEST_ASSERT_EQUAL_INT32(0, ramp.currentVelocity);
    TEST_ASSERT_EQUAL_INT32(0, ramp.targetVelocity);
    for (int i = 0; i < 1000; i++) {
      RampSetTarget(&ramp, 0);
      reference += RampTick(&ramp, kDt);
    }
    TEST_ASSERT_EQUAL_INT32(target, reference);
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_completed_move_holds_reference_through_jog_handoff);
  RUN_TEST(test_init_inactive);
  RUN_TEST(test_set_target_activates);
  RUN_TEST(test_tick_while_inactive_is_noop);
  RUN_TEST(test_already_at_target_completes_immediately);
  RUN_TEST(test_long_positive_move_never_overshoots_and_lands_exactly);
  RUN_TEST(test_long_negative_move_never_overshoots_and_lands_exactly);
  RUN_TEST(test_short_move_shorter_than_one_accel_step_lands_exactly);
  RUN_TEST(test_moderate_move_lands_exactly);
  RUN_TEST(test_slow_accel_relative_to_tick_rate_still_moves);
  RUN_TEST(test_stop_mid_move_deactivates_without_changing_target);
  RUN_TEST(test_retarget_mid_move_lands_on_new_target);
  RUN_TEST(test_shares_ramp_state_with_an_existing_jog_no_sudden_jump);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
