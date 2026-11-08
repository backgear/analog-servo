/*
 * Native unit tests for the built-in open-loop test-signal generator
 * (Signal.c). Pure fixed-point tick counting, no HAL/mocks needed at all.
 * Run: pio test -e native
 */
#include <unity.h>
#include "Signal.h"
#include "Control.h"
#include "main.h"
#include "Encoder.h"
#include "Output.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Ramp.c/Fault.c/Protocol.c (and everything they
 * call into) get linked into every test/test_* executable, this one
 * included. See platformio.ini and analog_quadrature's test suites for the
 * same pattern. */
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
int AppResetPosition(void) { return 0; }
void AppGetControlSample(AppControlSample *out) { *out = (AppControlSample){0}; }
int32_t AppGetLastVelocityActual(void) { return 0; }
int32_t AppGetLastTorqueCommand(void) { return 0; }
int32_t AppGetLastCommandedPosition(void) { return 0; }
int32_t AppGetLastCommandedVelocity(void) { return 0; }
int        AppGetSignalCycleStart(void) { return 0; }
int32_t EncoderGetPositionCounts(void) { return 0; }
int     EncoderZWasSeenSinceEnable(void) { return 0; }
void    UsartTransmit(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { (void)Buf; (void)Len; return USBD_OK; }
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel, uint8_t *pbuff, uint16_t length) { (void)pdev; (void)channel; (void)pbuff; (void)length; return USBD_OK; }
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) { (void)pdev; (void)channel; return USBD_OK; }

void setUp(void) { SignalInit(); }
void tearDown(void) {}

static void test_idle_produces_zero(void) {
  TEST_ASSERT_FALSE(SignalIsActive());
  TEST_ASSERT_EQUAL_INT(SIGNAL_NONE, SignalGetType());
  TEST_ASSERT_EQUAL_INT32(0, SignalTick());
}

static void test_target_defaults_to_torque(void) {
  TEST_ASSERT_EQUAL_INT(SIGNAL_TARGET_TORQUE, SignalGetTarget());
}

static void test_target_position_is_recorded(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_POSITION, 500, 10, 0);
  TEST_ASSERT_EQUAL_INT(SIGNAL_TARGET_POSITION, SignalGetTarget());
  /* SignalTick()'s waveform math itself doesn't care what the target means --
   * it's the same +-amplitude output either way; main.c decides how to use it. */
  TEST_ASSERT_EQUAL_INT32(500, SignalTick());
}

static void test_target_resets_to_torque_on_init(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_POSITION, 500, 10, 0);
  SignalInit();
  TEST_ASSERT_EQUAL_INT(SIGNAL_TARGET_TORQUE, SignalGetTarget());
}

/* ---- proving position-mode actually runs the closed loop, not just
 * torque-mode-with-a-different-label -------------------------------------
 *
 * With the DEFAULT gains (posGainQ8=256, velGainKpQ8=256 -- both exactly 1.0
 * in Q8.8) and a plant that never moves (no motor attached, or genuinely
 * stalled), the P->PI cascade's output degenerates to torqueCommand=posErr
 * exactly, tick after tick -- so it LOOKS pinned at +-amplitude, identical
 * to torque mode's raw bang-bang, purely because of that specific gain/
 * no-feedback combination, not because the code silently took the torque
 * branch. These tests reproduce that exact case, then prove it's really
 * going through Control.c by (a) showing a non-unity gain changes the
 * output, and (b) showing a plant that DOES respond makes the torque
 * command converge toward zero instead of staying pinned -- something a
 * raw open-loop square wave could never do. */

static void test_position_target_matches_torque_amplitude_with_unity_gains_and_stalled_plant(void) {
  ControlGains gains = {
    .posGainQ8 = 256, .velGainKpQ8 = 256, .velGainKiQ8 = 0,
    .velocityClamp = 100000, .torqueClampMin = -100000, .torqueClampMax = 100000,
  };
  ControlInit(&gains);
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_POSITION, 100, 100000, 0);

  int32_t basePosition = 0;
  int32_t stalledPositionActual = 0;   /* never moves -- e.g. no motor attached */
  for (int i = 0; i < 5; i++) {
    int32_t signalValue = SignalTick();
    int32_t positionTarget = basePosition + signalValue;
    int32_t torque = ControlTick(positionTarget, stalledPositionActual, 0, 1000);
    /* This is the exact behavior reported: with unity gains and zero real
     * feedback, torqueCommand == amplitude, every tick -- confirmed
     * expected, not a bug. */
    TEST_ASSERT_EQUAL_INT32(100, torque);
  }
}

static void test_position_target_differs_from_amplitude_with_nondefault_gains(void) {
  ControlGains gains = {
    .posGainQ8 = 128 /* 0.5 */, .velGainKpQ8 = 256, .velGainKiQ8 = 0,
    .velocityClamp = 100000, .torqueClampMin = -100000, .torqueClampMax = 100000,
  };
  ControlInit(&gains);
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_POSITION, 100, 100000, 0);

  int32_t signalValue = SignalTick();
  int32_t torque = ControlTick(0 + signalValue, 0, 0, 1000);
  /* posGainQ8=0.5 halves the velocity target (and therefore the resulting
   * torque, since velGainKp=1.0) relative to the raw amplitude -- if this
   * came out 100 regardless of the gain, that would mean the cascade
   * wasn't actually being consulted. */
  TEST_ASSERT_EQUAL_INT32(50, torque);
}

static void test_position_target_converges_with_a_responsive_plant(void) {
  ControlGains gains = {
    .posGainQ8 = 256, .velGainKpQ8 = 512, .velGainKiQ8 = 26,
    .velocityClamp = 100000, .torqueClampMin = -1000, .torqueClampMax = 1000,
  };
  ControlInit(&gains);
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_POSITION, 1000, 100000, 0);   /* huge half-period:
                                                                             stays on the + side
                                                                             for this whole test */

  int32_t basePosition = 0;
  int32_t pos = 0, vel = 0;
  int32_t firstTorque = 0, lastTorque = 0;
  for (int i = 0; i < 300; i++) {
    int32_t signalValue = SignalTick();
    int32_t positionTarget = basePosition + signalValue;
    int32_t torque = ControlTick(positionTarget, pos, vel, 1000);
    if (i == 0) firstTorque = torque;
    lastTorque = torque;
    /* Same toy plant as test_control.c: velocity integrates torque, position
     * integrates velocity -- just enough closed-loop dynamics to respond. */
    vel += torque / 16;
    pos += vel;
  }
  /* If this were open-loop torque mode (or position mode silently not
   * consulting the real position feedback), torque would stay pinned at
   * 1000 the entire run, and position would run away rather than settle.
   * With a responsive plant actually closing the loop, torque must fall
   * off well below its initial/clamped value as the position loop's error
   * shrinks, and position must land near the target -- neither of which a
   * raw torque square wave could ever produce. */
  TEST_ASSERT_EQUAL_INT32(1000, firstTorque);   /* starts pinned at the clamp, same as torque mode would */
  TEST_ASSERT_TRUE(lastTorque < 500);            /* but falls off as the plant actually responds */
  TEST_ASSERT_TRUE((pos > 800) && (pos < 1200)); /* and position actually converges near the target */
}

static void test_impulse_fires_once_then_stops(void) {
  SignalStart(SIGNAL_IMPULSE, SIGNAL_TARGET_TORQUE, 12345, 1, 0);
  TEST_ASSERT_TRUE(SignalIsActive());
  TEST_ASSERT_EQUAL_INT32(12345, SignalTick());
  TEST_ASSERT_FALSE(SignalIsActive());
  TEST_ASSERT_EQUAL_INT32(0, SignalTick());   /* subsequent ticks are silent */
}

static void test_impulse_negative_amplitude(void) {
  SignalStart(SIGNAL_IMPULSE, SIGNAL_TARGET_TORQUE, -500, 1, 0);
  TEST_ASSERT_EQUAL_INT32(-500, SignalTick());
  TEST_ASSERT_FALSE(SignalIsActive());
}

static void test_square_alternates_at_half_period(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 1000, 3, 0);   /* half-period = 3 ticks */
  int32_t expect[] = { 1000, 1000, 1000, -1000, -1000, -1000, 1000, 1000 };
  for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
    TEST_ASSERT_EQUAL_INT32(expect[i], SignalTick());
  }
  TEST_ASSERT_TRUE(SignalIsActive());   /* square runs until stopped/duration */
}

static void test_square_zero_half_period_treated_as_one(void) {
  /* SignalStart clamps a zero half-period to 1 rather than dividing by zero
   * or never toggling. */
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 500, 0, 0);
  TEST_ASSERT_EQUAL_INT32(500, SignalTick());
  TEST_ASSERT_EQUAL_INT32(-500, SignalTick());
  TEST_ASSERT_EQUAL_INT32(500, SignalTick());
}

static void test_square_duration_cap_stops_generator(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 100, 1, 4);
  for (int i = 0; i < 4; i++) SignalTick();
  TEST_ASSERT_TRUE(SignalIsActive());   /* still active for exactly `duration` ticks */
  TEST_ASSERT_EQUAL_INT32(0, SignalTick());   /* 5th tick: duration exceeded */
  TEST_ASSERT_FALSE(SignalIsActive());
}

static void test_stop_halts_immediately(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 100, 5, 0);
  SignalTick();
  SignalTick();
  SignalStop();
  TEST_ASSERT_FALSE(SignalIsActive());
  TEST_ASSERT_EQUAL_INT32(0, SignalTick());
}

static void test_cycle_start_pulse_fires_once_per_cycle(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 1000, 3, 0);   /* half-period = 3 ticks */
  int expectPulse[] = { 1, 0, 0, 0, 0, 0, 1, 0 };   /* ticks: +,+,+,-,-,-,+,+ -- pulse only
                                                          on the first tick of each + half */
  for (size_t i = 0; i < sizeof(expectPulse) / sizeof(expectPulse[0]); i++) {
    SignalTick();
    TEST_ASSERT_EQUAL_INT(expectPulse[i], SignalCycleStartPulse());
  }
}

static void test_cycle_start_pulse_zero_when_idle(void) {
  TEST_ASSERT_FALSE(SignalCycleStartPulse());
  SignalTick();   /* SIGNAL_NONE -- stays 0 */
  TEST_ASSERT_FALSE(SignalCycleStartPulse());
}

static void test_cycle_start_pulse_zero_for_impulse(void) {
  SignalStart(SIGNAL_IMPULSE, SIGNAL_TARGET_TORQUE, 500, 1, 0);
  SignalTick();
  TEST_ASSERT_FALSE(SignalCycleStartPulse());   /* impulse has no cycle concept */
}

static void test_cycle_start_pulse_clears_on_stop(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 100, 5, 0);
  SignalTick();
  TEST_ASSERT_TRUE(SignalCycleStartPulse());
  SignalStop();
  /* A stopped signal gets no further SignalTick() calls from main.c, so with
   * nothing to naturally zero this pulse again, SignalStop() must clear it
   * itself -- otherwise a host reading AppGetSignalCycleStart() every tick
   * (Monitor.c's status bit7, active or not) would see a permanently
   * "stuck high" cycle-start marker after whatever happened to be the last
   * active tick before the stop. */
  TEST_ASSERT_FALSE(SignalCycleStartPulse());
}

static void test_start_replaces_running_signal(void) {
  SignalStart(SIGNAL_SQUARE, SIGNAL_TARGET_TORQUE, 100, 10, 0);
  SignalTick();
  SignalStart(SIGNAL_IMPULSE, SIGNAL_TARGET_TORQUE, 999, 1, 0);   /* replaces the square wave outright */
  TEST_ASSERT_EQUAL_INT(SIGNAL_IMPULSE, SignalGetType());
  TEST_ASSERT_EQUAL_INT32(999, SignalTick());
  TEST_ASSERT_FALSE(SignalIsActive());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_idle_produces_zero);
  RUN_TEST(test_target_defaults_to_torque);
  RUN_TEST(test_target_position_is_recorded);
  RUN_TEST(test_target_resets_to_torque_on_init);
  RUN_TEST(test_position_target_matches_torque_amplitude_with_unity_gains_and_stalled_plant);
  RUN_TEST(test_position_target_differs_from_amplitude_with_nondefault_gains);
  RUN_TEST(test_position_target_converges_with_a_responsive_plant);
  RUN_TEST(test_impulse_fires_once_then_stops);
  RUN_TEST(test_impulse_negative_amplitude);
  RUN_TEST(test_square_alternates_at_half_period);
  RUN_TEST(test_square_zero_half_period_treated_as_one);
  RUN_TEST(test_square_duration_cap_stops_generator);
  RUN_TEST(test_stop_halts_immediately);
  RUN_TEST(test_cycle_start_pulse_fires_once_per_cycle);
  RUN_TEST(test_cycle_start_pulse_zero_when_idle);
  RUN_TEST(test_cycle_start_pulse_zero_for_impulse);
  RUN_TEST(test_cycle_start_pulse_clears_on_stop);
  RUN_TEST(test_start_replaces_running_signal);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
