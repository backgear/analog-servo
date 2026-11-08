/*
 * Native unit tests for Mode C (analog velocity)'s scale/hysteresis math
 * (Analog.c). Pure logic, no HAL -- no mocks needed for Analog.c itself.
 * Run: pio test -e native
 */
#include <unity.h>
#include "Analog.h"
#include "main.h"
#include "Encoder.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Fault.c/Protocol.c (and everything they call
 * into) get linked into every test/test_* executable, this one included.
 * See platformio.ini and analog_quadrature's test suites for the same
 * pattern. */
void OutputForceSafe(void) {}
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

void setUp(void) {
  AnalogVelocitySetConfig(256, 0);   /* unity scale, hysteresis off -- most tests opt into
                                         something tighter */
  AnalogVelocityReset();
}
void tearDown(void) {}

static void test_unity_scale_passes_raw_code_through(void) {
  TEST_ASSERT_EQUAL_INT32(1000, AnalogVelocityTick(1000, 0));
  TEST_ASSERT_EQUAL_INT32(0, AnalogVelocityTick(0, 0));
}

static void test_reversed_negates_the_result(void) {
  TEST_ASSERT_EQUAL_INT32(-1000, AnalogVelocityTick(1000, 1));
}

static void test_scale_below_unity_divides_down(void) {
  AnalogVelocitySetConfig(64, 0);   /* 0.25 in Q8.8 */
  TEST_ASSERT_EQUAL_INT32(250, AnalogVelocityTick(1000, 0));
}

static void test_scale_zero_always_yields_zero(void) {
  AnalogVelocitySetConfig(0, 0);
  TEST_ASSERT_EQUAL_INT32(0, AnalogVelocityTick(4095, 0));
  TEST_ASSERT_EQUAL_INT32(0, AnalogVelocityTick(4095, 1));
}

static void test_first_tick_always_acts_immediately_even_with_hysteresis(void) {
  AnalogVelocitySetConfig(256, 100);
  TEST_ASSERT_EQUAL_INT32(50, AnalogVelocityTick(50, 0));
}

static void test_hysteresis_holds_small_moves(void) {
  AnalogVelocitySetConfig(256, 10);
  TEST_ASSERT_EQUAL_INT32(100, AnalogVelocityTick(100, 0));
  /* Within the band -- the acted-on code doesn't move, output stays put. */
  TEST_ASSERT_EQUAL_INT32(100, AnalogVelocityTick(105, 0));
  TEST_ASSERT_EQUAL_INT32(100, AnalogVelocityTick(95, 0));
  TEST_ASSERT_EQUAL_INT32(100, AnalogVelocityTick(109, 0));
}

static void test_hysteresis_releases_past_the_band(void) {
  AnalogVelocitySetConfig(256, 10);
  AnalogVelocityTick(100, 0);
  /* >= 10 away from the last acted-on code (100) -- takes effect. */
  TEST_ASSERT_EQUAL_INT32(110, AnalogVelocityTick(110, 0));
  /* New reference is now 110 -- a move back to 103 (7 away) is held again. */
  TEST_ASSERT_EQUAL_INT32(110, AnalogVelocityTick(103, 0));
}

static void test_hysteresis_disabled_at_zero_or_negative(void) {
  AnalogVelocitySetConfig(256, 0);
  AnalogVelocityTick(100, 0);
  TEST_ASSERT_EQUAL_INT32(101, AnalogVelocityTick(101, 0));
  AnalogVelocitySetConfig(256, -5);
  TEST_ASSERT_EQUAL_INT32(102, AnalogVelocityTick(102, 0));
}

static void test_reset_forces_immediate_reacquire(void) {
  AnalogVelocitySetConfig(256, 1000);
  AnalogVelocityTick(2000, 0);
  /* Still within the huge hysteresis band -- held. */
  TEST_ASSERT_EQUAL_INT32(2000, AnalogVelocityTick(2500, 0));
  AnalogVelocityReset();
  /* Reset clears the "have a reference yet" state -- next tick acts immediately. */
  TEST_ASSERT_EQUAL_INT32(2500, AnalogVelocityTick(2500, 0));
}

static void test_reversed_sign_follows_the_acted_on_code_not_the_raw_one(void) {
  AnalogVelocitySetConfig(256, 10);
  AnalogVelocityTick(100, 0);
  /* Held within the band, but reversed still negates whatever the acted-on
   * magnitude currently is. */
  TEST_ASSERT_EQUAL_INT32(-100, AnalogVelocityTick(105, 1));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_unity_scale_passes_raw_code_through);
  RUN_TEST(test_reversed_negates_the_result);
  RUN_TEST(test_scale_below_unity_divides_down);
  RUN_TEST(test_scale_zero_always_yields_zero);
  RUN_TEST(test_first_tick_always_acts_immediately_even_with_hysteresis);
  RUN_TEST(test_hysteresis_holds_small_moves);
  RUN_TEST(test_hysteresis_releases_past_the_band);
  RUN_TEST(test_hysteresis_disabled_at_zero_or_negative);
  RUN_TEST(test_reset_forces_immediate_reacquire);
  RUN_TEST(test_reversed_sign_follows_the_acted_on_code_not_the_raw_one);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
