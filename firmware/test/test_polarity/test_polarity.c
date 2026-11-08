/*
 * Native unit tests for the polarity-learn pass/fail decision (Polarity.c).
 * Pure fixed-point math, no HAL/mocks needed at all. PolarityLearnEvaluate
 * requires a minimum movement magnitude before trusting the observed sign
 * -- a disconnected motor, a seized shaft, or an unplugged encoder all
 * observe zero movement, and must NOT "succeed" by silently defaulting
 * the sign to +1.
 * Run: pio test -e native
 */
#include <unity.h>
#include "Polarity.h"
#include "main.h"
#include "Encoder.h"
#include "Output.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Ramp.c/Fault.c/Signal.c/Protocol.c (and
 * everything they call into) get linked into every test/test_* executable,
 * this one included. See platformio.ini and analog_quadrature's test suites
 * for the same pattern. */
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
int     AppPolarityLearnBusy(void) { return 0; }
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

void setUp(void) {}
void tearDown(void) {}

static void test_no_movement_fails(void) {
  int8_t sign = 0;
  TEST_ASSERT_FALSE(PolarityLearnEvaluate(0, &sign));
}

static void test_tiny_jitter_fails(void) {
  /* Below the threshold -- e.g. encoder noise/backlash with no real motion,
   * still must not report success. */
  int8_t sign = 0;
  TEST_ASSERT_FALSE(PolarityLearnEvaluate(POLARITY_LEARN_MIN_MOVEMENT_COUNTS - 1, &sign));
  TEST_ASSERT_FALSE(PolarityLearnEvaluate(-(POLARITY_LEARN_MIN_MOVEMENT_COUNTS - 1), &sign));
}

static void test_at_threshold_succeeds(void) {
  int8_t sign = 0;
  TEST_ASSERT_TRUE(PolarityLearnEvaluate(POLARITY_LEARN_MIN_MOVEMENT_COUNTS, &sign));
  TEST_ASSERT_EQUAL_INT(1, sign);
}

static void test_positive_movement_gives_positive_sign(void) {
  int8_t sign = 0;
  TEST_ASSERT_TRUE(PolarityLearnEvaluate(1000, &sign));
  TEST_ASSERT_EQUAL_INT(1, sign);
}

static void test_negative_movement_gives_negative_sign(void) {
  int8_t sign = 0;
  TEST_ASSERT_TRUE(PolarityLearnEvaluate(-1000, &sign));
  TEST_ASSERT_EQUAL_INT(-1, sign);
}

static void test_negative_at_threshold_succeeds(void) {
  int8_t sign = 0;
  TEST_ASSERT_TRUE(PolarityLearnEvaluate(-POLARITY_LEARN_MIN_MOVEMENT_COUNTS, &sign));
  TEST_ASSERT_EQUAL_INT(-1, sign);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_no_movement_fails);
  RUN_TEST(test_tiny_jitter_fails);
  RUN_TEST(test_at_threshold_succeeds);
  RUN_TEST(test_positive_movement_gives_positive_sign);
  RUN_TEST(test_negative_movement_gives_negative_sign);
  RUN_TEST(test_negative_at_threshold_succeeds);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
