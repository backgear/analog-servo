/*
 * Native unit tests for the relay-feedback velocity-loop autotune
 * (Autotune.c). Pure fixed-point-in/float-out tick counting, no HAL/mocks
 * needed for Autotune.c itself -- the stubs below exist only because
 * [env:native]'s build_src_filter is env-wide, so Protocol.c (and everything
 * it calls into) gets linked into every test/test_* executable, this one
 * included. Same pattern as test_signal.c/test_polarity.c.
 * Run: pio test -e native
 */
#include <unity.h>
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
/* AppAutotune* are declared in main.h and called from Protocol.c, but this
 * suite exercises Autotune.c's own AutotuneStartRelay/Tick/etc. directly
 * (not through the App wrapper), so these just need to exist for the
 * linker. */
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

void setUp(void) { AutotuneInit(); }
void tearDown(void) {}

static void test_idle_produces_zero(void) {
  TEST_ASSERT_FALSE(AutotuneIsActive());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NONE, AutotuneGetResult());
  TEST_ASSERT_EQUAL_INT32(0, AutotuneTick(0));
}

static void test_start_goes_active(void) {
  AutotuneStartRelay(1000, 0);
  TEST_ASSERT_TRUE(AutotuneIsActive());
}

static void test_first_tick_commands_positive_amplitude(void) {
  AutotuneStartRelay(1000, 0);
  TEST_ASSERT_EQUAL_INT32(1000, AutotuneTick(0));   /* starts pushing +amplitude */
}

static void test_relay_holds_until_hysteresis_crossed(void) {
  AutotuneStartRelay(1000, 0);
  /* Velocity sitting inside the hysteresis band never triggers a switch --
   * the relay keeps commanding +amplitude every tick. */
  for (int i = 0; i < 20; i++) {
    TEST_ASSERT_EQUAL_INT32(1000, AutotuneTick(1));
  }
}

static void test_relay_switches_on_hysteresis_crossing(void) {
  AutotuneStartRelay(1000, 0);
  AutotuneTick(0);                       /* +amplitude, no crossing */
  int32_t out = AutotuneTick(50);        /* crosses +hysteresis -- switches to -amplitude */
  TEST_ASSERT_EQUAL_INT32(-1000, out);
  out = AutotuneTick(0);
  TEST_ASSERT_EQUAL_INT32(-1000, out);   /* stays low until it crosses -hysteresis */
}

/* Feeds a clean synthetic oscillation: each half-cycle is (halfPeriod-1)
 * ticks inside the hysteresis band (velocity 0), then one tick at
 * +-peakVelocity that triggers the switch and also becomes that half-
 * cycle's recorded peak -- gives an exactly-known period (halfPeriod*2
 * ticks between same-direction switches) and amplitude (peakVelocity) to
 * check the Ku/Pu math against by hand. Returns the final tick's output. */
static int32_t RunSyntheticOscillation(int32_t amplitude, uint32_t halfPeriod,
                                        int32_t peakVelocity, int numHalfCycles) {
  int32_t out = 0;
  for (int cycle = 0; cycle < numHalfCycles; cycle++) {
    int32_t crossingVelocity = (cycle % 2 == 0) ? peakVelocity : -peakVelocity;
    for (uint32_t t = 0; t < halfPeriod - 1U; t++) out = AutotuneTick(0);
    out = AutotuneTick(crossingVelocity);
    (void)amplitude;
  }
  return out;
}

static void test_settle_and_measure_produces_ok_result(void) {
  /* AUTOTUNE_SETTLE_SWITCHES=2, AUTOTUNE_MEASURE_SWITCHES=6 (Autotune.c) --
   * 8 half-cycles completes exactly one full attempt. */
  AutotuneStartRelay(1000, 0);
  int32_t lastOut = RunSyntheticOscillation(1000, 50U, 80, 8);

  TEST_ASSERT_FALSE(AutotuneIsActive());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());
  TEST_ASSERT_EQUAL_INT32(0, lastOut);   /* done -- no more torque commanded */

  TEST_ASSERT_EQUAL_UINT32(100U, AutotuneGetMeasuredPeriodTicks());   /* 2 * halfPeriod, exact */
  /* Hand-computed (double precision) from Ku=4*1000/(pi*80), Kp=0.45*Ku,
   * Ti=100/1.2, Ki=Kp/Ti, posP=(1.2/100)/5 -- +-2 counts tolerance for the
   * firmware's float (not double) arithmetic. */
  TEST_ASSERT_INT32_WITHIN(2, 1833, AutotuneGetKpResultQ8());
  TEST_ASSERT_INT32_WITHIN(2, 22,   AutotuneGetKiResultQ8());
  TEST_ASSERT_INT32_WITHIN(2, 1,    AutotuneGetPosPResultQ8());
}

static void test_no_oscillation_within_max_ticks_fails(void) {
  AutotuneStartRelay(1000, 500U);   /* small cap, well under what a real oscillation needs */
  for (uint32_t i = 0; i < 500U; i++) {
    AutotuneTick(0);   /* always inside the hysteresis band -- never switches */
  }
  TEST_ASSERT_FALSE(AutotuneIsActive());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NO_OSCILLATION, AutotuneGetResult());
}

static void test_explicit_stop_before_result_is_no_oscillation(void) {
  AutotuneStartRelay(1000, 0);
  AutotuneTick(0);
  AutotuneTick(50);   /* one switch happened, nowhere near enough */
  AutotuneStop();
  TEST_ASSERT_FALSE(AutotuneIsActive());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NO_OSCILLATION, AutotuneGetResult());
}

static void test_stop_after_result_keeps_result(void) {
  AutotuneStartRelay(1000, 0);
  RunSyntheticOscillation(1000, 50U, 80, 8);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());
  AutotuneStop();   /* already idle/finished -- must not clobber the OK result */
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());
}

static void test_tick_while_idle_is_a_noop(void) {
  TEST_ASSERT_EQUAL_INT32(0, AutotuneTick(12345));
  TEST_ASSERT_FALSE(AutotuneIsActive());
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NONE, AutotuneGetResult());
}

static void test_negative_amplitude_still_produces_ok_result(void) {
  /* The relay is polarity-agnostic (see Autotune.h) -- a negative amplitude
   * just flips which literal sign is commanded each half-cycle, the Ku/Pu
   * math only ever uses magnitudes, so this should tune just as cleanly. */
  AutotuneStartRelay(-1000, 0);
  RunSyntheticOscillation(-1000, 50U, 80, 8);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());
  TEST_ASSERT_TRUE(AutotuneGetKpResultQ8() > 0);
  TEST_ASSERT_TRUE(AutotuneGetKiResultQ8() > 0);
}

static void test_restart_clears_previous_result(void) {
  AutotuneStartRelay(1000, 0);
  RunSyntheticOscillation(1000, 50U, 80, 8);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());

  AutotuneStartRelay(1000, 0);   /* fresh attempt */
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_NONE, AutotuneGetResult());
  TEST_ASSERT_TRUE(AutotuneIsActive());
}

static void test_method_reported_correctly(void) {
  AutotuneStartRelay(1000, 0);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_METHOD_RELAY, AutotuneGetMethod());
}

static void test_relay_method_leaves_gain_result_zero(void) {
  /* AutotuneGetMeasuredGainQ8() is kept only for Protocol.c's
   * `autotune.plant_gain` shape (it was a step-method-only diagnostic) --
   * confirm a relay run never leaves a stale/nonzero value. */
  AutotuneStartRelay(1000, 0);
  RunSyntheticOscillation(1000, 50U, 80, 8);
  TEST_ASSERT_EQUAL_INT(AUTOTUNE_RESULT_OK, AutotuneGetResult());
  TEST_ASSERT_EQUAL_INT32(0, AutotuneGetMeasuredGainQ8());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_idle_produces_zero);
  RUN_TEST(test_start_goes_active);
  RUN_TEST(test_first_tick_commands_positive_amplitude);
  RUN_TEST(test_relay_holds_until_hysteresis_crossed);
  RUN_TEST(test_relay_switches_on_hysteresis_crossing);
  RUN_TEST(test_settle_and_measure_produces_ok_result);
  RUN_TEST(test_no_oscillation_within_max_ticks_fails);
  RUN_TEST(test_explicit_stop_before_result_is_no_oscillation);
  RUN_TEST(test_stop_after_result_keeps_result);
  RUN_TEST(test_tick_while_idle_is_a_noop);
  RUN_TEST(test_negative_amplitude_still_produces_ok_result);
  RUN_TEST(test_restart_clears_previous_result);
  RUN_TEST(test_method_reported_correctly);
  RUN_TEST(test_relay_method_leaves_gain_result_zero);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
