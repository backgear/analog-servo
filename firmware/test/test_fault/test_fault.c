/*
 * Native unit tests for the latched hard-fault state machine (Fault.c).
 * Fault.c calls OutputForceSafe() on latch -- stubbed here (Output.c itself
 * isn't linked into the native build, see platformio.ini's build_src_filter).
 * Run: pio test -e native
 */
#include <unity.h>
#include "Fault.h"
#include "main.h"
#include "Encoder.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

static int sForceSafeCalls = 0;
void OutputForceSafe(void) { sForceSafeCalls++; }

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Ramp.c/Protocol.c (and everything they call
 * into) get linked into every test/test_* executable, this one included.
 * See platformio.ini and analog_quadrature's test suites for the same
 * pattern. */
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
  FaultInit();
  sForceSafeCalls = 0;
}
void tearDown(void) {}

/* ---- polarity mismatch ----------------------------------------------------- */
static void test_polarity_single_blip_does_not_latch(void) {
  FaultTick(10000, -5, 0, 0);   /* one sample cannot establish acceleration */
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_polarity_sustained_mismatch_latches(void) {
  for (int i = 0; i < 80; i++) FaultTick(10000, -i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(10000, -80, 0, 80);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_POLARITY, FaultGetReason());
  TEST_ASSERT_EQUAL_INT(1, sForceSafeCalls);
}

static void test_braking_both_directions_is_valid(void) {
  for (int i = 0; i < 300; i++) FaultTick(-10000, 500-i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultClear();
  for (int i = 0; i < 300; i++) FaultTick(10000, -500+i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_wrong_acceleration_even_with_matching_velocity_latches(void) {
  for (int i = 0; i < 81; i++) FaultTick(-10000, -500+i, 0, (uint32_t)i);
  TEST_ASSERT_EQUAL_INT(FAULT_POLARITY, FaultGetReason());
}

static void test_quantization_and_constant_velocity_are_inconclusive(void) {
  for (int i = 0; i < 500; i++) FaultTick(10000, -10+(i&1), 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_reversals_and_low_torque_reset_evidence(void) {
  for (int i = 0; i < 300; i++) {
    int torque = (i%60 == 59) ? 0 : ((i/60)&1 ? -10000 : 10000);
    FaultTick(torque, -i, 0, (uint32_t)i);
  }
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_clear_discards_acceleration_history(void) {
  for (int i = 0; i < 70; i++) FaultTick(10000, -i, 0, (uint32_t)i);
  FaultClear();
  for (int i = 0; i < 70; i++) FaultTick(10000, -i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_polarity_gated_below_threshold(void) {
  /* Both torque and velocity below the gating thresholds -- sign mismatch
   * should be ignored entirely (inert), not accumulate. */
  for (int i = 0; i < 200; i++) FaultTick(1, -1, 0, 0);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_polarity_consistent_sign_never_latches(void) {
  for (int i = 0; i < 500; i++) FaultTick(10000, 5, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

/* ---- polarity-check suppression (polarity-learn's raw, uncorrected probe) -- */
static void test_polarity_check_suppressed_by_default_is_off(void) {
  TEST_ASSERT_FALSE(FaultGetPolarityCheckSuppressed());
}

static void test_polarity_suppressed_never_latches_even_past_the_window(void) {
  FaultSetPolarityCheckSuppressed(1);
  /* Same sustained mismatch that test_polarity_sustained_mismatch_latches()
   * proves DOES latch when unsuppressed -- run it for 4x the window
   * (POLARITY_LEARN_DURATION_TICKS-ish) to be sure this isn't just a longer
   * window, but a real bypass. */
  for (int i = 0; i < 200; i++) FaultTick(10000, -i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_polarity_suppression_resumes_normal_detection_once_lifted(void) {
  FaultSetPolarityCheckSuppressed(1);
  for (int i = 0; i < 200; i++) FaultTick(10000, -i, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());   /* suppressed -- confirms setup, see test above */

  FaultSetPolarityCheckSuppressed(0);
  /* Suppression must discard the old acceleration evidence. */
  for (int i = 0; i < 80; i++) FaultTick(10000, -i, 0, (uint32_t)(200+i));
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(10000, -80, 0, 280);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_POLARITY, FaultGetReason());
}

static void test_polarity_suppression_does_not_affect_other_detectors(void) {
  /* Suppressing the polarity check specifically must not accidentally
   * disable the OTHER detectors FaultTick() also runs -- zero-encoder here
   * (same shape as test_zero_pulse_sustained_no_edges_latches()). */
  FaultSetPolarityCheckSuppressed(1);
  for (int i = 0; i < 199; i++) FaultTick(10000, 0, 0, 0);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(10000, 0, 0, 0);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_ZERO_ENCODER, FaultGetReason());
}

/* ---- zero encoder pulses --------------------------------------------------- */
static void test_zero_pulse_sustained_no_edges_latches(void) {
  for (int i = 0; i < 199; i++) FaultTick(10000, 0, 0, 0);   /* edges never advance */
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(10000, 0, 0, 0);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_ZERO_ENCODER, FaultGetReason());
}

static void test_zero_pulse_chatter_with_edges_does_not_latch(void) {
  /* Edges keep advancing even though net position (not modeled here -- the
   * detector only ever sees the monotonic edge count) stays flat -- this is
   * exactly the "vibrating against a hard limit" case the design doc calls
   * out as one that must NOT false-fault. */
  for (int i = 0; i < 300; i++) FaultTick(10000, 0, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_zero_pulse_gated_below_threshold(void) {
  for (int i = 0; i < 300; i++) FaultTick(1, 0, 0, 0);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

/* ---- endstop ---------------------------------------------------------------- */
static void test_endstop_latches_immediately(void) {
  FaultEndstopIsr();
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_ENDSTOP, FaultGetReason());
  TEST_ASSERT_EQUAL_INT(1, sForceSafeCalls);
}

/* ---- first-to-latch wins ---------------------------------------------------- */
static void test_first_to_latch_wins(void) {
  FaultEndstopIsr();
  TEST_ASSERT_EQUAL_INT(FAULT_ENDSTOP, FaultGetReason());
  for (int i = 0; i < 100; i++) FaultTick(10000, -i, 0, (uint32_t)i);   /* would otherwise latch polarity */
  TEST_ASSERT_EQUAL_INT(FAULT_ENDSTOP, FaultGetReason());   /* unchanged */
  TEST_ASSERT_EQUAL_INT(1, sForceSafeCalls);                /* not called again */
}

/* ---- clear ------------------------------------------------------------------ */
static void test_clear_resets_state(void) {
  FaultEndstopIsr();
  TEST_ASSERT_TRUE(FaultIsLatched());
  FaultClear();
  TEST_ASSERT_FALSE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_NONE, FaultGetReason());
}

/* ---- following error --------------------------------------------------------- */
static void test_following_error_soft_by_default(void) {
  FaultSetFollowingErrorConfig(100, 0);
  FaultTick(0, 0, 500, 0);
  TEST_ASSERT_TRUE(FaultGetFollowingErrorActive());
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_following_error_hard_fault_when_enabled(void) {
  FaultSetFollowingErrorConfig(100, 1);
  FaultTick(0, 0, 500, 0);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_FOLLOWING_ERROR, FaultGetReason());
}

static void test_following_error_disabled_when_limit_zero(void) {
  FaultSetFollowingErrorConfig(0, 1);
  FaultTick(0, 0, 999999, 0);
  TEST_ASSERT_FALSE(FaultGetFollowingErrorActive());
  TEST_ASSERT_FALSE(FaultIsLatched());
}

/* ---- sustained high commanded current (on by default, no config) ---------- */
static void test_sustained_current_below_threshold_never_latches(void) {
  /* 9% of full scale (32767/10 ~= 3276 is the 10% threshold) -- below it,
   * indefinitely, must never latch. */
  for (int i = 0; i < 5000; i++) FaultTick(3000, 0, 0, (uint32_t)i);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_sustained_current_brief_spike_does_not_latch(void) {
  /* Encoder edges advance every tick (unlike the zero-pulse tests above) --
   * this is isolating the sustained-current detector, not retriggering the
   * zero-pulse one, which would otherwise latch first at its own 200-tick
   * window given the same high torque command. */
  uint32_t edge = 0;
  for (int i = 0; i < 1999; i++) FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());   /* window not yet reached */
  FaultTick(1, 0, 0, edge++);            /* drops below threshold -- resets */
  for (int i = 0; i < 1999; i++) FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());
}

static void test_sustained_current_held_high_latches(void) {
  uint32_t edge = 0;
  for (int i = 0; i < 1999; i++) FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_SUSTAINED_CURRENT, FaultGetReason());
  TEST_ASSERT_EQUAL_INT(1, sForceSafeCalls);
}

static void test_sustained_current_held_high_negative_also_latches(void) {
  uint32_t edge = 0;
  for (int i = 0; i < 2000; i++) FaultTick(-32767, 0, 0, edge++);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_SUSTAINED_CURRENT, FaultGetReason());
}

/* ---- sustained-current config is now runtime-settable ---------------------
 * Each test below restores the compile-time defaults at the end so later
 * tests (and re-runs) aren't left depending on run order -- config
 * survives FaultInit()/FaultClear() by design (see Fault.h), so nothing
 * else resets it between tests. */
#define SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE   (32767 / 10)
#define SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS 2000

static void test_sustained_current_zero_min_torque_disables_it(void) {
  FaultSetSustainedCurrentConfig(0, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
  uint32_t edge = 0;
  for (int i = 0; i < 5000; i++) FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultSetSustainedCurrentConfig(SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
}

static void test_sustained_current_shorter_window_latches_sooner(void) {
  FaultSetSustainedCurrentConfig(SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE, 100);
  uint32_t edge = 0;
  for (int i = 0; i < 99; i++) FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultTick(32767, 0, 0, edge++);
  TEST_ASSERT_TRUE(FaultIsLatched());
  TEST_ASSERT_EQUAL_INT(FAULT_SUSTAINED_CURRENT, FaultGetReason());
  FaultSetSustainedCurrentConfig(SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
}

static void test_sustained_current_higher_threshold_tolerates_old_default(void) {
  /* 32767/3 is above SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE (10% of full scale)
   * but below the 50% threshold configured here, so it must NOT latch at
   * this higher threshold. */
  FaultSetSustainedCurrentConfig(32767 / 2, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
  uint32_t edge = 0;
  for (int i = 0; i < 5000; i++) FaultTick(32767 / 3, 0, 0, edge++);
  TEST_ASSERT_FALSE(FaultIsLatched());
  FaultSetSustainedCurrentConfig(SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
}

static void test_sustained_current_getters_reflect_configured_values(void) {
  FaultSetSustainedCurrentConfig(1234, 5678);
  TEST_ASSERT_EQUAL_INT32(1234, FaultGetSustainedCurrentMinTorque());
  TEST_ASSERT_EQUAL_INT32(5678, FaultGetSustainedCurrentWindowTicks());
  FaultSetSustainedCurrentConfig(SUSTAINED_CURRENT_DEFAULT_MIN_TORQUE, SUSTAINED_CURRENT_DEFAULT_WINDOW_TICKS);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_braking_both_directions_is_valid);
  RUN_TEST(test_wrong_acceleration_even_with_matching_velocity_latches);
  RUN_TEST(test_quantization_and_constant_velocity_are_inconclusive);
  RUN_TEST(test_reversals_and_low_torque_reset_evidence);
  RUN_TEST(test_clear_discards_acceleration_history);
  RUN_TEST(test_polarity_single_blip_does_not_latch);
  RUN_TEST(test_polarity_sustained_mismatch_latches);
  RUN_TEST(test_polarity_gated_below_threshold);
  RUN_TEST(test_polarity_consistent_sign_never_latches);
  RUN_TEST(test_polarity_check_suppressed_by_default_is_off);
  RUN_TEST(test_polarity_suppressed_never_latches_even_past_the_window);
  RUN_TEST(test_polarity_suppression_resumes_normal_detection_once_lifted);
  RUN_TEST(test_polarity_suppression_does_not_affect_other_detectors);
  RUN_TEST(test_zero_pulse_sustained_no_edges_latches);
  RUN_TEST(test_zero_pulse_chatter_with_edges_does_not_latch);
  RUN_TEST(test_zero_pulse_gated_below_threshold);
  RUN_TEST(test_endstop_latches_immediately);
  RUN_TEST(test_first_to_latch_wins);
  RUN_TEST(test_clear_resets_state);
  RUN_TEST(test_following_error_soft_by_default);
  RUN_TEST(test_following_error_hard_fault_when_enabled);
  RUN_TEST(test_following_error_disabled_when_limit_zero);
  RUN_TEST(test_sustained_current_below_threshold_never_latches);
  RUN_TEST(test_sustained_current_brief_spike_does_not_latch);
  RUN_TEST(test_sustained_current_held_high_latches);
  RUN_TEST(test_sustained_current_held_high_negative_also_latches);
  RUN_TEST(test_sustained_current_zero_min_torque_disables_it);
  RUN_TEST(test_sustained_current_shorter_window_latches_sooner);
  RUN_TEST(test_sustained_current_higher_threshold_tolerates_old_default);
  RUN_TEST(test_sustained_current_getters_reflect_configured_values);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
