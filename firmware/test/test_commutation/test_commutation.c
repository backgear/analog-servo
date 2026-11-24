/*
 * Native unit tests for electrical-angle math, Ia/Ib sine synthesis, and the
 * alignment-learning accumulator (Commutation.c). Pure fixed-point, no
 * HAL/mocks needed -- the only link dependency is Output.h's
 * TORQUE_FULL_SCALE constant (a #define, not a function).
 * Run: pio test -e native -f test_commutation
 */
#include <unity.h>
#include <stdlib.h>
#include "Commutation.h"
#include "Output.h"
#include "main.h"
#include "Encoder.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Control.c/Ramp.c/Fault.c/Signal.c/Protocol.c (and
 * everything they call into) get linked into every test/test_* executable,
 * this one included. See platformio.ini and test_polarity.c for the same
 * pattern. */
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
OutputTopology AppGetOutputTopology(void) { return OUTPUT_TOPOLOGY_ANALOG_TORQUE; }
int            AppSetOutputTopology(OutputTopology t) { (void)t; return 0; }
void    AppSetCommutationConfig(int32_t p, int32_t c) { (void)p; (void)c; }
void    AppCommutationLearn(void) {}
int     AppCommutationLearnBusy(void) { return 0; }
int     AppGetCommutationLearnFailed(void) { return 0; }
void    AppCommutationDetectPoles(int32_t encoderCpr) { (void)encoderCpr; }
int     AppCommutationDetectPolesBusy(void) { return 0; }
int     AppGetCommutationDetectPolesFailed(void) { return 0; }
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
int        AppGetSignalCycleStart(void) { return 0; }
int AppResetPosition(void) { return 0; }
void AppGetControlSample(AppControlSample *out) { *out = (AppControlSample){0}; }
int32_t AppGetLastVelocityActual(void) { return 0; }
int32_t AppGetLastTorqueCommand(void) { return 0; }
int32_t AppGetLastCommandedPosition(void) { return 0; }
int32_t AppGetLastCommandedVelocity(void) { return 0; }
int32_t AppGetLastElectricalAngleDeg(void) { return 0; }
int32_t AppGetLastHallSector(void) { return 0; }
int32_t EncoderGetPositionCounts(void) { return 0; }
int     EncoderZWasSeenSinceEnable(void) { return 0; }
void    UsartTransmit(const uint8_t *data, uint16_t len) { (void)data; (void)len; }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { (void)Buf; (void)Len; return USBD_OK; }
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff, uint16_t length) {
  (void)pdev; (void)channel; (void)pbuff; (void)length;
  return USBD_OK;
}
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) {
  (void)pdev; (void)channel;
  return USBD_OK;
}
void AppSetCommandVelocityClamp(int32_t c) { (void)c; }
void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }
void AppSetAccelerationFeedforward(int32_t c) { (void)c; }
void AppSetVelocityJerk(int32_t c) { (void)c; }
void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }

void setUp(void) {}
void tearDown(void) {}

static void test_hall_angle_is_60_degrees_per_sector(void) {
  TEST_ASSERT_EQUAL_INT32(0,   CommutationElectricalAngleFromHallDeg(0));
  TEST_ASSERT_EQUAL_INT32(60,  CommutationElectricalAngleFromHallDeg(1));
  TEST_ASSERT_EQUAL_INT32(300, CommutationElectricalAngleFromHallDeg(5));
}

static void test_hall_angle_wraps_out_of_range_sector(void) {
  TEST_ASSERT_EQUAL_INT32(60, CommutationElectricalAngleFromHallDeg(7));   /* 7 mod 6 == 1 */
  TEST_ASSERT_EQUAL_INT32(300, CommutationElectricalAngleFromHallDeg(-1)); /* wraps to sector 5 */
}

static void test_encoder_angle_at_zero_counts_is_zero_offset(void) {
  TEST_ASSERT_EQUAL_INT32(0, CommutationElectricalAngleFromEncoderDeg(0, 1000, 0));
}

static void test_encoder_angle_half_rev_is_180_degrees(void) {
  TEST_ASSERT_EQUAL_INT32(180, CommutationElectricalAngleFromEncoderDeg(500, 1000, 0));
}

static void test_encoder_angle_wraps_across_multiple_revolutions(void) {
  /* 1500 counts into a 1000-count elec rev is the same angle as 500. */
  TEST_ASSERT_EQUAL_INT32(180, CommutationElectricalAngleFromEncoderDeg(1500, 1000, 0));
}

static void test_encoder_angle_handles_negative_counts(void) {
  /* -250 counts out of 1000 is 3/4 of a revolution backward == 270 degrees. */
  TEST_ASSERT_EQUAL_INT32(270, CommutationElectricalAngleFromEncoderDeg(-250, 1000, 0));
}

static void test_encoder_angle_offset_is_subtracted_and_wrapped(void) {
  TEST_ASSERT_EQUAL_INT32(170, CommutationElectricalAngleFromEncoderDeg(500, 1000, 10));
  TEST_ASSERT_EQUAL_INT32(350, CommutationElectricalAngleFromEncoderDeg(0, 1000, 10));   /* 0-10 wraps to 350 */
}

static void test_encoder_angle_guards_nonpositive_counts_per_rev(void) {
  TEST_ASSERT_EQUAL_INT32(0, CommutationElectricalAngleFromEncoderDeg(500, 0, 0));
  TEST_ASSERT_EQUAL_INT32(0, CommutationElectricalAngleFromEncoderDeg(500, -1, 0));
}

static void test_phases_zero_torque_gives_zero_phases(void) {
  int32_t ia, ib;
  CommutationComputePhases(0, 45, &ia, &ib);
  TEST_ASSERT_EQUAL_INT32(0, ia);
  TEST_ASSERT_EQUAL_INT32(0, ib);
}

static void test_phases_at_zero_degrees_ia_is_zero_ib_is_negative(void) {
  /* sin(0)=0, sin(-120)=-sin(120) < 0 */
  int32_t ia, ib;
  CommutationComputePhases(TORQUE_FULL_SCALE, 0, &ia, &ib);
  TEST_ASSERT_INT32_WITHIN(2, 0, ia);
  TEST_ASSERT_TRUE(ib < 0);
}

static void test_phases_at_90_degrees_near_full_scale(void) {
  int32_t ia, ib;
  CommutationComputePhases(TORQUE_FULL_SCALE, 90, &ia, &ib);
  TEST_ASSERT_INT32_WITHIN(2, TORQUE_FULL_SCALE, ia);
}

static void test_phases_clamp_torque_to_full_scale(void) {
  int32_t ia1, ib1, ia2, ib2;
  CommutationComputePhases(TORQUE_FULL_SCALE * 2, 37, &ia1, &ib1);
  CommutationComputePhases(TORQUE_FULL_SCALE, 37, &ia2, &ib2);
  TEST_ASSERT_EQUAL_INT32(ia2, ia1);
  TEST_ASSERT_EQUAL_INT32(ib2, ib1);
}

static void test_phases_never_exceed_full_scale_across_all_angles(void) {
  for (int32_t deg = 0; deg < 360; deg++) {
    int32_t ia, ib;
    CommutationComputePhases(TORQUE_FULL_SCALE, deg, &ia, &ib);
    TEST_ASSERT_TRUE(ia >= -TORQUE_FULL_SCALE && ia <= TORQUE_FULL_SCALE);
    TEST_ASSERT_TRUE(ib >= -TORQUE_FULL_SCALE && ib <= TORQUE_FULL_SCALE);
  }
}

static void test_phases_120_degrees_apart(void) {
  /* ib(angle) should equal ia(angle - 120) by construction. */
  for (int32_t deg = 0; deg < 360; deg += 30) {
    int32_t ia_at_deg, ib_at_deg, ia_at_deg_minus_120, ib_unused;
    CommutationComputePhases(TORQUE_FULL_SCALE, deg, &ia_at_deg, &ib_at_deg);
    CommutationComputePhases(TORQUE_FULL_SCALE, deg - 120, &ia_at_deg_minus_120, &ib_unused);
    TEST_ASSERT_INT32_WITHIN(2, ia_at_deg_minus_120, ib_at_deg);
  }
}

/* ---- alignment-learning accumulator ---------------------------------------- */

static void test_learn_reports_failure_below_minimum_samples(void) {
  CommutationLearnReset();
  for (int i = 0; i < COMMUTATION_LEARN_MIN_TRANSITIONS - 1; i++) {
    CommutationLearnAccumulate(60 * (i % 6), i % 6);   /* perfectly aligned, zero true offset */
  }
  int32_t offset = -1;
  TEST_ASSERT_FALSE(CommutationLearnEvaluate(&offset));
}

static void test_learn_recovers_zero_offset_when_perfectly_aligned(void) {
  CommutationLearnReset();
  for (int i = 0; i < COMMUTATION_LEARN_MIN_TRANSITIONS + 6; i++) {
    int sector = i % 6;
    CommutationLearnAccumulate(sector * 60, sector);   /* rawAngle exactly matches sector's angle */
  }
  int32_t offset = -1;
  TEST_ASSERT_TRUE(CommutationLearnEvaluate(&offset));
  TEST_ASSERT_EQUAL_INT32(0, offset);
}

static void test_learn_recovers_a_known_nonzero_offset(void) {
  CommutationLearnReset();
  const int32_t trueOffset = 25;   /* motor's electrical zero sits 25 degrees from encoder's */
  for (int i = 0; i < COMMUTATION_LEARN_MIN_TRANSITIONS + 6; i++) {
    int sector = i % 6;
    int32_t rawAngle = (sector * 60 + trueOffset) % 360;
    CommutationLearnAccumulate(rawAngle, sector);
  }
  int32_t offset = -1;
  TEST_ASSERT_TRUE(CommutationLearnEvaluate(&offset));
  TEST_ASSERT_EQUAL_INT32(trueOffset, offset);
}

static void test_learn_handles_offset_near_the_0_360_wraparound(void) {
  CommutationLearnReset();
  const int32_t trueOffset = 355;   /* close to the wrap boundary */
  for (int i = 0; i < COMMUTATION_LEARN_MIN_TRANSITIONS + 6; i++) {
    int sector = i % 6;
    int32_t rawAngle = (sector * 60 + trueOffset) % 360;
    CommutationLearnAccumulate(rawAngle, sector);
  }
  int32_t offset = -1;
  TEST_ASSERT_TRUE(CommutationLearnEvaluate(&offset));
  TEST_ASSERT_EQUAL_INT32(trueOffset, offset);
}

static void test_learn_averages_out_small_noise(void) {
  CommutationLearnReset();
  const int32_t trueOffset = 10;
  /* Alternating +-3 degree noise around the true offset should still
   * average out close to it. */
  for (int i = 0; i < COMMUTATION_LEARN_MIN_TRANSITIONS + 6; i++) {
    int sector = i % 6;
    int32_t noise = (i % 2 == 0) ? 3 : -3;
    int32_t rawAngle = ((sector * 60 + trueOffset + noise) % 360 + 360) % 360;
    CommutationLearnAccumulate(rawAngle, sector);
  }
  int32_t offset = -1;
  TEST_ASSERT_TRUE(CommutationLearnEvaluate(&offset));
  TEST_ASSERT_INT32_WITHIN(1, trueOffset, offset);
}

/* ---- pole-count detection: by Hall-sector spacing -------------------------- */

static void test_pole_spacing_reports_failure_below_minimum_transitions(void) {
  CommutationPoleLearnBySpacingReset();
  int32_t pos = 0;
  for (int i = 0; i < COMMUTATION_POLE_LEARN_MIN_TRANSITIONS; i++) {   /* N calls -> N-1 transitions */
    CommutationPoleLearnBySpacingAccumulate(pos, i % 6);
    pos += 200;
  }
  int32_t countsPerElecRev, polePairs;
  TEST_ASSERT_FALSE(CommutationPoleLearnBySpacingEvaluate(4800, &countsPerElecRev, &polePairs));
}

static void test_pole_spacing_detects_4_pole_pairs_given_known_cpr(void) {
  CommutationPoleLearnBySpacingReset();
  int32_t pos = 0;
  /* 200 counts/transition x 6 transitions/elec-rev == 1200 counts/elec-rev;
   * a 4800-count/mech-rev encoder (known CPR) implies 4 pole pairs. */
  for (int i = 0; i < COMMUTATION_POLE_LEARN_MIN_TRANSITIONS + 6; i++) {
    CommutationPoleLearnBySpacingAccumulate(pos, i % 6);
    pos += 200;
  }
  int32_t countsPerElecRev = -1, polePairs = -1;
  TEST_ASSERT_TRUE(CommutationPoleLearnBySpacingEvaluate(4800, &countsPerElecRev, &polePairs));
  TEST_ASSERT_EQUAL_INT32(1200, countsPerElecRev);
  TEST_ASSERT_EQUAL_INT32(4, polePairs);
}

static void test_pole_spacing_requires_positive_encoder_cpr(void) {
  CommutationPoleLearnBySpacingReset();
  int32_t pos = 0;
  for (int i = 0; i < COMMUTATION_POLE_LEARN_MIN_TRANSITIONS + 6; i++) {
    CommutationPoleLearnBySpacingAccumulate(pos, i % 6);
    pos += 200;
  }
  int32_t countsPerElecRev, polePairs;
  TEST_ASSERT_FALSE(CommutationPoleLearnBySpacingEvaluate(0, &countsPerElecRev, &polePairs));
}

static void test_pole_spacing_degenerate_no_motion_fails(void) {
  CommutationPoleLearnBySpacingReset();
  for (int i = 0; i < COMMUTATION_POLE_LEARN_MIN_TRANSITIONS + 6; i++) {
    CommutationPoleLearnBySpacingAccumulate(0, i % 6);   /* encoder position never moves */
  }
  int32_t countsPerElecRev, polePairs;
  TEST_ASSERT_FALSE(CommutationPoleLearnBySpacingEvaluate(4800, &countsPerElecRev, &polePairs));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_hall_angle_is_60_degrees_per_sector);
  RUN_TEST(test_hall_angle_wraps_out_of_range_sector);
  RUN_TEST(test_encoder_angle_at_zero_counts_is_zero_offset);
  RUN_TEST(test_encoder_angle_half_rev_is_180_degrees);
  RUN_TEST(test_encoder_angle_wraps_across_multiple_revolutions);
  RUN_TEST(test_encoder_angle_handles_negative_counts);
  RUN_TEST(test_encoder_angle_offset_is_subtracted_and_wrapped);
  RUN_TEST(test_encoder_angle_guards_nonpositive_counts_per_rev);
  RUN_TEST(test_phases_zero_torque_gives_zero_phases);
  RUN_TEST(test_phases_at_zero_degrees_ia_is_zero_ib_is_negative);
  RUN_TEST(test_phases_at_90_degrees_near_full_scale);
  RUN_TEST(test_phases_clamp_torque_to_full_scale);
  RUN_TEST(test_phases_never_exceed_full_scale_across_all_angles);
  RUN_TEST(test_phases_120_degrees_apart);
  RUN_TEST(test_learn_reports_failure_below_minimum_samples);
  RUN_TEST(test_learn_recovers_zero_offset_when_perfectly_aligned);
  RUN_TEST(test_learn_recovers_a_known_nonzero_offset);
  RUN_TEST(test_learn_handles_offset_near_the_0_360_wraparound);
  RUN_TEST(test_learn_averages_out_small_noise);
  RUN_TEST(test_pole_spacing_reports_failure_below_minimum_transitions);
  RUN_TEST(test_pole_spacing_detects_4_pole_pairs_given_known_cpr);
  RUN_TEST(test_pole_spacing_requires_positive_encoder_cpr);
  RUN_TEST(test_pole_spacing_degenerate_no_motion_fails);
  return UNITY_END();
}
