/*
 * Native unit tests for Hall-sector decode/encode (Hall.c). Pure integer
 * logic, no HAL/mocks needed by this suite itself.
 * Run: pio test -e native -f test_hall
 */
#include <unity.h>
#include "Hall.h"
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

static void test_invalid_states_rejected(void) {
  TEST_ASSERT_EQUAL_INT(-1, HallDecode(0, 0, 0));
  TEST_ASSERT_EQUAL_INT(-1, HallDecode(1, 1, 1));
}

static void test_all_six_sectors_decode_in_range_and_distinct(void) {
  int seen[6] = {0, 0, 0, 0, 0, 0};
  for (int s = 0; s < 6; s++) {
    int a, b, c;
    HallEncode(s, &a, &b, &c);
    int decoded = HallDecode(a, b, c);
    TEST_ASSERT_TRUE(decoded >= 0 && decoded <= 5);
    TEST_ASSERT_EQUAL_INT(0, seen[decoded]);   /* no two sectors map to the same state */
    seen[decoded] = 1;
  }
}

static void test_decode_encode_roundtrip(void) {
  /* Every sector 0..5, decode(encode(s)) == s. */
  for (int s = 0; s < 6; s++) {
    int a, b, c;
    HallEncode(s, &a, &b, &c);
    TEST_ASSERT_EQUAL_INT(s, HallDecode(a, b, c));
  }
}

static void test_encode_wraps_out_of_range_sector(void) {
  int a1, b1, c1, a2, b2, c2;
  HallEncode(7, &a1, &b1, &c1);    /* 7 mod 6 == 1 */
  HallEncode(1, &a2, &b2, &c2);
  TEST_ASSERT_EQUAL_INT(a2, a1);
  TEST_ASSERT_EQUAL_INT(b2, b1);
  TEST_ASSERT_EQUAL_INT(c2, c1);

  int a3, b3, c3, a4, b4, c4;
  HallEncode(-1, &a3, &b3, &c3);   /* wraps to sector 5 */
  HallEncode(5, &a4, &b4, &c4);
  TEST_ASSERT_EQUAL_INT(a4, a3);
  TEST_ASSERT_EQUAL_INT(b4, b3);
  TEST_ASSERT_EQUAL_INT(c4, c3);
}

static void test_adjacent_sectors_differ_by_one_bit(void) {
  /* The whole point of a Gray-code Hall sequence: a real rotation never
   * skips straight from one valid state to a non-adjacent one without
   * passing through the states in between. */
  for (int s = 0; s < 6; s++) {
    int a1, b1, c1, a2, b2, c2;
    HallEncode(s, &a1, &b1, &c1);
    HallEncode(s + 1, &a2, &b2, &c2);
    int diffBits = (a1 != a2) + (b1 != b2) + (c1 != c2);
    TEST_ASSERT_EQUAL_INT(1, diffBits);
  }
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_invalid_states_rejected);
  RUN_TEST(test_all_six_sectors_decode_in_range_and_distinct);
  RUN_TEST(test_decode_encode_roundtrip);
  RUN_TEST(test_encode_wraps_out_of_range_sector);
  RUN_TEST(test_adjacent_sectors_differ_by_one_bit);
  return UNITY_END();
}
