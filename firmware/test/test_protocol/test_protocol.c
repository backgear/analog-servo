/*
 * Native unit tests for the analog-servo line protocol. Protocol.c/Monitor.c
 * only reach HAL-touching modules (main.c's App* coordinator, Encoder,
 * usart, the composite USB class driver) through pure-stdint function
 * signatures -- all stubbed here with small observable state. Fault.c IS
 * linked for real (build_src_filter includes it), so the fault.* /
 * polarity.* commands exercise the real state machine, not a stub.
 * Run: pio test -e native
 */
#include <unity.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "Protocol.h"
#include "main.h"
#include "Encoder.h"
#include "Fault.h"
#include "Monitor.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* ---- App* coordinator stubs (real impl is main.c, not linked here) ------- */
static ServoMode  sMode = SERVO_MODE_STEPDIR;
static int        sEnabled = 0;
static int32_t     sCommandedVelocity = 0;
static int32_t     sPositionCmd = 0;
static ConfigData  sConfig;
static int sConfigSaveCalls = 0, sConfigLoadCalls = 0, sConfigDefaultsCalls = 0;
static int sConfigSaveResult = 0, sConfigLoadResult = 0;
static int     sPolarityLearnCalls = 0;
static int     sPolarityLearnFailed = 0;
static int32_t sPolarityLastMovedCounts = 0;
static int32_t sLastVelocityActual = 0, sLastTorqueCommand = 0;
static int32_t sLastCommandedPosition = 0, sLastCommandedVelocity = 0;

ServoMode AppGetServoMode(void) { return sMode; }
int AppSetServoMode(ServoMode m) {
  sMode = m;
  return 0;
}
int       AppGetEnabled(void) { return sEnabled; }
void      AppSetEnabled(int e) { sEnabled = e ? 1 : 0; }
int32_t   AppGetCommandedVelocity(void) { return sCommandedVelocity; }
void      AppSetCommandedVelocity(int32_t v) { sCommandedVelocity = v; }
int32_t   AppGetPositionCmd(void) { return sPositionCmd; }
void      AppSetPositionCmd(int32_t t) { sPositionCmd = t; }
const ConfigData *AppGetConfig(void) { return &sConfig; }
static int               sFaultFreezeExists = 0;
static FaultFreezeFrame  sFaultFreeze;
int                     AppFaultFreezeExists(void) { return sFaultFreezeExists; }
const FaultFreezeFrame *AppGetFaultFreeze(void) { return &sFaultFreeze; }
void AppSetGains(int32_t p, int32_t kp, int32_t ki) {
  sConfig.posGainQ8 = p; sConfig.velGainKpQ8 = kp; sConfig.velGainKiQ8 = ki;
}
void AppSetTorqueClamp(int32_t mn, int32_t mx) { sConfig.torqueClampMin = mn; sConfig.torqueClampMax = mx; }
void AppSetVelocityClamp(int32_t c) { sConfig.velocityClamp = c; }
void AppSetRampAccel(int32_t a) { sConfig.rampAccelCountsPerTickPerSec = a; }
void AppSetZeroTrim(int32_t trim) { sConfig.zeroTrimCounts = trim; }
void AppSetAnalogVelocityConfig(int32_t scaleQ8, int32_t hysteresisCounts) {
  sConfig.analogVelocityScaleQ8 = scaleQ8;
  sConfig.analogVelocityHysteresisCounts = hysteresisCounts;
}
static int32_t sAnalogRawAdc = 0;
static int     sAnalogReversed = 0;
static int     sHwEnableAsserted = 1;   /* defaults to asserted -- most tests don't care */
int32_t AppGetAnalogRawAdc(void) { return sAnalogRawAdc; }
int     AppGetAnalogReversed(void) { return sAnalogReversed; }
int     AppGetHwEnableAsserted(void) { return sHwEnableAsserted; }
static uint32_t sDiagTickCycles = 0, sDiagMaxTickCycles = 0;
static int32_t  sDiagTickLoadPct = 0;
uint32_t AppGetLastTickCycles(void) { return sDiagTickCycles; }
uint32_t AppGetMaxTickCycles(void) { return sDiagMaxTickCycles; }
int32_t  AppGetLastTickLoadPct(void) { return sDiagTickLoadPct; }
void AppSetFollowingErrorConfig(int32_t limit, int hard) {
  sConfig.followingErrorLimitCounts = limit;
  sConfig.followingErrorHardFault = hard ? 1 : 0;
  FaultSetFollowingErrorConfig(limit, hard);
}
void AppSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks) {
  sConfig.sustainedCurrentMinTorque = minTorque;
  sConfig.sustainedCurrentWindowTicks = windowTicks;
  FaultSetSustainedCurrentConfig(minTorque, windowTicks);
}
int  AppConfigSave(void) { sConfigSaveCalls++; return sConfigSaveResult; }
int  AppConfigLoad(void) { sConfigLoadCalls++; return sConfigLoadResult; }
void AppConfigDefaults(void) { sConfigDefaultsCalls++; }
void AppPolarityLearn(void) { sPolarityLearnCalls++; }
int  AppPolarityLearnBusy(void) { return 0; }
int     AppGetPolarityLearnFailed(void) { return sPolarityLearnFailed; }
int32_t AppGetPolarityLearnMovedCounts(void) { return sPolarityLastMovedCounts; }

static int          sSignalActive = 0;
static SignalType   sSignalType = SIGNAL_NONE;
static SignalTarget sSignalTargetVal = SIGNAL_TARGET_TORQUE;
static int32_t     sSignalLastAmplitude = 0;
static uint32_t    sSignalLastHalfPeriod = 0, sSignalLastDuration = 0;
static int         sSignalStopCalls = 0;
void AppSignalStart(SignalType t, SignalTarget tg, int32_t amp, uint32_t hp, uint32_t dur) {
  sSignalActive = 1;
  sSignalType = t;
  sSignalTargetVal = tg;
  sSignalLastAmplitude = amp;
  sSignalLastHalfPeriod = hp;
  sSignalLastDuration = dur;
}
void       AppSignalStop(void) { sSignalStopCalls++; sSignalActive = 0; sSignalType = SIGNAL_NONE; }
int        AppGetSignalActive(void) { return sSignalActive; }
SignalType   AppGetSignalType(void) { return sSignalType; }
SignalTarget AppGetSignalTarget(void) { return sSignalTargetVal; }

static AppControlSample sControlSample;
void AppGetControlSample(AppControlSample *out) { *out = sControlSample; }
int32_t AppGetLastVelocityActual(void) { return sLastVelocityActual; }
int32_t AppGetLastTorqueCommand(void) { return sLastTorqueCommand; }
int32_t AppGetLastCommandedPosition(void) { return sLastCommandedPosition; }
int32_t AppGetLastCommandedVelocity(void) { return sLastCommandedVelocity; }
int        AppGetSignalCycleStart(void) { return 0; }

static int      sAutotuneStartRelayCalls = 0;
static int      sAutotuneStopCalls = 0, sAutotuneApplyCalls = 0;
static int32_t  sAutotuneLastAmplitude = 0;
static uint32_t sAutotuneLastMaxTicks = 0;
static int      sAutotuneActive = 0;
static AutotuneMethod sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
static AutotuneResult sAutotuneResult = AUTOTUNE_RESULT_NONE;
static int32_t  sAutotuneKpResult = 0, sAutotuneKiResult = 0, sAutotunePosPResult = 0, sAutotuneGainResult = 0;
static uint32_t sAutotunePeriodTicks = 0;
static uint32_t sAutotuneRelayProgressSeq = 0;
static uint32_t sAutotuneRelayProgressTicksElapsed = 0;
static uint32_t sAutotuneRelayProgressMaxTicks = 0;
static uint32_t sAutotuneRelayProgressSwitchCount = 0;
static uint32_t sAutotuneRelayProgressSwitchesNeeded = 0;
static int32_t  sAutotuneRelayProgressPeakVelocity = 0;

void AppAutotuneStartRelay(int32_t amplitude, uint32_t maxTicks) {
  sAutotuneStartRelayCalls++;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneLastAmplitude = amplitude;
  sAutotuneLastMaxTicks = maxTicks;
  sAutotuneActive = 1;
}
void    AppAutotuneStop(void) { sAutotuneStopCalls++; sAutotuneActive = 0; }
int     AppAutotuneBusy(void) { return sAutotuneActive; }
AutotuneMethod AppAutotuneGetMethod(void) { return sAutotuneMethod; }
AutotuneResult AppAutotuneGetResult(void) { return sAutotuneResult; }
int32_t  AppAutotuneGetKpResultQ8(void) { return sAutotuneKpResult; }
int32_t  AppAutotuneGetKiResultQ8(void) { return sAutotuneKiResult; }
int32_t  AppAutotuneGetPosPResultQ8(void) { return sAutotunePosPResult; }
uint32_t AppAutotuneGetMeasuredPeriodTicks(void) { return sAutotunePeriodTicks; }
int32_t  AppAutotuneGetMeasuredGainQ8(void) { return sAutotuneGainResult; }
void     AppAutotuneApply(void) { sAutotuneApplyCalls++; }
uint32_t AppAutotuneGetRelayProgressSeq(void) { return sAutotuneRelayProgressSeq; }
uint32_t AppAutotuneGetRelayProgressTicksElapsed(void) { return sAutotuneRelayProgressTicksElapsed; }
uint32_t AppAutotuneGetRelayProgressMaxTicks(void) { return sAutotuneRelayProgressMaxTicks; }
uint32_t AppAutotuneGetRelayProgressSwitchCount(void) { return sAutotuneRelayProgressSwitchCount; }
uint32_t AppAutotuneGetRelayProgressSwitchesNeeded(void) { return sAutotuneRelayProgressSwitchesNeeded; }
int32_t  AppAutotuneGetRelayProgressPeakVelocity(void) { return sAutotuneRelayProgressPeakVelocity; }

/* ---- Encoder.c stubs -------------------------------------------------------- */
static int32_t sPosition = 0;
static int     sZSeen = 0;
int32_t EncoderGetPositionCounts(void) { return sPosition; }
int     EncoderZWasSeenSinceEnable(void) { return sZSeen; }

/* ---- Output.c stub (Fault.c calls this on any latch; Output.c itself isn't
 * linked into the native build, see platformio.ini's build_src_filter) ----- */
void OutputForceSafe(void) {}

/* ---- TX capture (UsartTransmit / CDCComposite_Transmit_Ch0 both write
 * here for the currently-active instance -- only one is ever driven per
 * `run()` call). ----------------------------------------------------------- */
static char   cap[16384];
static size_t capLen;
static void CaptureAppend(const uint8_t *data, uint16_t len) {
  if (capLen + len < sizeof(cap)) { memcpy(cap + capLen, data, len); capLen += len; cap[capLen] = 0; }
}
void    UsartTransmit(const uint8_t *data, uint16_t len) { CaptureAppend(data, len); }
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) { CaptureAppend(Buf, Len); return USBD_OK; }
static void capReset(void) { capLen = 0; cap[0] = 0; }

/* ---- Composite USB class driver stubs, for Monitor.c's channel-1 path ---
 * SetTxBuffer captures what channel 1 was handed (schema replies, frames)
 * so tests can assert on it -- channel 0's own capture (`cap`, above) is
 * separate, since a single run() only ever drives one channel at a time
 * but a monitor-port test may want both (e.g. `monitor schema` over the
 * CLI vs. '?' in-band) visible independently. */
USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t  monCap[2048];
static uint16_t monCapLen;
static void monCapReset(void) { monCapLen = 0; monCap[0] = 0; }
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff, uint16_t length) {
  (void)pdev;
  if (channel == 1U && length <= sizeof(monCap)) { memcpy(monCap, pbuff, length); monCapLen = length; }
  return USBD_OK;
}
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) {
  (void)pdev; (void)channel;
  return USBD_OK;
}

static int sPositionResetCalls;
int AppResetPosition(void) {
  sPositionResetCalls++;
  if (sEnabled || sLastVelocityActual || sAutotuneActive || sSignalActive) return -2;
  sPosition = sPositionCmd = sCommandedVelocity = 0;
  return 0;
}
void setUp(void) {
  sPositionResetCalls = 0;
  sControlSample = (AppControlSample){0};
  sMode = SERVO_MODE_STEPDIR;
  sEnabled = 0;
  sCommandedVelocity = 0;
  sPositionCmd = 0;
  sFaultFreezeExists = 0;
  memset(&sFaultFreeze, 0, sizeof(sFaultFreeze));
  memset(&sConfig, 0, sizeof(sConfig));
  sConfig.posGainQ8 = 256;
  sConfig.velGainKpQ8 = 512;
  sConfig.velGainKiQ8 = 0;
  sConfig.velocityClamp = 10000;
  sConfig.torqueClampMin = -1000;
  sConfig.torqueClampMax = 1000;
  sConfig.rampAccelCountsPerTickPerSec = 1000;
  sAnalogRawAdc = 0;
  sAnalogReversed = 0;
  sHwEnableAsserted = 1;
  sConfigSaveCalls = sConfigLoadCalls = sConfigDefaultsCalls = 0;
  sConfigSaveResult = sConfigLoadResult = 0;
  sPolarityLearnCalls = 0;
  sPolarityLearnFailed = 0;
  sPolarityLastMovedCounts = 0;
  sSignalActive = 0;
  sSignalType = SIGNAL_NONE;
  sSignalTargetVal = SIGNAL_TARGET_TORQUE;
  sSignalLastAmplitude = 0;
  sSignalLastHalfPeriod = 0;
  sSignalLastDuration = 0;
  sSignalStopCalls = 0;
  sAutotuneStartRelayCalls = 0;
  sAutotuneStopCalls = sAutotuneApplyCalls = 0;
  sAutotuneLastAmplitude = 0;
  sAutotuneLastMaxTicks = 0;
  sAutotuneActive = 0;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneResult = AUTOTUNE_RESULT_NONE;
  sAutotuneKpResult = sAutotuneKiResult = sAutotunePosPResult = sAutotuneGainResult = 0;
  sAutotunePeriodTicks = 0;
  sAutotuneRelayProgressSeq = 0U;
  sAutotuneRelayProgressTicksElapsed = 0U;
  sAutotuneRelayProgressMaxTicks = 0U;
  sAutotuneRelayProgressSwitchCount = 0U;
  sAutotuneRelayProgressSwitchesNeeded = 0U;
  sAutotuneRelayProgressPeakVelocity = 0;
  sLastVelocityActual = sLastTorqueCommand = 0;
  sPosition = 0;
  sZSeen = 0;
  FaultInit();
  FaultSetFollowingErrorConfig(0, 0);
  capReset();
  monCapReset();
  hUsbDeviceFS.pClassData = NULL;
  MonitorInit();
  ProtocolInit();
}
void tearDown(void) {}

/* Feeds the line over the USB instance -- UART instance's ring buffer/parser
 * is identical code, exercised implicitly by symmetry; not duplicated here. */
static void run(const char *line) {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "%s\n", line);
  ProtocolRxFromIsrUsb((const uint8_t *)buf, (uint32_t)n);
  ProtocolPoll();
}

/* ---- tests ------------------------------------------------------------------ */
static void test_version(void) {
  run("version");
  TEST_ASSERT_NOT_NULL(strstr(cap, "version=0.1.0"));
}

static void test_help_lists_commands(void) {
  run("help");
  TEST_ASSERT_NOT_NULL(strstr(cap, "get <name>"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "config <save|load|defaults>"));
}

static void test_schema_describes_wire_types_units_and_choices(void) {
  run("schema");
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"schema_version\":2"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"commands\":{\"version\":"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"input.source\":{\"access\":\"rw\",\"type\":\"enum\""));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"choices\":[\"stepdir\",\"host\",\"analog\"]"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"position.raw\":{\"access\":\"ro\""));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"fractional_bits\":8,\"unit\":\"1/tick\""));
  TEST_ASSERT_NOT_NULL(strstr(cap, "\"unit\":\"torque_unit\",\"full_scale\":32767"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "}}\r\n\r\n"));
  TEST_ASSERT_EQUAL_INT32(0, sEnabled);
  TEST_ASSERT_EQUAL_INT32(0, sConfigSaveCalls);
}

static void test_sample_is_one_coherent_read_without_enabling_monitor(void) {
  sControlSample = (AppControlSample){ .tick=123, .position=-456, .velocity=7,
    .torque=-890, .followingError=12, .flags=13 };
  run("sample");
  TEST_ASSERT_EQUAL_STRING("sample=123,-456,7,-890,12,13\r\n\r\n", cap);
  TEST_ASSERT_EQUAL_INT32(0, sEnabled);
  TEST_ASSERT_EQUAL_INT32(0, MonitorGetEnabled());
}

static void test_position_reset_pulse_and_state_rejection(void) {
  sPosition = 1234; sPositionCmd = 2000; sCommandedVelocity = 10;
  run("position zero extra");
  TEST_ASSERT_EQUAL_INT32(0, sPositionResetCalls);
  TEST_ASSERT_EQUAL_INT32(1234, sPosition);
  capReset(); sEnabled = 1;
  run("position zero");
  TEST_ASSERT_NOT_NULL(strstr(cap, "requires disabled"));
  TEST_ASSERT_EQUAL_INT32(1234, sPosition);
  capReset(); sEnabled = 0; sLastVelocityActual = 100;
  run("position zero");
  TEST_ASSERT_NOT_NULL(strstr(cap, "requires disabled"));
  capReset(); sLastVelocityActual = 0;
  run("position zero");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT32(0, sPosition);
  TEST_ASSERT_EQUAL_INT32(0, sPositionCmd);
  TEST_ASSERT_EQUAL_INT32(0, sCommandedVelocity);
  capReset(); run("get position.raw_reset");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown variable"));
  TEST_ASSERT_EQUAL_INT32(0, sConfigSaveCalls);
}

static void test_get_known_var(void) {
  run("get position.kp");
  TEST_ASSERT_NOT_NULL(strstr(cap, "position.kp=256"));
}

static void test_get_unknown_var(void) {
  run("get bogus.var");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown variable"));
}

static void test_set_and_get_roundtrip(void) {
  run("set position.kp 512");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  capReset();
  run("get position.kp");
  TEST_ASSERT_NOT_NULL(strstr(cap, "position.kp=512"));
}

static void test_set_readonly_var_rejected(void) {
  run("set position.raw 100");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=read-only"));
}

static void test_set_invalid_value_rejected(void) {
  run("set position.kp not_a_number");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
}

/* A negative gain is positive feedback (error pushes the command the wrong way), not just a
 * bad tune -- reject outright rather than silently accepting it. */
static void test_set_negative_gains_rejected(void) {
  run("set position.kp -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
  capReset();
  run("set velocity.kp -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
  capReset();
  run("set velocity.ki -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
}

static void test_set_servo_mode(void) {
  run("set input.source host");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT(SERVO_MODE_VELOCITY, sMode);
  capReset();
  run("get input.source");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.source=host"));
}

static void test_set_servo_mode_invalid(void) {
  run("set input.source bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
}

static void test_enable_disable_commands(void) {
  run("enable");
  TEST_ASSERT_EQUAL_INT(1, sEnabled);
  TEST_ASSERT_NOT_NULL(strstr(cap, "axis.enabled=1"));
  capReset();
  run("disable");
  TEST_ASSERT_EQUAL_INT(0, sEnabled);
  TEST_ASSERT_NOT_NULL(strstr(cap, "axis.enabled=0"));
}

static void test_status_snapshot(void) {
  sPosition = 500;
  sLastVelocityActual = 12;
  sLastTorqueCommand = 345;
  run("status");
  TEST_ASSERT_NOT_NULL(strstr(cap, "position.raw=500"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "velocity.raw=12"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "torque.command=345"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "fault.latched=0"));
}

static void test_settings_dumps_every_var(void) {
  run("settings");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.source="));
  TEST_ASSERT_NOT_NULL(strstr(cap, "velocity.ki="));
  TEST_ASSERT_NOT_NULL(strstr(cap, "output.polarity="));
  TEST_ASSERT_NOT_NULL(strstr(cap, "encoder.index_seen="));
}

static void test_fault_reason_reported(void) {
  FaultEndstopIsr();
  run("get fault.latched");
  TEST_ASSERT_NOT_NULL(strstr(cap, "fault.latched=1"));
  capReset();
  run("get fault.reason");
  TEST_ASSERT_NOT_NULL(strstr(cap, "fault.reason=endstop"));
}

static void test_fault_clear(void) {
  FaultEndstopIsr();
  TEST_ASSERT_TRUE(FaultIsLatched());
  run("fault clear");
  TEST_ASSERT_FALSE(FaultIsLatched());
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=cleared"));
}

static void test_fault_missing_arg(void) {
  run("fault");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
}

static void test_fault_freeze_not_captured_yet(void) {
  run("fault freeze");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
}

static void test_fault_freeze_dumps_captured_snapshot(void) {
  sFaultFreezeExists = 1;
  sFaultFreeze.tick = 12345;
  sFaultFreeze.reason = FAULT_POLARITY;
  sFaultFreeze.position = -500;
  sFaultFreeze.velocity = -7;
  sFaultFreeze.torqueCommand = 10000;
  sFaultFreeze.commandedPosition = 300;
  sFaultFreeze.commandedVelocity = 50;
  sFaultFreeze.mode = SERVO_MODE_VELOCITY;
  sFaultFreeze.enabled = 1;
  sFaultFreeze.hwEnableAsserted = 1;
  sFaultFreeze.idleReduced = 0;
  sFaultFreeze.synthetic = 1;
  sFaultFreeze.cycleStart = 0;
  sFaultFreeze.posGainQ8 = 77;
  sFaultFreeze.velGainKpQ8 = 320;
  sFaultFreeze.velGainKiQ8 = 8;
  sFaultFreeze.velocityClamp = 3000;
  sFaultFreeze.torqueClampMin = -32767;
  sFaultFreeze.torqueClampMax = 32767;

  run("fault freeze");
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.tick=12345"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.reason=polarity"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.position.raw=-500"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.velocity.raw=-7"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.torque.command=10000"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.position.reference=300"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.trajectory.velocity=50"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.input.source=host"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.axis.enabled=1"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.axis.enable_input=1"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.idle_reduced=0"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.synthetic=1"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.cycle_start=0"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.position.kp=77"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.velocity.kp=320"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.velocity.ki=8"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.velocity.limit=3000"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.torque.min=-32767"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.torque.max=32767"));
}

static void test_fault_freeze_survives_fault_clear(void) {
  /* The whole point, same as OBD's freeze frame outliving a cleared DTC --
   * see main.h's FaultFreezeFrame comment. */
  sFaultFreezeExists = 1;
  sFaultFreeze.reason = FAULT_ZERO_ENCODER;
  FaultEndstopIsr();
  run("fault clear");
  TEST_ASSERT_FALSE(FaultIsLatched());

  capReset();
  run("fault freeze");
  TEST_ASSERT_NOT_NULL(strstr(cap, "freeze.reason=zero_encoder"));
}

static void test_monitor_missing_arg(void) {
  run("monitor");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
}

static void test_monitor_unknown_subcommand(void) {
  run("monitor bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown monitor subcommand"));
}

/* ---- Monitor.c's in-band channel-1 commands + self-describing schema ---- */
static void test_monitor_rx_enable_command(void) {
  TEST_ASSERT_EQUAL_INT32(0, MonitorGetEnabled());
  MonitorRxFromIsrUsb((const uint8_t *)"1", 1);
  MonitorPoll();
  TEST_ASSERT_EQUAL_INT32(1, MonitorGetEnabled());
}

static void test_monitor_rx_disable_command(void) {
  MonitorSetEnabled(1);
  MonitorRxFromIsrUsb((const uint8_t *)"0", 1);
  MonitorPoll();
  TEST_ASSERT_EQUAL_INT32(0, MonitorGetEnabled());
}

static void test_monitor_rx_unknown_byte_is_ignored(void) {
  MonitorRxFromIsrUsb((const uint8_t *)"z", 1);
  MonitorPoll();   /* must not crash */
  TEST_ASSERT_EQUAL_INT32(0, MonitorGetEnabled());
}

static void test_monitor_rx_many_bytes_dont_overrun_the_ring(void) {
  /* Ring holds 8, one slot always kept empty (full-vs-empty convention) --
   * 20 queued bytes, no polling in between, must not crash or corrupt
   * state; only the first few are retained, the rest dropped. */
  uint8_t bytes[20];
  memset(bytes, '0', sizeof(bytes));
  MonitorRxFromIsrUsb(bytes, sizeof(bytes));
  for (int i = 0; i < 10; i++) MonitorPoll();   /* drain whatever was kept */
  TEST_ASSERT_EQUAL_INT32(0, MonitorGetEnabled());
}

static void test_monitor_schema_json_describes_the_real_frame(void) {
  const char *json = MonitorGetSchemaJson();
  TEST_ASSERT_NOT_NULL(json);
  TEST_ASSERT_EQUAL_INT32('{', json[0]);
  /* frame_size/sync come from sizeof(MonitorFrame)/MONITOR_FRAME_SYNC, not
   * hand-typed -- these two literal checks lock in today's known-correct
   * values (28 bytes, sync 0xAA = 170) as a regression guard, same as the
   * rest of this file pins other known-good behavior. */
  TEST_ASSERT_NOT_NULL(strstr(json, "\"frame_size\":28"));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"sync\":170"));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"name\":\"seq\",\"offset\":1,\"size\":4,\"type\":\"u32\""));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"name\":\"torque_command\",\"offset\":13,\"size\":4,\"type\":\"i32\""));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"name\":\"crc8\",\"offset\":27,\"size\":1,\"type\":\"u8\""));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"cycle_start\":7"));
  /* Same name list Protocol.c's `fault.reason` ENUM_META quotes -- both
   * read FAULT_REASON_NAMES_JSON, so this also guards against the two
   * drifting apart (see Fault.h). */
  TEST_ASSERT_NOT_NULL(strstr(json, "\"fault_reasons\":[\"none\",\"polarity\",\"zero_encoder\","
                                      "\"endstop\",\"following_error\",\"sustained_current\"]"));
  TEST_ASSERT_NOT_NULL(strstr(json, "\"rx_commands\""));
  TEST_ASSERT_EQUAL_CHAR('\n', json[strlen(json) - 1]);   /* single trailing newline, no CRLF */
}

static void test_monitor_rx_schema_request_sends_it_over_channel1(void) {
  hUsbDeviceFS.pClassData = (void *)1;   /* "a host is attached" */
  MonitorRxFromIsrUsb((const uint8_t *)"?", 1);
  MonitorPoll();
  const char *json = MonitorGetSchemaJson();
  TEST_ASSERT_EQUAL_UINT16((uint16_t)strlen(json), monCapLen);
  TEST_ASSERT_EQUAL_INT32(0, memcmp(monCap, json, monCapLen));
}

static void test_monitor_cli_schema_matches_inband_schema(void) {
  /* `monitor schema` (channel 0) and '?' (channel 1, above) serve the same
   * MonitorGetSchemaJson() content -- a tool can't be fed a different
   * answer depending which port it asked on. The CLI dispatcher (line 804
   * above) unconditionally appends its own trailing "\r\n" after ANY
   * command handler returns (see cmdSchema's existing double-CRLF for the
   * same reason) -- channel 0's copy picks up that one extra line ending
   * on top of the schema's own single trailing '\n'; channel 1's in-band
   * reply (tested above) has no such wrapper and is the pure bytes. */
  char expected[1100];
  snprintf(expected, sizeof(expected), "%s\r\n", MonitorGetSchemaJson());
  capReset();
  run("monitor schema");
  TEST_ASSERT_EQUAL_STRING(expected, cap);
}

static void test_monitor_dump_with_no_history_is_an_error(void) {
  /* MonitorTick() is never called in this test (no control-tick ISR here),
   * so the history ring buffer is still empty -- MonitorDumpFramesRemaining()
   * is 0 right after MonitorInit(). */
  run("monitor dump");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=no history recorded yet"));
}

static void test_monitor_freeze_post_ticks_set_accepted(void) {
  /* Same native-env limitation as the dump test above: MonitorSetFreezePostTicks()
   * is a no-op stub here (always reports back 0 regardless of what was
   * set), so a real get-after-set roundtrip isn't checkable natively --
   * this only exercises Protocol.c's own parse-and-call-through plumbing,
   * not Monitor.c's real clamp behavior (MONITOR_FREEZE_POST_TICKS_MAX
   * isn't visible outside Monitor.c at all). */
  run("set monitor.post_trigger_ticks 50");
  TEST_ASSERT_NULL(strstr(cap, "error="));
}

static void test_monitor_freeze_post_ticks_rejects_negative(void) {
  run("set monitor.post_trigger_ticks -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid value"));
}

static void test_polarity_learn(void) {
  run("output learn_polarity");
  TEST_ASSERT_EQUAL_INT(1, sPolarityLearnCalls);
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=learning"));
}

static void test_polarity_learn_failed_telemetry(void) {
  sPolarityLearnFailed = 1;
  sPolarityLastMovedCounts = 2;   /* below the real PolarityLearnEvaluate threshold */
  run("get output.polarity_failed");
  TEST_ASSERT_NOT_NULL(strstr(cap, "output.polarity_failed=1"));
  capReset();
  run("get output.polarity_motion");
  TEST_ASSERT_NOT_NULL(strstr(cap, "output.polarity_motion=2"));
}

static void test_signal_impulse(void) {
  run("test impulse 5000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=running"));
  TEST_ASSERT_EQUAL_INT(SIGNAL_IMPULSE, sSignalType);
  TEST_ASSERT_EQUAL_INT32(5000, sSignalLastAmplitude);
}

static void test_signal_impulse_missing_arg(void) {
  run("test impulse");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
}

static void test_signal_square(void) {
  run("test square torque -2000 100 5000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=running"));
  TEST_ASSERT_EQUAL_INT(SIGNAL_SQUARE, sSignalType);
  TEST_ASSERT_EQUAL_INT(SIGNAL_TARGET_TORQUE, sSignalTargetVal);
  TEST_ASSERT_EQUAL_INT32(-2000, sSignalLastAmplitude);
  TEST_ASSERT_EQUAL_UINT32(100, sSignalLastHalfPeriod);
  TEST_ASSERT_EQUAL_UINT32(5000, sSignalLastDuration);
}

static void test_signal_square_position_target(void) {
  run("test square position 750 20");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=running"));
  TEST_ASSERT_EQUAL_INT(SIGNAL_TARGET_POSITION, sSignalTargetVal);
  TEST_ASSERT_EQUAL_INT32(750, sSignalLastAmplitude);
}

static void test_signal_square_invalid_target_rejected(void) {
  run("test square bogus 1000 50");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid target"));
}

static void test_signal_square_no_duration_defaults_unlimited(void) {
  run("test square torque 1000 50");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_UINT32(0, sSignalLastDuration);
}

static void test_signal_square_invalid_half_period_rejected(void) {
  run("test square torque 1000 0");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid half_period_ticks"));
}

static void test_signal_square_missing_args(void) {
  run("test square torque 1000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
}

static void test_signal_stop(void) {
  run("test impulse 100");
  run("test stop");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=stopped"));
  TEST_ASSERT_EQUAL_INT(1, sSignalStopCalls);
  TEST_ASSERT_FALSE(AppGetSignalActive());
}

static void test_signal_unknown_subcommand(void) {
  run("test bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown test subcommand"));
}

static void test_signal_telemetry_vars(void) {
  run("test impulse 100");
  capReset();
  run("get test.active");
  TEST_ASSERT_NOT_NULL(strstr(cap, "test.active=1"));
  capReset();
  run("get test.type");
  TEST_ASSERT_NOT_NULL(strstr(cap, "test.type=impulse"));
  capReset();
  run("get test.target");
  TEST_ASSERT_NOT_NULL(strstr(cap, "test.target=torque"));
}

static void test_signal_telemetry_target_position(void) {
  run("test square position 100 10");
  capReset();
  run("get test.target");
  TEST_ASSERT_NOT_NULL(strstr(cap, "test.target=position"));
}

static void test_autotune_start_relay(void) {
  sConfig.polarityLearned = 1;
  run("autotune start relay 5000 3000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=running"));
  TEST_ASSERT_EQUAL_INT(1, sAutotuneStartRelayCalls);
  TEST_ASSERT_EQUAL_INT32(5000, sAutotuneLastAmplitude);
  TEST_ASSERT_EQUAL_UINT32(3000, sAutotuneLastMaxTicks);
}

static void test_autotune_start_relay_no_max_ticks_defaults_zero(void) {
  sConfig.polarityLearned = 1;
  run("autotune start relay 2000");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_UINT32(0, sAutotuneLastMaxTicks);
}

static void test_autotune_start_relay_missing_arg(void) {
  sConfig.polarityLearned = 1;
  run("autotune start relay");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
  TEST_ASSERT_EQUAL_INT(0, sAutotuneStartRelayCalls);
}

static void test_autotune_start_relay_invalid_amplitude(void) {
  sConfig.polarityLearned = 1;
  run("autotune start relay bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid amplitude"));
}

static void test_autotune_start_relay_invalid_max_ticks(void) {
  sConfig.polarityLearned = 1;
  run("autotune start relay 2000 bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=invalid max_ticks"));
}

static void test_autotune_start_unknown_method(void) {
  sConfig.polarityLearned = 1;
  run("autotune start bogus 1000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=usage:"));
}

/* Not just a defensive check: relay's switching logic assumes +torque produces +velocity
 * and gets stuck holding one sign for the whole run, every amplitude, on a reversed-wiring
 * plant that's never had `output learn_polarity` run -- see Autotune.h's file comment. */
static void test_autotune_start_relay_rejected_before_polarity_learned(void) {
  TEST_ASSERT_EQUAL_INT(0, sConfig.polarityLearned);   /* setUp()'s default */
  run("autotune start relay 5000");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=run output learn_polarity before autotune start relay"));
  TEST_ASSERT_EQUAL_INT(0, sAutotuneStartRelayCalls);
}

static void test_autotune_relay_progress_prints_on_new_seq(void) {
  sAutotuneActive = 1;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneRelayProgressSeq = 1;
  sAutotuneRelayProgressTicksElapsed = 1500;
  sAutotuneRelayProgressMaxTicks = 20000;
  sAutotuneRelayProgressSwitchCount = 3;
  sAutotuneRelayProgressSwitchesNeeded = 8;
  sAutotuneRelayProgressPeakVelocity = 1;

  capReset();
  ProtocolPoll();
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.progress ticks=1500/20000 switches=3/8 peak_vel=1"));
}

static void test_autotune_relay_progress_not_reprinted_for_same_seq(void) {
  sAutotuneActive = 1;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneRelayProgressSeq = 1;

  capReset();
  ProtocolPoll();
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.progress"));

  capReset();
  ProtocolPoll();   /* same seq -- nothing new to print */
  TEST_ASSERT_NULL(strstr(cap, "autotune.progress"));
}

static void test_autotune_relay_progress_not_printed_when_idle(void) {
  sAutotuneActive = 0;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneRelayProgressSeq = 99;

  capReset();
  ProtocolPoll();
  TEST_ASSERT_NULL(strstr(cap, "autotune.progress"));
}

static void test_autotune_stop(void) {
  run("autotune start relay 2000");
  run("autotune stop");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=stopped"));
  TEST_ASSERT_EQUAL_INT(1, sAutotuneStopCalls);
}

static void test_autotune_apply_without_result_rejected(void) {
  sAutotuneResult = AUTOTUNE_RESULT_NONE;
  run("autotune apply");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=no successful autotune result to apply"));
  TEST_ASSERT_EQUAL_INT(0, sAutotuneApplyCalls);
}

static void test_autotune_apply_after_failed_result_rejected(void) {
  sAutotuneResult = AUTOTUNE_RESULT_NO_OSCILLATION;
  run("autotune apply");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT(0, sAutotuneApplyCalls);
}

static void test_autotune_apply_ok_result(void) {
  sAutotuneResult = AUTOTUNE_RESULT_OK;
  run("autotune apply");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=applied"));
  TEST_ASSERT_EQUAL_INT(1, sAutotuneApplyCalls);
}

static void test_autotune_unknown_subcommand(void) {
  run("autotune bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown autotune subcommand"));
}

static void test_autotune_telemetry_vars(void) {
  sAutotuneActive = 1;
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  sAutotuneResult = AUTOTUNE_RESULT_OK;
  sAutotuneKpResult = 300;
  sAutotuneKiResult = 40;
  sAutotunePosPResult = 50;
  sAutotunePeriodTicks = 120;
  sAutotuneGainResult = 13;

  capReset();
  run("get autotune.active");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.active=1"));
  capReset();
  run("get autotune.method");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.method=relay"));
  capReset();
  run("get autotune.result");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.result=ok"));
  capReset();
  run("get autotune.velocity_kp");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.velocity_kp=300"));
  capReset();
  run("get autotune.velocity_ki");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.velocity_ki=40"));
  capReset();
  run("get autotune.position_kp");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.position_kp=50"));
  capReset();
  run("get autotune.period_ticks");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.period_ticks=120"));
  capReset();
  run("get autotune.plant_gain");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.plant_gain=13"));
}

static void test_autotune_method_name_relay(void) {
  sAutotuneMethod = AUTOTUNE_METHOD_RELAY;
  capReset(); run("get autotune.method");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.method=relay"));
}

static void test_autotune_result_names(void) {
  sAutotuneResult = AUTOTUNE_RESULT_NONE;
  capReset(); run("get autotune.result");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.result=none"));

  sAutotuneResult = AUTOTUNE_RESULT_NO_OSCILLATION;
  capReset(); run("get autotune.result");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.result=no_oscillation"));

  sAutotuneResult = AUTOTUNE_RESULT_BAD_MEASUREMENT;
  capReset(); run("get autotune.result");
  TEST_ASSERT_NOT_NULL(strstr(cap, "autotune.result=bad_measurement"));
}

static void test_config_save_ok(void) {
  sConfigSaveResult = 0;
  run("config save");
  TEST_ASSERT_EQUAL_INT(1, sConfigSaveCalls);
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=saved"));
}

static void test_config_save_failure(void) {
  sConfigSaveResult = -1;
  run("config save");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=flash write failed"));
}

static void test_config_load_ok(void) {
  sConfigLoadResult = 0;
  run("config load");
  TEST_ASSERT_EQUAL_INT(1, sConfigLoadCalls);
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=loaded"));
  TEST_ASSERT_NULL(strstr(cap, "warning="));   /* enabled matches (both 0) -- no warning */
}

static void test_config_load_warns_when_enabled_disagrees(void) {
  /* AppConfigLoad() (real main.c) overwrites sConfig (including .enabled)
   * from flash but deliberately leaves live sEnabled untouched -- this
   * stub mirrors just that end state rather than re-implementing the real
   * function, since what's under test here is Protocol.c's warning logic,
   * not AppConfigLoad() itself (see docs/README.md's Headless
   * Mode C operation section). */
  sEnabled = 0;
  sConfig.enabled = 1;
  sConfigLoadResult = 0;
  run("config load");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=loaded"));
  TEST_ASSERT_NOT_NULL(strstr(cap, "warning=axis.enabled not changed by config load"));
}

static void test_config_load_no_warning_when_enabled_agrees(void) {
  sEnabled = 1;
  sConfig.enabled = 1;
  sConfigLoadResult = 0;
  run("config load");
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=loaded"));
  TEST_ASSERT_NULL(strstr(cap, "warning="));
}

static void test_config_load_no_record(void) {
  sConfigLoadResult = -1;
  run("config load");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=no valid config in flash"));
}

static void test_config_defaults(void) {
  run("config defaults");
  TEST_ASSERT_EQUAL_INT(1, sConfigDefaultsCalls);
  TEST_ASSERT_NOT_NULL(strstr(cap, "state=defaults"));
}

static void test_config_unknown_subcommand(void) {
  run("config bogus");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown config subcommand"));
}

static void test_following_error_settings(void) {
  run("set position.error_limit 250");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  run("set position.error_trip 1");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  capReset();
  run("get position.error_limit");
  TEST_ASSERT_NOT_NULL(strstr(cap, "position.error_limit=250"));
  capReset();
  run("get position.error_trip");
  TEST_ASSERT_NOT_NULL(strstr(cap, "position.error_trip=1"));
}

static void test_sustained_current_config_roundtrip(void) {
  run("set torque.sustained_threshold 5000");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  run("set torque.sustained_ticks 500");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  capReset();
  run("get torque.sustained_threshold");
  TEST_ASSERT_NOT_NULL(strstr(cap, "torque.sustained_threshold=5000"));
  capReset();
  run("get torque.sustained_ticks");
  TEST_ASSERT_NOT_NULL(strstr(cap, "torque.sustained_ticks=500"));
}

static void test_sustained_current_min_torque_zero_means_disabled(void) {
  run("set torque.sustained_threshold 0");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT32(0, sConfig.sustainedCurrentMinTorque);
}

static void test_sustained_current_config_rejects_invalid(void) {
  run("set torque.sustained_threshold -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
  capReset();
  run("set torque.sustained_ticks 0");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
  capReset();
  run("set torque.sustained_ticks -5");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
}

static void test_output_zero_trim_roundtrip(void) {
  run("get output.trim");
  TEST_ASSERT_NOT_NULL(strstr(cap, "output.trim=0"));   /* defaults to zero */
  capReset();
  run("set output.trim -12");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT32(-12, sConfig.zeroTrimCounts);
  capReset();
  run("get output.trim");
  TEST_ASSERT_NOT_NULL(strstr(cap, "output.trim=-12"));
}

static void test_diag_tick_vars_are_read_only_and_wired(void) {
  sDiagTickCycles = 123;
  sDiagMaxTickCycles = 456;
  sDiagTickLoadPct = 7;
  run("get diag.tick_cycles");
  TEST_ASSERT_NOT_NULL(strstr(cap, "diag.tick_cycles=123"));
  capReset();
  run("get diag.tick_cycles_max");
  TEST_ASSERT_NOT_NULL(strstr(cap, "diag.tick_cycles_max=456"));
  capReset();
  run("get diag.tick_load_pct");
  TEST_ASSERT_NOT_NULL(strstr(cap, "diag.tick_load_pct=7"));
  capReset();
  run("set diag.tick_cycles 1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=read-only"));
}

static void test_servo_mode_accepts_analog(void) {
  run("set input.source analog");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT(SERVO_MODE_ANALOG, sMode);
  capReset();
  run("get input.source");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.source=analog"));
}

static void test_analog_config_roundtrip(void) {
  run("get input.analog.scale");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.analog.scale=0"));   /* zeroed test fixture, not firmware default */
  capReset();
  run("set input.analog.scale 32");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT32(32, sConfig.analogVelocityScaleQ8);
  capReset();
  run("set input.analog.hysteresis 16");
  TEST_ASSERT_NULL(strstr(cap, "error="));
  TEST_ASSERT_EQUAL_INT32(16, sConfig.analogVelocityHysteresisCounts);
  capReset();
  run("get input.analog.scale");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.analog.scale=32"));
  capReset();
  run("get input.analog.hysteresis");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.analog.hysteresis=16"));
}

static void test_analog_hysteresis_rejects_negative(void) {
  run("set input.analog.hysteresis -1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error="));
}

static void test_analog_diagnostics_are_read_only(void) {
  sAnalogRawAdc = 1234;
  sAnalogReversed = 1;
  run("get input.analog.raw");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.analog.raw=1234"));
  capReset();
  run("get input.analog.reverse");
  TEST_ASSERT_NOT_NULL(strstr(cap, "input.analog.reverse=1"));
  capReset();
  run("set input.analog.raw 1");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=read-only"));
}

static void test_enable_pin_asserted_reads_live_state(void) {
  sHwEnableAsserted = 0;
  run("get axis.enable_input");
  TEST_ASSERT_NOT_NULL(strstr(cap, "axis.enable_input=0"));
  capReset();
  run("status");
  TEST_ASSERT_NOT_NULL(strstr(cap, "axis.enable_input=0"));
}

static void test_unknown_command(void) {
  run("frobnicate");
  TEST_ASSERT_NOT_NULL(strstr(cap, "error=unknown command"));
}

static void test_command_velocity_limit_roundtrip_and_validation(void) {
  run("set trajectory.velocity_limit 40");
  TEST_ASSERT_EQUAL_INT32(40, sConfig.commandVelocityClamp);
  run("get trajectory.velocity_limit");
  TEST_ASSERT_NOT_NULL(strstr(cap, "trajectory.velocity_limit=40"));
  run("set trajectory.velocity_limit -1");
  TEST_ASSERT_EQUAL_INT32(40, sConfig.commandVelocityClamp);
  run("set trajectory.velocity_limit 0");
  TEST_ASSERT_EQUAL_INT32(0, sConfig.commandVelocityClamp);
}

static void test_control_options_validate_and_roundtrip(void) {
  run("set velocity.filter_shift 1");
  TEST_ASSERT_EQUAL_INT32(1,sConfig.velocityFilterShift);
  run("set velocity.filter_shift 5");
  TEST_ASSERT_EQUAL_INT32(1,sConfig.velocityFilterShift);
  run("set torque.accel_feedforward 1536000");
  TEST_ASSERT_EQUAL_INT32(1536000,sConfig.accelerationFeedforwardQ8);
  run("set torque.accel_feedforward -1");
  TEST_ASSERT_EQUAL_INT32(1536000,sConfig.accelerationFeedforwardQ8);
  run("set trajectory.jerk_limit 120000");
  TEST_ASSERT_EQUAL_INT32(120000,sConfig.velocityJerk);
  run("set trajectory.jerk_limit -1");
  TEST_ASSERT_EQUAL_INT32(120000,sConfig.velocityJerk);
  run("set velocity.deadband 1");
  TEST_ASSERT_EQUAL_INT32(1,sConfig.velocityErrorDeadbandCounts);
  capReset();
  run("get velocity.deadband");
  TEST_ASSERT_NOT_NULL(strstr(cap, "velocity.deadband=1"));
  run("set velocity.deadband -1");
  TEST_ASSERT_EQUAL_INT32(1,sConfig.velocityErrorDeadbandCounts);
}

static void test_point_move_feedforward_scale_is_bounded(void) {
  run("set trajectory.move_feedforward 128");
  TEST_ASSERT_EQUAL_INT32(128,sConfig.positionMoveFeedforwardQ8);
  run("set trajectory.move_feedforward 257");
  TEST_ASSERT_EQUAL_INT32(128,sConfig.positionMoveFeedforwardQ8);
  run("set trajectory.move_feedforward -1");
  TEST_ASSERT_EQUAL_INT32(128,sConfig.positionMoveFeedforwardQ8);
  run("get trajectory.move_feedforward");
  TEST_ASSERT_NOT_NULL(strstr(cap,"trajectory.move_feedforward=128"));
}

static void test_control_diagnostics_are_read_only_snapshot_values(void) {
  sLastVelocityActual = 30;
  sControlSample = (AppControlSample){ .positionReference=1234,
      .velocityReference=20, .velocityFiltered=12, .velocityError=8,
      .velocityControlError=0, .flags=8 };
  run("get velocity.raw");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.raw=30"));
  capReset(); run("get velocity.filtered");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.filtered=12"));
  capReset(); run("get velocity.reference");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.reference=20"));
  capReset(); run("get velocity.error");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.error=8"));
  capReset(); run("get velocity.control_error");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.control_error=0"));
  capReset(); run("get position.reference");
  TEST_ASSERT_NOT_NULL(strstr(cap,"position.reference=1234"));
  capReset(); run("get velocity.valid");
  TEST_ASSERT_NOT_NULL(strstr(cap,"velocity.valid=1"));
  capReset(); run("set velocity.filtered 77");
  TEST_ASSERT_NOT_NULL(strstr(cap,"error=read-only"));
  sControlSample.flags=0;
  capReset(); run("get position.valid");
  TEST_ASSERT_NOT_NULL(strstr(cap,"position.valid=0"));
}

static void test_schema_uses_only_canonical_names_and_actions(void) {
  run("schema");
  TEST_ASSERT_NULL(strstr(cap,"servo."));
  TEST_ASSERT_NULL(strstr(cap,"gains."));
  TEST_ASSERT_NOT_NULL(strstr(cap,"\"velocity.filtered\":{\"access\":\"ro\""));
  TEST_ASSERT_NOT_NULL(strstr(cap,"zero -- reset coordinates"));
  TEST_ASSERT_NOT_NULL(strstr(cap,"learn_polarity"));
  capReset(); run("set servo.enabled 1");
  TEST_ASSERT_NOT_NULL(strstr(cap,"error=unknown variable"));
  TEST_ASSERT_EQUAL_INT(0,sEnabled);
  capReset(); run("set input.source velocity");
  TEST_ASSERT_NOT_NULL(strstr(cap,"error=invalid value"));
  capReset(); run("set input.source host");
  TEST_ASSERT_EQUAL_INT(SERVO_MODE_VELOCITY,sMode);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_control_diagnostics_are_read_only_snapshot_values);
  RUN_TEST(test_schema_uses_only_canonical_names_and_actions);
  RUN_TEST(test_point_move_feedforward_scale_is_bounded);
  RUN_TEST(test_control_options_validate_and_roundtrip);
  RUN_TEST(test_command_velocity_limit_roundtrip_and_validation);
  RUN_TEST(test_version);
  RUN_TEST(test_help_lists_commands);
  RUN_TEST(test_position_reset_pulse_and_state_rejection);
  RUN_TEST(test_sample_is_one_coherent_read_without_enabling_monitor);
  RUN_TEST(test_schema_describes_wire_types_units_and_choices);
  RUN_TEST(test_get_known_var);
  RUN_TEST(test_get_unknown_var);
  RUN_TEST(test_set_and_get_roundtrip);
  RUN_TEST(test_set_readonly_var_rejected);
  RUN_TEST(test_set_invalid_value_rejected);
  RUN_TEST(test_set_negative_gains_rejected);
  RUN_TEST(test_set_servo_mode);
  RUN_TEST(test_set_servo_mode_invalid);
  RUN_TEST(test_enable_disable_commands);
  RUN_TEST(test_status_snapshot);
  RUN_TEST(test_settings_dumps_every_var);
  RUN_TEST(test_fault_reason_reported);
  RUN_TEST(test_fault_clear);
  RUN_TEST(test_fault_missing_arg);
  RUN_TEST(test_fault_freeze_not_captured_yet);
  RUN_TEST(test_fault_freeze_dumps_captured_snapshot);
  RUN_TEST(test_fault_freeze_survives_fault_clear);
  RUN_TEST(test_monitor_missing_arg);
  RUN_TEST(test_monitor_unknown_subcommand);
  RUN_TEST(test_monitor_rx_enable_command);
  RUN_TEST(test_monitor_rx_disable_command);
  RUN_TEST(test_monitor_rx_unknown_byte_is_ignored);
  RUN_TEST(test_monitor_rx_many_bytes_dont_overrun_the_ring);
  RUN_TEST(test_monitor_schema_json_describes_the_real_frame);
  RUN_TEST(test_monitor_rx_schema_request_sends_it_over_channel1);
  RUN_TEST(test_monitor_cli_schema_matches_inband_schema);
  RUN_TEST(test_monitor_dump_with_no_history_is_an_error);
  RUN_TEST(test_monitor_freeze_post_ticks_set_accepted);
  RUN_TEST(test_monitor_freeze_post_ticks_rejects_negative);
  RUN_TEST(test_polarity_learn);
  RUN_TEST(test_polarity_learn_failed_telemetry);
  RUN_TEST(test_signal_impulse);
  RUN_TEST(test_signal_impulse_missing_arg);
  RUN_TEST(test_signal_square);
  RUN_TEST(test_signal_square_position_target);
  RUN_TEST(test_signal_square_invalid_target_rejected);
  RUN_TEST(test_signal_square_no_duration_defaults_unlimited);
  RUN_TEST(test_signal_square_invalid_half_period_rejected);
  RUN_TEST(test_signal_square_missing_args);
  RUN_TEST(test_signal_stop);
  RUN_TEST(test_signal_unknown_subcommand);
  RUN_TEST(test_signal_telemetry_vars);
  RUN_TEST(test_signal_telemetry_target_position);
  RUN_TEST(test_autotune_start_relay);
  RUN_TEST(test_autotune_start_relay_rejected_before_polarity_learned);
  RUN_TEST(test_autotune_start_relay_no_max_ticks_defaults_zero);
  RUN_TEST(test_autotune_start_relay_missing_arg);
  RUN_TEST(test_autotune_start_relay_invalid_amplitude);
  RUN_TEST(test_autotune_start_relay_invalid_max_ticks);
  RUN_TEST(test_autotune_start_unknown_method);
  RUN_TEST(test_autotune_relay_progress_prints_on_new_seq);
  RUN_TEST(test_autotune_relay_progress_not_reprinted_for_same_seq);
  RUN_TEST(test_autotune_relay_progress_not_printed_when_idle);
  RUN_TEST(test_autotune_stop);
  RUN_TEST(test_autotune_apply_without_result_rejected);
  RUN_TEST(test_autotune_apply_after_failed_result_rejected);
  RUN_TEST(test_autotune_apply_ok_result);
  RUN_TEST(test_autotune_unknown_subcommand);
  RUN_TEST(test_autotune_telemetry_vars);
  RUN_TEST(test_autotune_method_name_relay);
  RUN_TEST(test_autotune_result_names);
  RUN_TEST(test_config_save_ok);
  RUN_TEST(test_config_save_failure);
  RUN_TEST(test_config_load_ok);
  RUN_TEST(test_config_load_warns_when_enabled_disagrees);
  RUN_TEST(test_config_load_no_warning_when_enabled_agrees);
  RUN_TEST(test_config_load_no_record);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_config_unknown_subcommand);
  RUN_TEST(test_following_error_settings);
  RUN_TEST(test_sustained_current_config_roundtrip);
  RUN_TEST(test_sustained_current_min_torque_zero_means_disabled);
  RUN_TEST(test_sustained_current_config_rejects_invalid);
  RUN_TEST(test_output_zero_trim_roundtrip);
  RUN_TEST(test_diag_tick_vars_are_read_only_and_wired);
  RUN_TEST(test_servo_mode_accepts_analog);
  RUN_TEST(test_analog_config_roundtrip);
  RUN_TEST(test_analog_hysteresis_rejects_negative);
  RUN_TEST(test_analog_diagnostics_are_read_only);
  RUN_TEST(test_enable_pin_asserted_reads_live_state);
  RUN_TEST(test_unknown_command);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { sConfig.commandVelocityClamp = c; }

void AppSetVelocityFilterShift(int32_t c) { sConfig.velocityFilterShift = c; }
void AppSetVelocityErrorDeadband(int32_t c) { sConfig.velocityErrorDeadbandCounts = c; }

void AppSetAccelerationFeedforward(int32_t c) { sConfig.accelerationFeedforwardQ8 = c; }

void AppSetVelocityJerk(int32_t c) { sConfig.velocityJerk = c; }

void AppSetPositionMoveFeedforward(int32_t c) { sConfig.positionMoveFeedforwardQ8 = c; }
