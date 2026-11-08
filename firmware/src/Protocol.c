/**
 * analog-servo line protocol, transport-agnostic over USB CDC-ACM (channel
 * 0 of the composite device) + USART2. Framing/registry shape: an
 * accessor-function-based registry, since telemetry spans several modules
 * -- generalized to TWO instances (one per transport), each with its own
 * ring buffer/line-assembly state, sharing one registry table and one
 * dispatcher via a small transport vtable. No checksum on either
 * transport (USB CDC is a reliable framed transport; USART2 here is plain
 * point-to-point, not RS485).
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "Protocol.h"
#include "main.h"
#include "Encoder.h"
#include "Fault.h"
#include "Monitor.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"

#define FW_VERSION "0.1.0"
#define LINE_MAX 128U
#define RX_RING_SIZE 256U

/* ---- transport-agnostic instance ------------------------------------------ */
typedef void (*tx_fn)(const uint8_t *data, uint16_t len);

typedef struct {
  volatile uint8_t  ring[RX_RING_SIZE];
  volatile uint32_t head;   /* written by the ISR */
  volatile uint32_t tail;   /* read by ProtocolPoll */
  char     line[LINE_MAX + 1];
  uint16_t len;
  char     pendEol;
  tx_fn    tx;
} ProtocolInstance;

static void TxUsb(const uint8_t *data, uint16_t len) {
  while (CDCComposite_Transmit_Ch0((uint8_t *)data, len) == USBD_BUSY) { }
}
static void TxUart(const uint8_t *data, uint16_t len) { UsartTransmit(data, len); }

static ProtocolInstance sUsbInst  = { .tx = TxUsb };
static ProtocolInstance sUartInst = { .tx = TxUart };

/* Set right before Dispatch() runs, for the duration of one line's response
 * -- every response helper below (txStr/respKV/...) writes to whichever
 * instance issued the command that's currently being answered. Safe with no
 * locking: ProtocolPoll (the only place this is read/written) runs only from
 * the main superloop, never preempted by itself. */
static ProtocolInstance *sActive;

static int RingPop(ProtocolInstance *inst, uint8_t *out) {
  if (inst->tail == inst->head) return 0;
  *out = inst->ring[inst->tail];
  inst->tail = (inst->tail + 1U) % RX_RING_SIZE;
  return 1;
}

static void RingPush(ProtocolInstance *inst, const uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint32_t next = (inst->head + 1U) % RX_RING_SIZE;
    if (next == inst->tail) break;   /* full: drop rather than overwrite unread data */
    inst->ring[inst->head] = data[i];
    inst->head = next;
  }
}

void ProtocolRxFromIsrUsb(const uint8_t *data, uint32_t len)  { RingPush(&sUsbInst, data, len); }
void ProtocolRxFromIsrUart(const uint8_t *data, uint32_t len) { RingPush(&sUartInst, data, len); }

/* ---- TX helpers (write to sActive) ----------------------------------------- */
static void txStr(const char *s) {
  uint16_t len = (uint16_t)strlen(s);
  if (len == 0U) return;
  sActive->tx((const uint8_t *)s, len);
}
static void respKV(const char *key, const char *val) { txStr(key); txStr("="); txStr(val); txStr("\r\n"); }
static void respKVi(const char *key, int32_t val) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%ld", (long)val);
  respKV(key, buf);
}
static void respError(const char *reason) { txStr("error="); txStr(reason); txStr("\r\n"); }
static void respEnd(void) { txStr("\r\n"); }
static void respHelpLine(const char *name, const char *help) { txStr(name); txStr(" "); txStr(help); txStr("\r\n"); }

/* ---- variable registry: accessor-function-based --------------------------- */
typedef void (*var_get_fn)(char *buf, size_t bufSize);
typedef int  (*var_set_fn)(const char *valStr);   /* 0 ok, -1 invalid */

typedef struct {
  const char *name;
  var_get_fn  get;
  var_set_fn  set;   /* NULL == read-only */
  const char *metadata; /* JSON fields: wire type, units, scaling, choices */
} var_entry_t;

static int ParseI32(const char *s, int32_t *out) {
  char *end;
  long val = strtol(s, &end, 10);
  if (*end != '\0') return -1;
  *out = (int32_t)val;
  return 0;
}
static int ParseBool(const char *s, int *out) {
  if (!strcmp(s, "1") || !strcmp(s, "true")) { *out = 1; return 0; }
  if (!strcmp(s, "0") || !strcmp(s, "false")) { *out = 0; return 0; }
  return -1;
}
static int ParseU32(const char *s, uint32_t *out) {
  int32_t val;
  if (ParseI32(s, &val) != 0 || val < 0) return -1;
  *out = (uint32_t)val;
  return 0;
}

static const char *ServoModeName(ServoMode m) {
  switch (m) {
    case SERVO_MODE_VELOCITY: return "host";
    case SERVO_MODE_ANALOG:   return "analog";
    default:                  return "stepdir";
  }
}
static const char *FaultReasonName(FaultReason r) {
  switch (r) {
    case FAULT_POLARITY:        return "polarity";
    case FAULT_ZERO_ENCODER:    return "zero_encoder";
    case FAULT_ENDSTOP:         return "endstop";
    case FAULT_FOLLOWING_ERROR: return "following_error";
    case FAULT_SUSTAINED_CURRENT: return "sustained_current";
    default:                    return "none";
  }
}
static const char *SignalTypeName(SignalType t) {
  switch (t) {
    case SIGNAL_IMPULSE: return "impulse";
    case SIGNAL_SQUARE:  return "square";
    default:              return "none";
  }
}
static const char *SignalTargetName(SignalTarget t) {
  return (t == SIGNAL_TARGET_POSITION) ? "position" : "torque";
}
static const char *AutotuneResultName(AutotuneResult r) {
  switch (r) {
    case AUTOTUNE_RESULT_OK:               return "ok";
    case AUTOTUNE_RESULT_NO_OSCILLATION:   return "no_oscillation";
    case AUTOTUNE_RESULT_BAD_MEASUREMENT:  return "bad_measurement";
    default:                               return "none";
  }
}
static const char *AutotuneMethodName(AutotuneMethod m) {
  (void)m;   /* only AUTOTUNE_METHOD_RELAY exists today -- see Autotune.h */
  return "relay";
}

static void getServoMode(char *b, size_t n) { snprintf(b, n, "%s", ServoModeName(AppGetServoMode())); }
static int  setServoMode(const char *v) {
  if (!strcmp(v, "stepdir"))  return AppSetServoMode(SERVO_MODE_STEPDIR);
  if (!strcmp(v, "host")) return AppSetServoMode(SERVO_MODE_VELOCITY);
  if (!strcmp(v, "analog"))   return AppSetServoMode(SERVO_MODE_ANALOG);
  return -1;
}
static void getServoEnabled(char *b, size_t n) { snprintf(b, n, "%d", AppGetEnabled()); }
static int  setServoEnabled(const char *v) {
  int en;
  if (ParseBool(v, &en) != 0) return -1;
  AppSetEnabled(en);
  return 0;
}
static void getServoVelocityCmd(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetCommandedVelocity()); }
static int  setServoVelocityCmd(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetCommandedVelocity(val);
  return 0;
}
static void getServoPositionCmd(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetPositionCmd()); }
static int  setServoPositionCmd(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetPositionCmd(val);
  return 0;
}
static void getServoPosition(char *b, size_t n) { snprintf(b, n, "%ld", (long)EncoderGetPositionCounts()); }
static void getServoVelocity(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetLastVelocityActual()); }
static void getFollowingError(char *b, size_t n) {
  AppControlSample sample;
  AppGetControlSample(&sample);
  snprintf(b, n, "%ld", (long)sample.followingError);
}
/* These signals all come from the completed control-tick snapshot. */
#define SAMPLE_GETTER(name, member) \
static void name(char *b, size_t n) { \
  AppControlSample sample; AppGetControlSample(&sample); \
  snprintf(b, n, "%ld", (long)sample.member); \
}
SAMPLE_GETTER(getPositionReference, positionReference)
SAMPLE_GETTER(getVelocityReference, velocityReference)
SAMPLE_GETTER(getVelocityFiltered, velocityFiltered)
SAMPLE_GETTER(getVelocityError, velocityError)
SAMPLE_GETTER(getVelocityControlError, velocityControlError)
static void getControlValid(char *b, size_t n) {
  AppControlSample sample; AppGetControlSample(&sample);
  snprintf(b, n, "%u", (sample.flags & 8U) != 0);
}

static void getServoTorqueCmd(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetLastTorqueCommand()); }

static void getGainsPosP(char *b, size_t n)   { snprintf(b, n, "%ld", (long)AppGetConfig()->posGainQ8); }
static int  setGainsPosP(const char *v) {
  int32_t val;
  /* A negative gain here is positive feedback (position/velocity error pushes the command
   * the WRONG way), not just a bad tune -- reject outright, same convention as
   * setVelocityClamp's val<0 check below. */
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetGains(val, AppGetConfig()->velGainKpQ8, AppGetConfig()->velGainKiQ8);
  return 0;
}
static void getGainsVelKp(char *b, size_t n)  { snprintf(b, n, "%ld", (long)AppGetConfig()->velGainKpQ8); }
static int  setGainsVelKp(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;   /* see setGainsPosP's comment */
  AppSetGains(AppGetConfig()->posGainQ8, val, AppGetConfig()->velGainKiQ8);
  return 0;
}
static void getGainsVelKi(char *b, size_t n)  { snprintf(b, n, "%ld", (long)AppGetConfig()->velGainKiQ8); }
static int  setGainsVelKi(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;   /* see setGainsPosP's comment */
  AppSetGains(AppGetConfig()->posGainQ8, AppGetConfig()->velGainKpQ8, val);
  return 0;
}
static void getVelocityClamp(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->velocityClamp); }
static int  setVelocityClamp(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetVelocityClamp(val);
  return 0;
}
static void getPositionMoveFeedforward(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->positionMoveFeedforwardQ8); }
static int setPositionMoveFeedforward(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0 || val > 256) return -1;
  AppSetPositionMoveFeedforward(val); return 0;
}
static void getAccelerationFeedforward(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->accelerationFeedforwardQ8); }
static int setAccelerationFeedforward(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0 || val > 8000000) return -1;
  AppSetAccelerationFeedforward(val); return 0;
}
static void getVelocityJerk(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->velocityJerk); }
static int setVelocityJerk(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0 || val > 10000000) return -1;
  AppSetVelocityJerk(val); return 0;
}
static void getVelocityFilterShift(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->velocityFilterShift); }
static int setVelocityFilterShift(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0 || val > 4) return -1;
  AppSetVelocityFilterShift(val);
  return 0;
}
static void getVelocityErrorDeadband(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->velocityErrorDeadbandCounts); }
static int setVelocityErrorDeadband(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetVelocityErrorDeadband(val);
  return 0;
}
static void getCommandVelocityClamp(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->commandVelocityClamp); }
static int setCommandVelocityClamp(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetCommandVelocityClamp(val);
  return 0;
}
static void getRampAccel(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->rampAccelCountsPerTickPerSec); }
static int  setRampAccel(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val <= 0) return -1;   /* 0/negative would mean "never ramp" --
    RampTick()'s step magnitude would be stuck at 0 forever, same as an accidental divide-by-zero
    in spirit even though the actual math wouldn't literally crash */
  AppSetRampAccel(val);
  return 0;
}
static void getTorqueClampMin(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->torqueClampMin); }
static int  setTorqueClampMin(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetTorqueClamp(val, AppGetConfig()->torqueClampMax);
  return 0;
}
static void getTorqueClampMax(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->torqueClampMax); }
static int  setTorqueClampMax(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetTorqueClamp(AppGetConfig()->torqueClampMin, val);
  return 0;
}

static void getOutputZeroTrim(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->zeroTrimCounts); }
static int  setOutputZeroTrim(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetZeroTrim(val);
  return 0;
}

static void getDiagTickCycles(char *b, size_t n)   { snprintf(b, n, "%lu", (unsigned long)AppGetLastTickCycles()); }
static void getDiagTickCyclesMax(char *b, size_t n) { snprintf(b, n, "%lu", (unsigned long)AppGetMaxTickCycles()); }
static void getDiagTickLoadPct(char *b, size_t n)  { snprintf(b, n, "%ld", (long)AppGetLastTickLoadPct()); }

static void getAnalogScale(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->analogVelocityScaleQ8); }
static int  setAnalogScale(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0) return -1;
  AppSetAnalogVelocityConfig(val, AppGetConfig()->analogVelocityHysteresisCounts);
  return 0;
}
static void getAnalogHysteresis(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->analogVelocityHysteresisCounts); }
static int  setAnalogHysteresis(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetAnalogVelocityConfig(AppGetConfig()->analogVelocityScaleQ8, val);
  return 0;
}
static void getAnalogRawAdc(char *b, size_t n)  { snprintf(b, n, "%ld", (long)AppGetAnalogRawAdc()); }
static void getAnalogReversed(char *b, size_t n) { snprintf(b, n, "%d", AppGetAnalogReversed()); }

static void getEnablePinAsserted(char *b, size_t n) { snprintf(b, n, "%d", AppGetHwEnableAsserted()); }

static void getFollowingErrorLimit(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->followingErrorLimitCounts); }
static int  setFollowingErrorLimit(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  AppSetFollowingErrorConfig(val, AppGetConfig()->followingErrorHardFault);
  return 0;
}
static void getFollowingErrorHardFault(char *b, size_t n) { snprintf(b, n, "%d", AppGetConfig()->followingErrorHardFault); }
static int  setFollowingErrorHardFault(const char *v) {
  int en;
  if (ParseBool(v, &en) != 0) return -1;
  AppSetFollowingErrorConfig(AppGetConfig()->followingErrorLimitCounts, en);
  return 0;
}
static void getFollowingErrorActive(char *b, size_t n) { snprintf(b, n, "%d", FaultGetFollowingErrorActive()); }

static void getSustainedCurrentMinTorque(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->sustainedCurrentMinTorque); }
static int  setSustainedCurrentMinTorque(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;   /* 0 == disabled, same convention
                                                           as position.error_limit */
  AppSetSustainedCurrentConfig(val, AppGetConfig()->sustainedCurrentWindowTicks);
  return 0;
}
static void getSustainedCurrentWindowTicks(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetConfig()->sustainedCurrentWindowTicks); }
static int  setSustainedCurrentWindowTicks(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val <= 0) return -1;
  AppSetSustainedCurrentConfig(AppGetConfig()->sustainedCurrentMinTorque, val);
  return 0;
}

static void getFaultLatched(char *b, size_t n) { snprintf(b, n, "%d", FaultIsLatched()); }
static void getFaultReason(char *b, size_t n)  { snprintf(b, n, "%s", FaultReasonName(FaultGetReason())); }

static void getPolarityLearned(char *b, size_t n) { snprintf(b, n, "%d", AppGetConfig()->polarityLearned); }
static void getPolaritySign(char *b, size_t n)     { snprintf(b, n, "%d", AppGetConfig()->polaritySign); }
static void getPolarityLearnFailed(char *b, size_t n) { snprintf(b, n, "%d", AppGetPolarityLearnFailed()); }
static void getPolarityLastMovedCounts(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppGetPolarityLearnMovedCounts()); }

static void getEncoderZSeen(char *b, size_t n) { snprintf(b, n, "%d", EncoderZWasSeenSinceEnable()); }

static void getSignalActive(char *b, size_t n) { snprintf(b, n, "%d", AppGetSignalActive()); }
static void getSignalType(char *b, size_t n)   { snprintf(b, n, "%s", SignalTypeName(AppGetSignalType())); }
static void getSignalTarget(char *b, size_t n) { snprintf(b, n, "%s", SignalTargetName(AppGetSignalTarget())); }

static void getMonitorActive(char *b, size_t n) { snprintf(b, n, "%d", MonitorGetEnabled()); }
static void getMonitorFreezePostTicks(char *b, size_t n) { snprintf(b, n, "%lu", (unsigned long)MonitorGetFreezePostTicks()); }
static int  setMonitorFreezePostTicks(const char *v) {
  int32_t val;
  if (ParseI32(v, &val) != 0 || val < 0) return -1;
  MonitorSetFreezePostTicks((uint32_t)val);   /* silently clamped to at most half the
    history buffer -- see Monitor.h's comment; a value above that doesn't error, it just
    doesn't get what it asked for, same "clamped, not rejected" convention as e.g.
    PositionMove.c's maxVelocityCountsPerTick<=0 case */
  return 0;
}

static void getAutotuneActive(char *b, size_t n) { snprintf(b, n, "%d", AppAutotuneBusy()); }
static void getAutotuneMethod(char *b, size_t n) { snprintf(b, n, "%s", AutotuneMethodName(AppAutotuneGetMethod())); }
static void getAutotuneResult(char *b, size_t n) { snprintf(b, n, "%s", AutotuneResultName(AppAutotuneGetResult())); }
static void getAutotuneKpResult(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppAutotuneGetKpResultQ8()); }
static void getAutotuneKiResult(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppAutotuneGetKiResultQ8()); }
static void getAutotunePosPResult(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppAutotuneGetPosPResultQ8()); }
static void getAutotunePeriod(char *b, size_t n) { snprintf(b, n, "%lu", (unsigned long)AppAutotuneGetMeasuredPeriodTicks()); }
static void getAutotuneGain(char *b, size_t n) { snprintf(b, n, "%ld", (long)AppAutotuneGetMeasuredGainQ8()); }

/* Wire integers use explicit fractional bits, avoiding ambiguous Q16/Q8
 * labels: value = raw / 2^fractional_bits. Torque units are normalized command
 * codes (32767 full scale), not measured Nm or fixed-point Q16. */
#define INT_META(unit) "\"type\":\"i32\",\"fractional_bits\":0,\"unit\":\"" unit "\""
#define UINT_META(unit) "\"type\":\"u32\",\"fractional_bits\":0,\"unit\":\"" unit "\""
#define Q8_META(unit) "\"type\":\"i32\",\"fractional_bits\":8,\"unit\":\"" unit "\""
#define BOOL_META "\"type\":\"bool\",\"unit\":\"boolean\",\"choices\":[\"0\",\"1\"]"
#define ENUM_META(values) "\"type\":\"enum\",\"unit\":\"name\",\"choices\":[" values "]"
#define TORQUE_META INT_META("torque_unit") ",\"full_scale\":32767"

static const var_entry_t kVars[] = {
  { "axis.enabled",      getServoEnabled,      setServoEnabled, BOOL_META },
  { "axis.enable_input", getEnablePinAsserted, NULL, BOOL_META },
  { "input.source",         getServoMode,         setServoMode, ENUM_META("\"stepdir\",\"host\",\"analog\"") },
  { "input.analog.scale",       getAnalogScale,       setAnalogScale, Q8_META("(encoder_count/tick)/adc_code") },
  { "input.analog.hysteresis",  getAnalogHysteresis,  setAnalogHysteresis, INT_META("adc_code") },
  { "input.analog.raw",     getAnalogRawAdc,      NULL, INT_META("adc_code") },
  { "input.analog.reverse",    getAnalogReversed,    NULL, BOOL_META },
  { "trajectory.move_feedforward", getPositionMoveFeedforward, setPositionMoveFeedforward, Q8_META("ratio") },
  { "trajectory.jerk_limit", getVelocityJerk, setVelocityJerk, INT_META("encoder_count/tick/second^2") },
  { "trajectory.velocity_limit", getCommandVelocityClamp, setCommandVelocityClamp, INT_META("encoder_count/tick") },
  { "trajectory.accel_limit",          getRampAccel,        setRampAccel, INT_META("encoder_count/tick/second") },
  { "position.reference", getPositionReference, NULL, INT_META("encoder_count") },
  { "position.valid", getControlValid, NULL, BOOL_META },
  { "position.command", getServoPositionCmd,  setServoPositionCmd, INT_META("encoder_count") },
  { "position.raw",     getServoPosition,     NULL, INT_META("encoder_count") },
  { "position.error", getFollowingError, NULL, INT_META("encoder_count") },
  { "position.kp",        getGainsPosP,         setGainsPosP, Q8_META("1/tick") },
  { "position.error_limit",      getFollowingErrorLimit,      setFollowingErrorLimit, INT_META("encoder_count") },
  { "position.error_trip", getFollowingErrorHardFault,  setFollowingErrorHardFault, BOOL_META },
  { "position.error_active",     getFollowingErrorActive,     NULL, BOOL_META },
  { "velocity.reference", getVelocityReference, NULL, INT_META("encoder_count/tick") },
  { "velocity.filtered", getVelocityFiltered, NULL, INT_META("encoder_count/tick") },
  { "velocity.error", getVelocityError, NULL, INT_META("encoder_count/tick") },
  { "velocity.control_error", getVelocityControlError, NULL, INT_META("encoder_count/tick") },
  { "velocity.valid", getControlValid, NULL, BOOL_META },
  { "velocity.command", getServoVelocityCmd,  setServoVelocityCmd, INT_META("encoder_count/tick") },
  { "velocity.raw",     getServoVelocity,     NULL, INT_META("encoder_count/tick") },
  { "velocity.kp",       getGainsVelKp,        setGainsVelKp, Q8_META("torque_unit/(encoder_count/tick)") },
  { "velocity.ki",       getGainsVelKi,        setGainsVelKi, Q8_META("torque_unit/(encoder_count/tick)/tick") },
  { "velocity.limit", getVelocityClamp,   setVelocityClamp, INT_META("encoder_count/tick") },
  { "velocity.filter_shift", getVelocityFilterShift, setVelocityFilterShift, INT_META("filter_shift") },
  { "velocity.deadband", getVelocityErrorDeadband, setVelocityErrorDeadband, INT_META("encoder_count/tick") },
  { "torque.command",   getServoTorqueCmd,    NULL, TORQUE_META },
  { "torque.accel_feedforward", getAccelerationFeedforward, setAccelerationFeedforward, Q8_META("torque_unit/(encoder_count/tick^2)") },
  { "torque.min", getTorqueClampMin, setTorqueClampMin, TORQUE_META },
  { "torque.max", getTorqueClampMax, setTorqueClampMax, TORQUE_META },
  { "torque.sustained_threshold",   getSustainedCurrentMinTorque,   setSustainedCurrentMinTorque, TORQUE_META },
  { "torque.sustained_ticks", getSustainedCurrentWindowTicks, setSustainedCurrentWindowTicks, INT_META("tick") },
  { "output.trim",   getOutputZeroTrim,    setOutputZeroTrim, INT_META("output_code") },
  { "output.polarity_learned",   getPolarityLearned,   NULL, BOOL_META },
  { "output.polarity",      getPolaritySign,      NULL, INT_META("sign") },
  { "output.polarity_failed",      getPolarityLearnFailed,     NULL, BOOL_META },
  { "output.polarity_motion", getPolarityLastMovedCounts, NULL, INT_META("encoder_count") },
  { "encoder.index_seen",     getEncoderZSeen,      NULL, BOOL_META },
  { "fault.latched",      getFaultLatched,      NULL, BOOL_META },
  { "fault.reason",       getFaultReason,       NULL, ENUM_META(FAULT_REASON_NAMES_JSON) },
  { "test.active",      getSignalActive,      NULL, BOOL_META },
  { "test.type",        getSignalType,        NULL, ENUM_META("\"none\",\"impulse\",\"square\"") },
  { "test.target",      getSignalTarget,      NULL, ENUM_META("\"torque\",\"position\"") },
  { "monitor.active",     getMonitorActive,     NULL, BOOL_META },
  { "monitor.post_trigger_ticks", getMonitorFreezePostTicks, setMonitorFreezePostTicks, UINT_META("tick") },
  { "autotune.active",       getAutotuneActive,     NULL, BOOL_META },
  { "autotune.method",       getAutotuneMethod,     NULL, ENUM_META("\"relay\"") },
  { "autotune.result",       getAutotuneResult,     NULL, ENUM_META("\"none\",\"ok\",\"no_oscillation\",\"bad_measurement\"") },
  { "autotune.velocity_kp",    getAutotuneKpResult,   NULL, Q8_META("torque_unit/(encoder_count/tick)") },
  { "autotune.velocity_ki",    getAutotuneKiResult,   NULL, Q8_META("torque_unit/(encoder_count/tick)/tick") },
  { "autotune.position_kp", getAutotunePosPResult, NULL, Q8_META("1/tick") },
  { "autotune.period_ticks", getAutotunePeriod,     NULL, UINT_META("tick") },
  { "autotune.plant_gain",  getAutotuneGain,       NULL, Q8_META("reserved") },
  { "diag.tick_cycles",     getDiagTickCycles,     NULL, UINT_META("cpu_cycle") },
  { "diag.tick_cycles_max", getDiagTickCyclesMax,  NULL, UINT_META("cpu_cycle") },
  { "diag.tick_load_pct",   getDiagTickLoadPct,    NULL, INT_META("percent") },
  { NULL, NULL, NULL, NULL },
};

/* ---- command handlers ----------------------------------------------------- */
typedef void (*cmd_fn)(int argc, char **argv);
typedef struct { const char *name; cmd_fn fn; const char *help; } cmd_t;
static const cmd_t kCommands[];

static void cmdVersion(int argc, char **argv) { (void)argc; (void)argv; respKV("version", FW_VERSION); }

static void cmdHelp(int argc, char **argv) {
  (void)argc; (void)argv;
  for (const cmd_t *c = kCommands; c->name; c++) respHelpLine(c->name, c->help);
}

/* Stream discovery JSON without allocating a document-sized RAM buffer. */
static void txJsonString(const char *value) {
  char buffer[64];
  size_t n = 0;
  buffer[n++] = '"';
  for (const unsigned char *p = (const unsigned char *)value; *p; p++) {
    char escaped[7];
    size_t len;
    if (*p == '"' || *p == '\\') { escaped[0] = '\\'; escaped[1] = (char)*p; len = 2; }
    else if (*p < 32) { snprintf(escaped, sizeof(escaped), "\\u%04x", (unsigned)*p); len = 6; }
    else { escaped[0] = (char)*p; len = 1; }
    if (n + len > sizeof(buffer)) { sActive->tx((uint8_t *)buffer, (uint16_t)n); n = 0; }
    memcpy(buffer + n, escaped, len); n += len;
  }
  if (n == sizeof(buffer)) { sActive->tx((uint8_t *)buffer, (uint16_t)n); n = 0; }
  buffer[n++] = '"';
  sActive->tx((uint8_t *)buffer, (uint16_t)n);
}
static void cmdSchema(int argc, char **argv) {
  (void)argc; (void)argv;
  txStr("{\"schema_version\":2,\"tick_hz\":1000,\"max_command_bytes\":128,\"commands\":{");
  for (const cmd_t *c = kCommands; c->name; c++) {
    if (c != kCommands) txStr(",");
    txJsonString(c->name); txStr(":"); txJsonString(c->help);
  }
  txStr("},\"variables\":{");
  for (const var_entry_t *v = kVars; v->name; v++) {
    if (v != kVars) txStr(",");
    txJsonString(v->name); txStr(":{\"access\":");
    txJsonString(v->set ? "rw" : "ro"); txStr(","); txStr(v->metadata); txStr("}");
  }
  txStr("},\"samples\":{\"sample\":{\"fields\":[\"tick\",\"position.raw\",\"velocity.raw\",\"torque.command\",\"position.error\",\"flags\"],");
  txStr("\"flags\":{\"axis.enabled\":1,\"fault.latched\":2,\"axis.enable_input\":4,\"position.valid\":8}}}}\r\n");
}

static void cmdGet(int argc, char **argv) {
  if (argc < 2) { respError("usage: get <name>"); return; }
  for (const var_entry_t *v = kVars; v->name; v++) {
    if (!strcmp(argv[1], v->name)) {
      char buf[24];
      v->get(buf, sizeof(buf));
      respKV(v->name, buf);
      return;
    }
  }
  respError("unknown variable");
}

static void cmdSet(int argc, char **argv) {
  if (argc < 3) { respError("usage: set <name> <value>"); return; }
  for (const var_entry_t *v = kVars; v->name; v++) {
    if (strcmp(argv[1], v->name) != 0) continue;
    if (!v->set) { respError("read-only"); return; }
    int result = v->set(argv[2]);
    if (result != 0) {
      respError("invalid value");
      return;
    }
    respEnd();
    return;
  }
  respError("unknown variable");
}

static void cmdSettings(int argc, char **argv) {
  (void)argc; (void)argv;
  for (const var_entry_t *v = kVars; v->name; v++) {
    char buf[24];
    v->get(buf, sizeof(buf));
    respKV(v->name, buf);
  }
}

static void cmdSample(int argc, char **argv) {
  (void)argc; (void)argv;
  AppControlSample sample;
  char buffer[96];
  AppGetControlSample(&sample);
  snprintf(buffer, sizeof(buffer), "sample=%lu,%ld,%ld,%ld,%ld,%lu",
           (unsigned long)sample.tick, (long)sample.position, (long)sample.velocity,
           (long)sample.torque, (long)sample.followingError, (unsigned long)sample.flags);
  txStr(buffer); txStr("\r\n");
}

static void cmdStatus(int argc, char **argv) {
  (void)argc; (void)argv;
  respKV("input.source", ServoModeName(AppGetServoMode()));
  respKVi("axis.enabled", AppGetEnabled());
  respKVi("position.raw", EncoderGetPositionCounts());
  respKVi("velocity.raw", AppGetLastVelocityActual());
  respKVi("torque.command", AppGetLastTorqueCommand());
  respKVi("axis.enable_input", AppGetHwEnableAsserted());
  respKVi("fault.latched", FaultIsLatched());
  respKV("fault.reason", FaultReasonName(FaultGetReason()));
}

static void cmdEnable(int argc, char **argv)  { (void)argc; (void)argv; AppSetEnabled(1); respKV("axis.enabled", "1"); }
static void cmdDisable(int argc, char **argv) { (void)argc; (void)argv; AppSetEnabled(0); respKV("axis.enabled", "0"); }

static void cmdFault(int argc, char **argv) {
  if (argc < 2) { respError("usage: fault <clear|freeze>"); return; }
  if (!strcmp(argv[1], "clear")) { FaultClear(); respKV("state", "cleared"); return; }
  if (!strcmp(argv[1], "freeze")) {
    /* OBD-II's "freeze frame" -- a snapshot of what was going on at the
     * exact tick a fault last LATCHED, queryable even after FaultClear()
     * (see main.h's FaultFreezeFrame comment for why that's deliberate).
     * Not reset by FaultClear() -- reports whatever the MOST RECENT
     * fault captured, which may be from well before the current (now
     * cleared) state. */
    if (!AppFaultFreezeExists()) { respError("no fault has latched yet this boot"); return; }
    const FaultFreezeFrame *f = AppGetFaultFreeze();
    respKVi("freeze.tick", (int32_t)f->tick);   /* free-running counter, never realistically
      reaches INT32_MAX (~24.8 days continuous @ 1kHz) within one boot -- not worth a
      dedicated unsigned-print helper for that */
    respKV ("freeze.reason", FaultReasonName(f->reason));
    respKVi("freeze.position.raw", f->position);
    respKVi("freeze.velocity.raw", f->velocity);
    respKVi("freeze.torque.command", f->torqueCommand);
    respKVi("freeze.position.reference", f->commandedPosition);
    respKVi("freeze.trajectory.velocity", f->commandedVelocity);
    respKV ("freeze.input.source", ServoModeName(f->mode));
    respKVi("freeze.axis.enabled", f->enabled);
    respKVi("freeze.axis.enable_input", f->hwEnableAsserted);
    respKVi("freeze.idle_reduced", f->idleReduced);
    respKVi("freeze.synthetic", f->synthetic);
    respKVi("freeze.cycle_start", f->cycleStart);
    respKVi("freeze.position.kp", f->posGainQ8);
    respKVi("freeze.velocity.kp", f->velGainKpQ8);
    respKVi("freeze.velocity.ki", f->velGainKiQ8);
    respKVi("freeze.velocity.limit", f->velocityClamp);
    respKVi("freeze.torque.min", f->torqueClampMin);
    respKVi("freeze.torque.max", f->torqueClampMax);
    return;
  }
  respError("unknown fault subcommand");
}

static void cmdPosition(int argc, char **argv) {
  if (argc != 2 || strcmp(argv[1], "zero")) { respError("usage: position zero"); return; }
  int result = AppResetPosition();
  if (result == -2) respError("position zero requires disabled, stationary axis with no active test");
  else if (result != 0) respError("position zero failed");
}

static void cmdOutput(int argc, char **argv) {
  if (argc != 2 || strcmp(argv[1], "learn_polarity")) { respError("usage: output learn_polarity"); return; }
  AppPolarityLearn(); respKV("state", "learning");
}

static void cmdSignal(int argc, char **argv) {
  if (argc < 2) { respError("usage: test <impulse|square|stop>"); return; }
  if (!strcmp(argv[1], "stop")) { AppSignalStop(); respKV("state", "stopped"); return; }
  if (!strcmp(argv[1], "impulse")) {
    if (argc < 3) { respError("usage: test impulse <amplitude>"); return; }
    int32_t amp;
    if (ParseI32(argv[2], &amp) != 0) { respError("invalid amplitude"); return; }
    AppSignalStart(SIGNAL_IMPULSE, SIGNAL_TARGET_TORQUE, amp, 1U, 1U);
    respKV("state", "running");
    return;
  }
  if (!strcmp(argv[1], "square")) {
    if (argc < 5) { respError("usage: test square <torque|position> <amplitude> <half_period_ticks> [duration_ticks]"); return; }
    SignalTarget target;
    if (!strcmp(argv[2], "torque")) target = SIGNAL_TARGET_TORQUE;
    else if (!strcmp(argv[2], "position")) target = SIGNAL_TARGET_POSITION;
    else { respError("invalid target -- must be torque or position"); return; }
    int32_t amp;
    uint32_t halfPeriod, duration = 0U;
    if (ParseI32(argv[3], &amp) != 0) { respError("invalid amplitude"); return; }
    if (ParseU32(argv[4], &halfPeriod) != 0 || halfPeriod == 0U) { respError("invalid half_period_ticks"); return; }
    if (argc >= 6 && ParseU32(argv[5], &duration) != 0) { respError("invalid duration_ticks"); return; }
    AppSignalStart(SIGNAL_SQUARE, target, amp, halfPeriod, duration);
    respKV("state", "running");
    return;
  }
  respError("unknown test subcommand");
}

static void cmdAutotuneStart(int argc, char **argv) {
  static const char *usage = "usage: autotune start relay <amplitude> [max_ticks]";
  if (argc < 3) { respError(usage); return; }

  if (!strcmp(argv[2], "relay")) {
    if (argc < 4) { respError(usage); return; }
    int32_t amp;
    uint32_t maxTicks = 0U;
    if (ParseI32(argv[3], &amp) != 0) { respError("invalid amplitude"); return; }
    if (argc >= 5 && ParseU32(argv[4], &maxTicks) != 0) { respError("invalid max_ticks"); return; }
    /* Relay's switching logic (Autotune.c's RelayTick) commits to +amplitude first and only
     * flips once velocity itself overshoots the opposite hysteresis band -- that only happens
     * if +torque actually produces +velocity. On a reversed-wiring plant it never flips at all:
     * holds one sign for the whole max_ticks window, every amplitude, and just saturates.
     * main.c's polaritySign correction (applied to AutotuneTick()'s output once learned) fixes
     * this, but only once learned -- so require it up front instead of silently running a
     * doomed campaign. */
    if (!AppGetConfig()->polarityLearned) { respError("run output learn_polarity before autotune start relay"); return; }
    AppAutotuneStartRelay(amp, maxTicks);
    respKV("state", "running");
    return;
  }
  respError(usage);
}

static void cmdAutotune(int argc, char **argv) {
  if (argc < 2) {
    respError("usage: autotune <start relay <amp> [max_ticks] | stop | apply>");
    return;
  }
  if (!strcmp(argv[1], "stop")) { AppAutotuneStop(); respKV("state", "stopped"); return; }
  if (!strcmp(argv[1], "apply")) {
    if (AppAutotuneGetResult() != AUTOTUNE_RESULT_OK) { respError("no successful autotune result to apply"); return; }
    AppAutotuneApply();
    respKV("state", "applied");
    return;
  }
  if (!strcmp(argv[1], "start")) { cmdAutotuneStart(argc, argv); return; }
  respError("unknown autotune subcommand");
}

static void cmdMonitor(int argc, char **argv) {
  if (argc < 2) { respError("usage: monitor <start|stop|dump|schema>"); return; }
  if (!strcmp(argv[1], "start")) { MonitorSetEnabled(1); respKV("state", "started"); return; }
  if (!strcmp(argv[1], "stop"))  { MonitorSetEnabled(0); respKV("state", "stopped"); return; }
  if (!strcmp(argv[1], "schema")) {
    /* Same JSON MonitorGetSchemaJson() serves in-band on channel 1 itself
     * (see docs/monitor-protocol.md) -- exposed here too, over the CLI's
     * channel 0, purely as a convenience for a human poking at a terminal
     * rather than a tool that only has channel 1 open. */
    txStr(MonitorGetSchemaJson());
    return;
  }
  if (!strcmp(argv[1], "dump")) {
    /* Queues the whole rolling history buffer (recorded continuously,
     * independent of monitor start/stop -- see Monitor.h's
     * MonitorDumpHistory() comment) for one-shot transmission over
     * channel 1, oldest frame first. This CLI response just confirms it
     * started and says how many frames to expect; the actual timeseries
     * data goes out the BINARY monitor port, not here -- same split as
     * `monitor start` itself. */
    MonitorDumpHistory();
    if (!MonitorDumpActive() && MonitorDumpFramesRemaining() == 0U) {
      respError("no history recorded yet");
      return;
    }
    respKV ("state", "dumping");
    respKVi("frames", (int32_t)MonitorDumpFramesRemaining());
    return;
  }
  respError("unknown monitor subcommand");
}

static void cmdConfig(int argc, char **argv) {
  if (argc < 2) { respError("usage: config <save|load|defaults>"); return; }
  if (!strcmp(argv[1], "save")) {
    if (AppConfigSave() != 0) { respError("flash write failed"); return; }
    respKV("state", "saved");
    return;
  }
  if (!strcmp(argv[1], "load")) {
    if (AppConfigLoad() != 0) { respError("no valid config in flash"); return; }
    respKV("state", "loaded");
    /* AppConfigLoad() deliberately does NOT touch live axis.enabled (see
     * docs/bringup.md's Headless Mode C operation section) --
     * warn here if the record just loaded disagrees with what's actually
     * running, so a runtime `config load` can never silently leave the
     * servo armed/disarmed differently than the operator thinks without at
     * least saying so. */
    if ((AppGetConfig()->enabled != 0) != (AppGetEnabled() != 0)) {
      respKV("warning", "axis.enabled not changed by config load -- "
                         "use enable/disable, or reboot to apply the loaded value");
    }
    return;
  }
  if (!strcmp(argv[1], "defaults")) {
    AppConfigDefaults();
    respKV("state", "defaults");
    return;
  }
  respError("unknown config subcommand");
}

static const cmd_t kCommands[] = {
  { "version",  cmdVersion,  "Firmware version" },
  { "help",     cmdHelp,     "List commands" },
  { "schema", cmdSchema, "JSON commands, variables, units, scaling and choices" },
  { "get",      cmdGet,      "<name> -- read one variable" },
  { "set",      cmdSet,      "<name> <value> -- write one variable" },
  { "settings", cmdSettings, "Dump every variable" },
  { "sample", cmdSample, "One-line CSV control sample; layout in schema.samples" },
  { "status",   cmdStatus,   "One-shot position/velocity/fault snapshot" },
  { "enable",   cmdEnable,   "Enable the servo (clears the PI integrator)" },
  { "disable",  cmdDisable,  "Disable the servo" },
  { "fault",    cmdFault,    "<clear|freeze> -- clear a latched hard fault, or dump the freeze frame" },
  { "position", cmdPosition, "zero -- reset coordinates while disabled and stationary" },
  { "output", cmdOutput, "learn_polarity -- one-time polarity commissioning step" },
  { "test",     cmdSignal,   "<impulse <amp> | square <torque|position> <amp> <half_period_ticks> [duration] | stop>" },
  { "monitor",  cmdMonitor,  "<start|stop|dump|schema> -- toggle the channel-1 binary telemetry stream, dump recorded history to it, or print its frame schema as JSON" },
  { "autotune", cmdAutotune,
    "<start relay <amp> [max_ticks] | stop | apply>" },
  { "config",   cmdConfig,   "<save|load|defaults> -- flash config persistence" },
  { NULL, NULL, NULL },
};

static void Dispatch(char *line) {
  char *argv[8];
  int argc = 0;
  char *p = strtok(line, " \t");
  while (p && argc < 8) { argv[argc++] = p; p = strtok(NULL, " \t"); }
  if (argc == 0) return;   /* blank line: no-op */

  for (const cmd_t *c = kCommands; c->name; c++) {
    if (!strcmp(argv[0], c->name)) { c->fn(argc, argv); respEnd(); return; }
  }
  respError("unknown command");
  respEnd();
}

static void PollInstance(ProtocolInstance *inst) {
  uint8_t b;
  while (RingPop(inst, &b)) {
    if (b == '\r' || b == '\n') {
      if (inst->pendEol && (char)b != inst->pendEol && inst->len == 0) { inst->pendEol = 0; continue; }   /* \r\n pair */
      inst->pendEol = (char)b;
      if (inst->len > 0) {
        inst->line[inst->len] = '\0';
        inst->len = 0;
        sActive = inst;
        Dispatch(inst->line);
        return;   /* one line per instance per call, so we never block the caller for long */
      }
      continue;
    }
    inst->pendEol = 0;
    if (inst->len < LINE_MAX) inst->line[inst->len++] = (char)b;
  }
}

/* A relay run can take up to its ~20s default timeout of otherwise-silent
 * tick-by-tick state cycling -- print a periodic status line on both
 * transports (unsolicited, so it doesn't go through sActive/respKV) rather
 * than leave the CLI looking hung between the `running` and final result
 * responses. AppAutotuneGetRelayProgressSeq() only advances every ~0.5s
 * (Autotune.c), so this poll is cheap even though it runs every superloop
 * iteration -- one integer comparison in the common case. */
static uint32_t sLastPrintedRelayProgressSeq = 0U;
static void PollAutotuneProgress(void) {
  if (!AppAutotuneBusy()) return;

  uint32_t seq = AppAutotuneGetRelayProgressSeq();
  if (seq == sLastPrintedRelayProgressSeq) return;
  sLastPrintedRelayProgressSeq = seq;

  /* peak_vel is the number to watch for a run heading toward
   * NO_OSCILLATION -- see AutotuneGetRelayProgressPeakVelocity()'s
   * comment: if it never approaches the configured loop gains (or just
   * AUTOTUNE_HYSTERESIS_COUNTS=2) the mechanism isn't producing a
   * velocity signal relay can switch on at all. */
  char buf[96];
  int n = snprintf(buf, sizeof(buf),
                "autotune.progress ticks=%lu/%lu switches=%lu/%lu peak_vel=%ld\r\n",
                (unsigned long)AppAutotuneGetRelayProgressTicksElapsed(),
                (unsigned long)AppAutotuneGetRelayProgressMaxTicks(),
                (unsigned long)AppAutotuneGetRelayProgressSwitchCount(),
                (unsigned long)AppAutotuneGetRelayProgressSwitchesNeeded(),
                (long)AppAutotuneGetRelayProgressPeakVelocity());
  if (n <= 0) return;
  sUsbInst.tx((const uint8_t *)buf, (uint16_t)n);
  sUartInst.tx((const uint8_t *)buf, (uint16_t)n);
}

void ProtocolInit(void) {
  sUsbInst.head = sUsbInst.tail = 0;
  sUsbInst.len = 0;
  sUsbInst.pendEol = 0;
  sUartInst.head = sUartInst.tail = 0;
  sUartInst.len = 0;
  sUartInst.pendEol = 0;
  sLastPrintedRelayProgressSeq = 0U;
}

void ProtocolPoll(void) {
  PollInstance(&sUsbInst);
  PollInstance(&sUartInst);
  PollAutotuneProgress();
}
