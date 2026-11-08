/*
 * Native unit tests for the cascaded P->PI control core (Control.c). Pure
 * fixed-point math, no HAL/mocks needed at all.
 * Run: pio test -e native
 */
#include <unity.h>
#include <stdlib.h>
#include "Control.h"
#include "main.h"
#include "Encoder.h"
#include "Output.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"

/* Unused in this suite -- exist only because [env:native]'s build_src_filter
 * is env-wide, so Fault.c and Protocol.c (and everything they call into) get
 * linked into every test/test_* executable, this one included. See
 * platformio.ini and analog_quadrature's test suites for the same pattern. */
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

void setUp(void) {
  ControlGains g = {
    .posGainQ8 = 256,       /* 1:1 */
    .velGainKpQ8 = 512,     /* 2:1 */
    .velGainKiQ8 = 26,      /* ~0.1:1 per tick */
    .velocityClamp = 100000,
    .torqueClampMin = -1000,
    .torqueClampMax = 1000,
  };
  ControlInit(&g);
}
void tearDown(void) {}

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* Toy discrete plant: velocity integrates torque (velocity += torque/16 each
 * tick), position integrates velocity -- just enough closed-loop dynamics to
 * exercise convergence, not a claim about the real electromechanical plant. */
static void RunPlant(int32_t positionTarget, int ticks, int32_t *position, int32_t *velocity) {
  int32_t pos = 0, vel = 0;
  for (int i = 0; i < ticks; i++) {
    int32_t torque = ControlTick(positionTarget, pos, vel, 1000);
    vel += torque / 16;
    pos += vel;
  }
  *position = pos;
  *velocity = vel;
}

static void test_step_response_converges(void) {
  int32_t pos, vel;
  RunPlant(10000, 500, &pos, &vel);
  TEST_ASSERT_TRUE(Abs32(pos - 10000) < 200);
}

static void test_zero_error_zero_torque(void) {
  int32_t torque = ControlTick(0, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(0, torque);
}

static void test_position_p_only_no_velocity_feedback(void) {
  /* posErr=1000 -> velocityTarget = 1000 (1:1 gain); velErr = 1000 - 0 = 1000;
   * pTerm = 1000*512/256 = 2000, clamped to torqueClampMax=1000. */
  int32_t torque = ControlTick(1000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(1000, torque);
}

static void test_gains_hot_reload_zeroes_position_loop(void) {
  ControlGains g = *ControlGetGains();
  g.posGainQ8 = 0;   /* position error no longer produces a velocity target */
  ControlSetGains(&g);
  int32_t torque = ControlTick(1000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(0, torque);
  TEST_ASSERT_EQUAL_INT32(0, ControlGetLastVelocityTarget());
}

static void test_anti_windup_no_delayed_unwind_on_reversal(void) {
  /* Sustain a huge positive error for many ticks -- torque pins at
   * torqueClampMax the whole time (conditional integration should be
   * freezing the integral, not winding it up unboundedly). */
  for (int i = 0; i < 300; i++) {
    int32_t torque = ControlTick(1000000, 0, 0, 1000);
    TEST_ASSERT_EQUAL_INT32(1000, torque);
  }
  /* Now reverse hard: if the integrator had wound up during saturation, the
   * output would stay pinned positive for a while even with a huge negative
   * error. With conditional integration, it should react on the very next
   * tick instead. */
  int32_t torqueAfterReversal = ControlTick(-1000000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(-1000, torqueAfterReversal);
}

static void test_torque_scale_tightens_clamp(void) {
  /* Same setup as test_position_p_only_no_velocity_feedback: posErr=1000 ->
   * pTerm=2000, which unscaled clamps to torqueClampMax=1000. At 50% scale
   * (128 in Q8.8) the effective clamp is 500. */
  ControlSetTorqueScale(128);
  int32_t torque = ControlTick(1000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(500, torque);
}

static void test_torque_scale_resets_to_full_via_init_and_reset(void) {
  ControlSetTorqueScale(64);   /* 25% */
  ControlInit(NULL);           /* NULL: keep setUp()'s gains, only reset transient state */
  int32_t torque = ControlTick(1000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(1000, torque);   /* back to full-scale clamp */

  ControlSetTorqueScale(64);
  ControlReset();
  torque = ControlTick(1000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(1000, torque);
}

static void test_torque_scale_anti_windup_still_correct_when_reduced(void) {
  /* Same shape as test_anti_windup_no_delayed_unwind_on_reversal, but at 50%
   * scale (effective clamp +-500) -- the saturation check inside ControlTick
   * must compare against the SCALED clamp, not the configured one, or this
   * would either pin at the wrong level or let the integral wind up. */
  ControlSetTorqueScale(128);
  for (int i = 0; i < 300; i++) {
    int32_t torque = ControlTick(1000000, 0, 0, 1000);
    TEST_ASSERT_EQUAL_INT32(500, torque);
  }
  int32_t torqueAfterReversal = ControlTick(-1000000, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(-500, torqueAfterReversal);
}

static void test_reset_clears_integrator(void) {
  /* Build up some integral action first. */
  for (int i = 0; i < 50; i++) ControlTick(500, 0, 0, 1000);
  ControlReset();
  /* Immediately after reset, with zero error, torque must be exactly zero --
   * a nonzero result would mean stale integral state survived the reset. */
  int32_t torque = ControlTick(0, 0, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(0, torque);
}

static void test_last_position_error_reported(void) {
  ControlTick(1234, 34, 0, 1000);
  TEST_ASSERT_EQUAL_INT32(1234 - 34, ControlGetLastPositionError());
}

static void test_velocity_tick_matches_equivalent_position_tick(void) {
  /* ControlTick(positionTarget, positionActual, ...) computes
   * velocityTarget = posErr*posGainQ8>>8 (clamped), then runs the same PI
   * stage ControlVelocityTick() exposes directly -- feeding that same
   * velocityTarget straight to ControlVelocityTick() must produce an
   * identical torque command, tick for tick, since it's the same stage. */
  ControlGains gains = { .posGainQ8 = 256, .velGainKpQ8 = 128, .velGainKiQ8 = 16,
                         .velocityClamp = 10000, .torqueClampMin = -30000, .torqueClampMax = 30000 };
  ControlInit(&gains);
  int32_t viaPosition = ControlTick(500, 0, 10, 1000);   /* posErr=500, velTarget=500 (unclamped) */

  ControlInit(&gains);
  int32_t viaVelocity = ControlVelocityTick(500, 10, 1000);

  TEST_ASSERT_EQUAL_INT32(viaPosition, viaVelocity);
  TEST_ASSERT_EQUAL_INT32(500, ControlGetLastVelocityTarget());
}

static void test_velocity_tick_does_not_touch_last_position_error(void) {
  ControlGains gains = { .posGainQ8 = 256, .velGainKpQ8 = 128, .velGainKiQ8 = 0,
                         .velocityClamp = 10000, .torqueClampMin = -30000, .torqueClampMax = 30000 };
  ControlInit(&gains);
  ControlTick(1234, 34, 0, 1000);   /* sets ControlGetLastPositionError() to 1200 */
  ControlVelocityTick(999, 0, 1000);   /* no position target at all in this mode */
  TEST_ASSERT_EQUAL_INT32(1234 - 34, ControlGetLastPositionError());
}

static void test_velocity_tick_anti_windup_matches_position_path(void) {
  /* Same reversal-without-delayed-unwind check as
   * test_anti_windup_no_delayed_unwind_on_reversal, but driven straight
   * through ControlVelocityTick() -- the anti-windup logic lives in the
   * shared stage, so it must behave identically regardless of entry point. */
  ControlGains gains = { .posGainQ8 = 256, .velGainKpQ8 = 0, .velGainKiQ8 = 256,
                         .velocityClamp = 10000, .torqueClampMin = -1000, .torqueClampMax = 1000 };
  ControlInit(&gains);
  for (int i = 0; i < 20; i++) ControlVelocityTick(10000, 0, 1000);   /* saturate high */
  int32_t afterReversal = ControlVelocityTick(-10000, 0, 1000);   /* error flips sign */
  TEST_ASSERT_TRUE(afterReversal < 1000);   /* unwinds immediately, not stuck at the old clamp */
}

/* Regression guard against both failure modes on either side of gain
 * selection: unity gains saturate the clamps almost immediately for even a
 * modest error, while gains chosen too far below unity quantize away to
 * near-zero/exactly-1 through the cascade's chained integer truncation.
 * Checks the sane middle ground directly against
 * ConfigStore.h's named default-gain constants -- without linking
 * ConfigStore.c's flash-dependent code, since this only needs the #define
 * values, not ConfigDataSetDefaults() itself. */
static void test_default_gains_avoid_saturation_and_quantization_deadzone(void) {
  ControlGains gains = {
    .posGainQ8 = CONFIG_DEFAULT_POS_GAIN_Q8,
    .velGainKpQ8 = CONFIG_DEFAULT_VEL_GAIN_KP_Q8,
    .velGainKiQ8 = CONFIG_DEFAULT_VEL_GAIN_KI_Q8,
    .velocityClamp = 10000,
    .torqueClampMin = -TORQUE_FULL_SCALE,
    .torqueClampMax = TORQUE_FULL_SCALE,
  };
  ControlInit(&gains);
  /* A moderate, realistic signal-generator test amplitude, with zero plant
   * feedback -- the worst case for quantization (no motor attached is
   * exactly the scenario that surfaced this). Must be clearly nonzero (not
   * lost to integer truncation) but nowhere near the torque clamp (not
   * immediately saturating -- the unity-gain mistake this replaced). */
  int32_t torque = ControlTick(500, 0, 0, 1000);
  TEST_ASSERT_TRUE(torque > 10);
  TEST_ASSERT_TRUE(torque < 1000);
}

static void test_velocity_feedforward_tracks_without_position_lag(void) {
  ControlGains g = *ControlGetGains(); g.velGainKiQ8 = 0; ControlInit(&g);
  TEST_ASSERT_EQUAL_INT32(0, ControlTickWithVelocity(100, 100, 20, 20, 1000));
  TEST_ASSERT_EQUAL_INT32(20, ControlGetLastVelocityTarget());
}

static void test_feedforward_adds_feedback_and_clamps_total(void) {
  ControlGains g = *ControlGetGains(); g.velocityClamp = 40; g.velGainKiQ8 = 0;
  ControlInit(&g);
  ControlTickWithVelocity(10, 0, 0, 20, 1000);
  TEST_ASSERT_EQUAL_INT32(30, ControlGetLastVelocityTarget());
  ControlTickWithVelocity(100, 0, 0, 20, 1000);
  TEST_ASSERT_EQUAL_INT32(40, ControlGetLastVelocityTarget());
  ControlTickWithVelocity(-100, 0, 0, -20, 1000);
  TEST_ASSERT_EQUAL_INT32(-40, ControlGetLastVelocityTarget());
}

static void test_fractional_position_error_is_symmetric_and_nonzero(void) {
  ControlGains g = *ControlGetGains();
  g.posGainQ8=64; g.velGainKpQ8=1024; g.velGainKiQ8=0; ControlInit(&g);
  TEST_ASSERT_EQUAL_INT32(1, ControlTick(1,0,0,1000));
  TEST_ASSERT_EQUAL_INT32(-1, ControlTick(-1,0,0,1000));
}

static void test_fractional_integral_accumulates_both_signs(void) {
  ControlGains g = *ControlGetGains();
  g.velGainKpQ8=0; g.velGainKiQ8=1;
  for (int sign=-1; sign<=1; sign+=2) {
    ControlInit(&g); int32_t torque=0;
    for (int i=0;i<256;i++) torque=ControlVelocityTick(sign,0,1000);
    TEST_ASSERT_EQUAL_INT32(sign,torque);
  }
}

static void test_velocity_filter_is_causal_and_resets(void) {
  ControlGains g = *ControlGetGains();
  g.velGainKpQ8=256; g.velGainKiQ8=0; g.velocityFilterShift=1; ControlInit(&g);
  ControlVelocityTick(0,0,1000);
  TEST_ASSERT_EQUAL_INT32(-5, ControlVelocityTick(0,10,1000));
  ControlReset();
  TEST_ASSERT_EQUAL_INT32(0, ControlVelocityTick(0,0,1000));
}

static void test_acceleration_feedforward_uses_current_ramp_change(void) {
  ControlGains g=*ControlGetGains(); g.velGainKiQ8=0; g.accelerationFeedforwardQ8=2560;
  ControlInit(&g);
  TEST_ASSERT_EQUAL_INT32(15,ControlTickWithVelocity(0,0,3,3,1000));
  TEST_ASSERT_EQUAL_INT32(8,ControlTickWithVelocity(0,0,3,3,1000));
  TEST_ASSERT_EQUAL_INT32(-6,ControlTickWithVelocity(0,0,1,1,1000));
  TEST_ASSERT_EQUAL_INT32(0,ControlTick(0,0,0,1000));
}

static void test_fractional_reference_feedforward_is_preserved(void) {
  ControlGains g=*ControlGetGains(); g.velGainKpQ8=1024; g.velGainKiQ8=0;
  ControlInit(&g);
  TEST_ASSERT_EQUAL_INT32(2,ControlTickWithReferenceQ8(0,0,0,128,1000));
  TEST_ASSERT_EQUAL_INT32(-2,ControlTickWithReferenceQ8(0,0,0,-128,1000));
}

static void test_command_specific_speed_limit_cannot_exceed_feedback_limit(void) {
  ControlGains g=*ControlGetGains(); g.velocityClamp=60;g.velGainKiQ8=0;ControlInit(&g);
  ControlTickWithReferenceLimitQ8(1000,0,0,0,40,1000);
  TEST_ASSERT_EQUAL_INT32(40,ControlGetLastVelocityTarget());
  ControlTickWithReferenceLimitQ8(1000,0,0,0,100,1000);
  TEST_ASSERT_EQUAL_INT32(60,ControlGetLastVelocityTarget());
}

/* A large velGainKpQ8 (tuned aggressively for fast settle) turns +-1 count/tick of
 * UNAVOIDABLE encoder quantization noise at rest into a disproportionate torque kick with
 * no net motion -- real wasted power/heat, not a tuning mistake. The deadband exists to
 * suppress exactly that, without masking a real error. */
static void test_velocity_error_deadband_suppresses_quantization_noise_only(void) {
  ControlGains g = {
    .posGainQ8 = 0, .velGainKpQ8 = 1024, .velGainKiQ8 = 0,
    .velocityClamp = 10000,
    .torqueClampMin = -TORQUE_FULL_SCALE, .torqueClampMax = TORQUE_FULL_SCALE,
    .velocityErrorDeadbandCounts = 1,
  };
  ControlInit(&g);
  /* Exactly +-1 count/tick (the deadband's own boundary) -- a settled axis can't resolve
   * any finer than this, so there's nothing genuine to correct. */
  TEST_ASSERT_EQUAL_INT32(0, ControlTick(0, 0, 1, 1000));
  TEST_ASSERT_EQUAL_INT32(0, ControlTick(0, 0, -1, 1000));
  /* A genuinely larger error (2 counts, outside the 1-count deadband) still produces real
   * torque -- this isn't just zeroing the proportional term unconditionally. */
  TEST_ASSERT_EQUAL_INT32(-8, ControlTick(0, 0, 2, 1000));
}

static void test_velocity_error_deadband_disabled_by_default_is_a_no_op(void) {
  ControlGains g = {
    .posGainQ8 = 0, .velGainKpQ8 = 1024, .velGainKiQ8 = 0,
    .velocityClamp = 10000,
    .torqueClampMin = -TORQUE_FULL_SCALE, .torqueClampMax = TORQUE_FULL_SCALE,
    .velocityErrorDeadbandCounts = 0,
  };
  ControlInit(&g);
  TEST_ASSERT_EQUAL_INT32(-4, ControlTick(0, 0, 1, 1000));
}

static void test_velocity_diagnostics_match_filter_and_deadband(void) {
  ControlGains g=*ControlGetGains();
  g.velocityFilterShift=1; g.velocityErrorDeadbandCounts=20;
  ControlInit(&g);
  ControlVelocityTick(20,0,1000);
  ControlVelocityTick(20,10,1000);
  ControlVelocitySample sample;
  ControlGetVelocitySample(&sample);
  TEST_ASSERT_EQUAL_INT32(20,sample.reference);
  TEST_ASSERT_EQUAL_INT32(5,sample.filtered);
  TEST_ASSERT_EQUAL_INT32(15,sample.error);
  TEST_ASSERT_EQUAL_INT32(0,sample.controlError);
  ControlReset(); ControlGetVelocitySample(&sample);
  TEST_ASSERT_EQUAL_INT32(0,sample.reference);
  TEST_ASSERT_EQUAL_INT32(0,sample.filtered);
  TEST_ASSERT_EQUAL_INT32(0,sample.error);
  TEST_ASSERT_EQUAL_INT32(0,sample.controlError);
}

static void test_velocity_diagnostics_round_fractional_values_symmetrically(void) {
  ControlGains g=*ControlGetGains();
  g.velocityFilterShift=1; g.velocityErrorDeadbandCounts=0;
  ControlInit(&g);
  ControlVelocityTick(0,0,1000); ControlVelocityTick(0,1,1000);
  ControlVelocitySample sample; ControlGetVelocitySample(&sample);
  TEST_ASSERT_EQUAL_INT32(1,sample.filtered);
  TEST_ASSERT_EQUAL_INT32(-1,sample.error);
  ControlReset();
  ControlVelocityTick(0,0,1000); ControlVelocityTick(0,-1,1000);
  ControlGetVelocitySample(&sample);
  TEST_ASSERT_EQUAL_INT32(-1,sample.filtered);
  TEST_ASSERT_EQUAL_INT32(1,sample.error);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_velocity_diagnostics_match_filter_and_deadband);
  RUN_TEST(test_velocity_diagnostics_round_fractional_values_symmetrically);
  RUN_TEST(test_command_specific_speed_limit_cannot_exceed_feedback_limit);
  RUN_TEST(test_velocity_error_deadband_suppresses_quantization_noise_only);
  RUN_TEST(test_velocity_error_deadband_disabled_by_default_is_a_no_op);
  RUN_TEST(test_fractional_reference_feedforward_is_preserved);
  RUN_TEST(test_acceleration_feedforward_uses_current_ramp_change);
  RUN_TEST(test_fractional_position_error_is_symmetric_and_nonzero);
  RUN_TEST(test_fractional_integral_accumulates_both_signs);
  RUN_TEST(test_velocity_filter_is_causal_and_resets);
  RUN_TEST(test_velocity_feedforward_tracks_without_position_lag);
  RUN_TEST(test_feedforward_adds_feedback_and_clamps_total);
  RUN_TEST(test_step_response_converges);
  RUN_TEST(test_zero_error_zero_torque);
  RUN_TEST(test_position_p_only_no_velocity_feedback);
  RUN_TEST(test_gains_hot_reload_zeroes_position_loop);
  RUN_TEST(test_anti_windup_no_delayed_unwind_on_reversal);
  RUN_TEST(test_torque_scale_tightens_clamp);
  RUN_TEST(test_torque_scale_resets_to_full_via_init_and_reset);
  RUN_TEST(test_torque_scale_anti_windup_still_correct_when_reduced);
  RUN_TEST(test_reset_clears_integrator);
  RUN_TEST(test_last_position_error_reported);
  RUN_TEST(test_velocity_tick_matches_equivalent_position_tick);
  RUN_TEST(test_velocity_tick_does_not_touch_last_position_error);
  RUN_TEST(test_velocity_tick_anti_windup_matches_position_path);
  RUN_TEST(test_default_gains_avoid_saturation_and_quantization_deadzone);
  return UNITY_END();
}

void AppSetCommandVelocityClamp(int32_t c) { (void)c; }

void AppSetVelocityFilterShift(int32_t c) { (void)c; }
void AppSetVelocityErrorDeadband(int32_t c) { (void)c; }

void AppSetAccelerationFeedforward(int32_t c) { (void)c; }

void AppSetVelocityJerk(int32_t c) { (void)c; }

void AppSetPositionMoveFeedforward(int32_t c) { (void)c; }
