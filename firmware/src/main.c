/**
 * analog-servo -- closed-loop analog torque-command servo controller
 * firmware (STM32L152, NUCLEO-L152RE). See docs/architecture.md.
 *
 * Self-contained superloop: no RTOS, HAL polled/superloop, SysTick
 * timebase. This file is also the top-level mode-dispatch coordinator
 * (App* functions declared in main.h) -- that state spans StepDir/Encoder/
 * Ramp/Control/ConfigStore/Fault, no single one of those modules is the
 * natural owner.
 */
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "Encoder.h"
#include "StepDir.h"
#include "Output.h"
#include "Control.h"
#include "Ramp.h"
#include "PositionMove.h"
#include "Analog.h"
#include "Adc.h"
#include "Fault.h"
#include "Signal.h"
#include "Polarity.h"
#include "Indicator.h"
#include "ConfigStore.h"
#include "Monitor.h"
#include "Protocol.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"

void Error_Handler(void) { __disable_irq(); while (1) { } }

/* NUCLEO-L152RE: 8 MHz HSE -> PLL x12 /3 -> 32 MHz SYSCLK (L152's max).
 * PLLVCO (96 MHz) / 2 = 48 MHz taps USB automatically.
 *
 * HSE is RCC_HSE_BYPASS, NOT RCC_HSE_ON: on a Nucleo-64 board this "HSE" is
 * actually the ST-Link's own MCO output (a driven 8 MHz square wave on
 * PH0-OSC_IN), not a passive crystal -- the X3 crystal footprint is
 * unpopulated by default.
 *
 * 32 MHz needs PWR voltage-scale range1 (1.8V) and 1 flash wait state.
 * APB1/APB2 both stay undivided -- also what Ramp.c's SYSCLK_HZ-driven
 * RAMP_DENOM assumes for TIM6's PSC (see tim_l1.c). */
void SystemClock_Config(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) { }   /* wait for VOSF to clear */

  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_BYPASS;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL12;
  osc.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

/* ---- App* coordinator state ------------------------------------------------ */
static ServoMode      sMode = SERVO_MODE_STEPDIR;
static int        sEnabled = 0;
static int32_t     sCommandedVelocity = 0;
static ConfigData  sConfig;
static RampState   sRamp;
static PositionMoveState sPositionMove;   /* position.command -- shares sRamp above, see
                                              PositionMove.h */
static int32_t     sPositionTargetAccum = 0;
static uint32_t    sLastCycles = 0;

#define POLARITY_LEARN_TEST_TORQUE    (TORQUE_FULL_SCALE / 3)
#define POLARITY_LEARN_DURATION_TICKS 200   /* ~200ms @ 1kHz */
static int      sLearningActive = 0;
static int32_t  sLearningTicksRemaining = 0;
static int32_t  sLearningStartPosition = 0;
static int      sLearningFailed = 0;         /* last attempt rejected for insufficient
                                                 movement (see Polarity.h) -- does NOT
                                                 clear or overwrite a previously-learned
                                                 sConfig.polarityLearned/polaritySign */
static int32_t  sLearningLastMovedCounts = 0;

static int32_t sSignalBasePosition = 0;   /* captured at AppSignalStart() for SIGNAL_TARGET_POSITION */

static int32_t sLastVelocityActual = 0;
static uint32_t sFreezeTickCounter = 0;
static int       sWasFaultLatched = 0;
static int       sFaultFreezeExists = 0;
static FaultFreezeFrame sFaultFreeze;
static int32_t sLastTorqueCommand = 0;

/* Commanded (desired) position/velocity, snapshotted whenever a branch
 * actually computes a real one this tick -- NOT touched by branches with
 * no such target (polarity learning, raw open-loop signal torque,
 * autotune, disabled). Exists specifically so Monitor.c can report what
 * was ASKED for alongside what the encoder says actually happened. */
static AppControlSample sControlSample;
static uint32_t sStationaryTicks = 0;
static int32_t sLastCommandedPosition = 0;
static int32_t sLastCommandedVelocity = 0;

static int32_t sLastAnalogRawAdc = 0;
static int     sLastAnalogReversed = 0;
static int     sLastHwEnableAsserted = 0;

static uint32_t sLastTickCycles = 0;   /* whole-ISR DWT cycle count, last tick */
static uint32_t sMaxTickCycles = 0;    /* peak since boot -- see AppGetMaxTickCycles() */

static void ApplyConfigToModules(void) {
  ControlGains gains = {
    .posGainQ8 = sConfig.posGainQ8,
    .velGainKpQ8 = sConfig.velGainKpQ8,
    .velGainKiQ8 = sConfig.velGainKiQ8,
    .velocityClamp = sConfig.velocityClamp,
    .velocityFilterShift = sConfig.velocityFilterShift,
    .accelerationFeedforwardQ8 = sConfig.accelerationFeedforwardQ8,
    .torqueClampMin = sConfig.torqueClampMin,
    .torqueClampMax = sConfig.torqueClampMax,
    .velocityErrorDeadbandCounts = sConfig.velocityErrorDeadbandCounts,
  };
  ControlSetGains(&gains);
  FaultSetFollowingErrorConfig(sConfig.followingErrorLimitCounts, sConfig.followingErrorHardFault);
  FaultSetSustainedCurrentConfig(sConfig.sustainedCurrentMinTorque, sConfig.sustainedCurrentWindowTicks);
  OutputSetZeroTrim(sConfig.zeroTrimCounts);
  AnalogVelocitySetConfig(sConfig.analogVelocityScaleQ8, sConfig.analogVelocityHysteresisCounts);
  RampSetAcceleration(&sRamp, sConfig.rampAccelCountsPerTickPerSec);
  RampSetJerk(&sRamp, sConfig.velocityJerk);
}

ServoMode AppGetServoMode(void) { return sMode; }
int AppSetServoMode(ServoMode mode) {
  sMode = mode;
  sConfig.servoMode = (uint8_t)mode;
  sPositionTargetAccum = EncoderGetPositionCounts();   /* don't yank the axis on a mode switch */
  RampInit(&sRamp);
  RampSetAcceleration(&sRamp, sConfig.rampAccelCountsPerTickPerSec);
  RampSetJerk(&sRamp, sConfig.velocityJerk);   /* RampInit() resets to
    its own hardcoded default -- restore the configured rate immediately */
  PositionMoveInit(&sPositionMove);   /* a position_cmd move from before this switch must
    never resurface stale after a trip through another mode */
  AnalogVelocityReset();   /* Mode C: never inherit a stale hysteresis-held raw code */
  return 0;
}

int  AppGetEnabled(void) { return sEnabled; }
void AppSetEnabled(int enabled) {
  sEnabled = enabled ? 1 : 0;
  sConfig.enabled = (uint8_t)sEnabled;   /* live-only -- `config save` persists it */
  if (sEnabled) {
    ControlReset();
    sPositionTargetAccum = EncoderGetPositionCounts();
    RampInit(&sRamp);
    RampSetAcceleration(&sRamp, sConfig.rampAccelCountsPerTickPerSec);
    RampSetJerk(&sRamp, sConfig.velocityJerk);
    /* Deliberately NOT PositionMoveInit()'d here, unlike AppSetServoMode() --
       disabling mid-move is a pause, not a cancel. */
  }
}

int32_t AppGetCommandedVelocity(void) { return sCommandedVelocity; }
void    AppSetCommandedVelocity(int32_t countsPerTick) { sCommandedVelocity = countsPerTick; }

int32_t AppGetPositionCmd(void) { return PositionMoveGetTarget(&sPositionMove); }
void    AppSetPositionCmd(int32_t targetPositionCounts) {
  PositionMoveSetTarget(&sPositionMove, targetPositionCounts);
}

int AppResetPosition(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (sEnabled || sStationaryTicks < 50U || sLearningActive ||
      AppAutotuneBusy() || SignalIsActive()) {
    __set_PRIMASK(primask);
    return -2;
  }
  EncoderResetPosition();
  (void)StepDirGetDeltaCounts(); /* discard pending step input at new origin */
  sPositionTargetAccum = 0;
  sCommandedVelocity = 0;
  PositionMoveInit(&sPositionMove);
  RampInit(&sRamp);
  RampSetAcceleration(&sRamp, sConfig.rampAccelCountsPerTickPerSec);
  RampSetJerk(&sRamp, sConfig.velocityJerk);
  ControlReset();
  AnalogVelocityReset();
  sLastCommandedPosition = sLastCommandedVelocity = 0;
  sLastVelocityActual = sLastTorqueCommand = 0;
  sControlSample.position = sControlSample.velocity = sControlSample.torque = 0;
  sControlSample.followingError = 0;
  sControlSample.positionReference = sControlSample.velocityReference = 0;
  sControlSample.velocityFiltered = sControlSample.velocityError = sControlSample.velocityControlError = 0;
  sControlSample.flags &= 6U; /* retain fault/hardware-enable, not enabled/valid */
  OutputForceSafe();
  __set_PRIMASK(primask);
  return 0;
}

const ConfigData *AppGetConfig(void) { return &sConfig; }

int                     AppFaultFreezeExists(void) { return sFaultFreezeExists; }
const FaultFreezeFrame *AppGetFaultFreeze(void) { return &sFaultFreeze; }

void AppSetGains(int32_t posGainQ8, int32_t velGainKpQ8, int32_t velGainKiQ8) {
  sConfig.posGainQ8 = posGainQ8;
  sConfig.velGainKpQ8 = velGainKpQ8;
  sConfig.velGainKiQ8 = velGainKiQ8;
  ApplyConfigToModules();
}
void AppSetTorqueClamp(int32_t clampMin, int32_t clampMax) {
  sConfig.torqueClampMin = clampMin;
  sConfig.torqueClampMax = clampMax;
  ApplyConfigToModules();
}
void AppSetVelocityClamp(int32_t clamp) {
  sConfig.velocityClamp = clamp;
  ApplyConfigToModules();
}
void AppSetPositionMoveFeedforward(int32_t gain) {
  sConfig.positionMoveFeedforwardQ8 = gain;
}
void AppSetAccelerationFeedforward(int32_t gain) {
  sConfig.accelerationFeedforwardQ8 = gain;
  ApplyConfigToModules();
}
void AppSetVelocityJerk(int32_t jerk) {
  sConfig.velocityJerk = jerk;
  ApplyConfigToModules();
}
void AppSetVelocityFilterShift(int32_t shift) {
  sConfig.velocityFilterShift = shift;
  ApplyConfigToModules();
}
void AppSetVelocityErrorDeadband(int32_t deadbandCounts) {
  sConfig.velocityErrorDeadbandCounts = deadbandCounts;
  ApplyConfigToModules();
}
void AppSetCommandVelocityClamp(int32_t clamp) {
  sConfig.commandVelocityClamp = clamp;
}
static int32_t CommandVelocityLimit(void) {
  int32_t command = sConfig.commandVelocityClamp;
  return command > 0 && command < sConfig.velocityClamp ? command : sConfig.velocityClamp;
}
void AppSetRampAccel(int32_t accelCountsPerTickPerSec) {
  sConfig.rampAccelCountsPerTickPerSec = accelCountsPerTickPerSec;
  ApplyConfigToModules();
}
void AppSetZeroTrim(int32_t trimCounts) {
  sConfig.zeroTrimCounts = trimCounts;
  ApplyConfigToModules();
}
void AppSetAnalogVelocityConfig(int32_t scaleQ8, int32_t hysteresisCounts) {
  sConfig.analogVelocityScaleQ8 = scaleQ8;
  sConfig.analogVelocityHysteresisCounts = hysteresisCounts;
  ApplyConfigToModules();
}
int32_t AppGetAnalogRawAdc(void) { return sLastAnalogRawAdc; }
int     AppGetAnalogReversed(void) { return sLastAnalogReversed; }
int     AppGetHwEnableAsserted(void) { return sLastHwEnableAsserted; }
void AppSetFollowingErrorConfig(int32_t limitCounts, int hardFault) {
  sConfig.followingErrorLimitCounts = limitCounts;
  sConfig.followingErrorHardFault = hardFault ? 1 : 0;
  ApplyConfigToModules();
}
void AppSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks) {
  sConfig.sustainedCurrentMinTorque = minTorque;
  sConfig.sustainedCurrentWindowTicks = windowTicks;
  ApplyConfigToModules();
}

int AppConfigSave(void) { return ConfigStoreSave(&sConfig); }

int AppConfigLoad(void) {
  ConfigData loaded;
  if (ConfigStoreLoad(&loaded) != 0) return -1;
  sConfig = loaded;
  sMode = (ServoMode)sConfig.servoMode;
  ApplyConfigToModules();
  return 0;
}

void AppConfigDefaults(void) {
  ConfigDataSetDefaults(&sConfig);
  sMode = (ServoMode)sConfig.servoMode;
  ApplyConfigToModules();
}

void AppPolarityLearn(void) {
  if (FaultIsLatched()) return;   /* can't learn while faulted */
  /* Fresh test, fresh fault-detection window: Fault.c's sPolarityMismatchTicks
   * (FAULT_POLARITY's streak counter) is deliberately left untouched -- not
   * reset -- whenever |torqueCommand|/|velocityActual| drop below their own
   * minimums, which includes the idle gap between two commands typed by a
   * human. FaultClear() here is safe (FaultIsLatched() above already
   * confirmed sLatched==0, so this only zeroes transient window counters)
   * and gives this run the full, honest POLARITY_WINDOW_TICKS it's
   * supposed to get. */
  FaultClear();
  sLearningActive = 1;
  sLearningTicksRemaining = POLARITY_LEARN_DURATION_TICKS;
  sLearningStartPosition = EncoderGetPositionCounts();
  sLearningFailed = 0;   /* clear any stale failure from a previous attempt */
}
int     AppPolarityLearnBusy(void) { return sLearningActive; }
int     AppGetPolarityLearnFailed(void) { return sLearningFailed; }
int32_t AppGetPolarityLearnMovedCounts(void) { return sLearningLastMovedCounts; }

static int32_t ClampToTorqueRange(int32_t amp) {
  if (amp > sConfig.torqueClampMax) amp = sConfig.torqueClampMax;
  if (amp < sConfig.torqueClampMin) amp = sConfig.torqueClampMin;
  return amp;
}

void AppAutotuneStartRelay(int32_t amplitude, uint32_t maxTicks) {
  if (FaultIsLatched()) return;   /* can't run a test while faulted */
  FaultClear();   /* see AppPolarityLearn()'s comment -- fresh test, fresh window */
  AutotuneStartRelay(ClampToTorqueRange(amplitude), maxTicks);
}
void    AppAutotuneStop(void) { AutotuneStop(); }
int     AppAutotuneBusy(void) { return AutotuneIsActive(); }
AutotuneMethod AppAutotuneGetMethod(void) { return AutotuneGetMethod(); }
AutotuneResult AppAutotuneGetResult(void) { return AutotuneGetResult(); }
int32_t  AppAutotuneGetKpResultQ8(void) { return AutotuneGetKpResultQ8(); }
int32_t  AppAutotuneGetKiResultQ8(void) { return AutotuneGetKiResultQ8(); }
int32_t  AppAutotuneGetPosPResultQ8(void) { return AutotuneGetPosPResultQ8(); }
uint32_t AppAutotuneGetMeasuredPeriodTicks(void) { return AutotuneGetMeasuredPeriodTicks(); }
int32_t  AppAutotuneGetMeasuredGainQ8(void) { return AutotuneGetMeasuredGainQ8(); }
void AppAutotuneApply(void) {
  if (AutotuneGetResult() != AUTOTUNE_RESULT_OK) return;
  AppSetGains(AutotuneGetPosPResultQ8(), AutotuneGetKpResultQ8(), AutotuneGetKiResultQ8());
}

uint32_t AppAutotuneGetRelayProgressSeq(void) { return AutotuneGetRelayProgressSeq(); }
uint32_t AppAutotuneGetRelayProgressTicksElapsed(void) { return AutotuneGetRelayProgressTicksElapsed(); }
uint32_t AppAutotuneGetRelayProgressMaxTicks(void) { return AutotuneGetRelayProgressMaxTicks(); }
uint32_t AppAutotuneGetRelayProgressSwitchCount(void) { return AutotuneGetRelayProgressSwitchCount(); }
uint32_t AppAutotuneGetRelayProgressSwitchesNeeded(void) { return AutotuneGetRelayProgressSwitchesNeeded(); }
int32_t  AppAutotuneGetRelayProgressPeakVelocity(void) { return AutotuneGetRelayProgressPeakVelocity(); }

void AppSignalStart(SignalType type, SignalTarget target, int32_t amplitude,
                     uint32_t halfPeriodTicks, uint32_t durationTicks) {
  if (FaultIsLatched()) return;   /* can't run a test signal while faulted */
  FaultClear();   /* see AppPolarityLearn()'s comment -- fresh test, fresh window */
  int32_t amp = amplitude;
  if (target == SIGNAL_TARGET_TORQUE) {
    if (amp > sConfig.torqueClampMax) amp = sConfig.torqueClampMax;
    if (amp < sConfig.torqueClampMin) amp = sConfig.torqueClampMin;
  } else {
    /* SIGNAL_TARGET_POSITION: amplitude is encoder counts, not torque units. */
    sSignalBasePosition = EncoderGetPositionCounts();
    ControlReset();   /* don't inherit a stale PI integrator from before */
  }
  SignalStart(type, target, amp, halfPeriodTicks, durationTicks);
}
void       AppSignalStop(void) { SignalStop(); }
int        AppGetSignalActive(void) { return SignalIsActive(); }
SignalType   AppGetSignalType(void) { return SignalGetType(); }
SignalTarget AppGetSignalTarget(void) { return SignalGetTarget(); }
int        AppGetSignalCycleStart(void) { return SignalCycleStartPulse(); }

/* Where the velocity command comes from (SERVO_MODE_VELOCITY: the CLI's
 * velocity.command; SERVO_MODE_ANALOG: the ADC) -- not called for
 * SERVO_MODE_STEPDIR at all, that mode doesn't use a velocity command. */
static int32_t ComputeCommandedVelocity(void) {
  int32_t velocityCmd = sCommandedVelocity;
  if (sMode == SERVO_MODE_ANALOG) {
    int32_t rawAdc = AdcGetVelocityRaw();
    int reversed = (HAL_GPIO_ReadPin(ANALOG_REVERSE_GPIO_PORT, ANALOG_REVERSE_PIN) == GPIO_PIN_RESET);
    velocityCmd = AnalogVelocityTick(rawAdc, reversed);
    sLastAnalogRawAdc = rawAdc;
    sLastAnalogReversed = reversed;
    sCommandedVelocity = velocityCmd;   /* keep velocity.command's telemetry live */
  }
  int32_t limit = CommandVelocityLimit();
  return velocityCmd > limit ? limit : velocityCmd < -limit ? -limit : velocityCmd;
}

void AppControlTick(void) {
  HAL_GPIO_WritePin(TICK_ACTIVE_GPIO_PORT, TICK_ACTIVE_PIN, GPIO_PIN_SET);

  uint32_t nowCycles = DWT->CYCCNT;
  uint32_t dtCycles = nowCycles - sLastCycles;
  sLastCycles = nowCycles;

  int32_t velocityActual = EncoderGetDeltaSinceLastTick();
  if (velocityActual != 0) sStationaryTicks = 0;
  else if (sStationaryTicks < 50U) sStationaryTicks++;
  uint32_t totalEdges = EncoderTotalEdgesSeen();

  /* Hardware ENABLE pin -- read every tick regardless of mode/fault/enabled
   * state, so axis.enable_input telemetry is always live. Only the
   * normal-cascade branch below actually acts on it. */
  int hwEnableAsserted = (HAL_GPIO_ReadPin(HW_ENABLE_GPIO_PORT, HW_ENABLE_PIN) == GPIO_PIN_RESET);
  sLastHwEnableAsserted = hwEnableAsserted;

  int32_t torqueCommand = 0;

  if (sLearningActive) {
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);   /* doesn't run ControlTick, but keep
                                                             it correct-by-construction */
    torqueCommand = POLARITY_LEARN_TEST_TORQUE;
    sLearningTicksRemaining--;
    if (sLearningTicksRemaining <= 0) {
      int32_t moved = EncoderGetPositionCounts() - sLearningStartPosition;
      sLearningLastMovedCounts = moved;
      int8_t sign;
      if (PolarityLearnEvaluate(moved, &sign)) {
        sConfig.polarityLearned = 1;
        sConfig.polaritySign = sign;
        sLearningFailed = 0;
      } else {
        /* No meaningful motion observed -- almost certainly a disconnected/
         * seized motor or an unplugged encoder. Don't silently accept a
         * bogus default sign; leave any previously-learned value untouched
         * and flag the failure (output.polarity_failed) for the host to see. */
        sLearningFailed = 1;
      }
      sLearningActive = 0;
      torqueCommand = 0;
    }
  } else if (AppAutotuneBusy()) {
    /* Bypasses the cascade entirely -- the relay drives torque directly off
     * velocityActual's sign, it isn't pursuing a position target. */
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
    torqueCommand = AutotuneTick(velocityActual);
    if (sConfig.polarityLearned) torqueCommand *= sConfig.polaritySign;
  } else if (SignalIsActive()) {
    HAL_GPIO_WritePin(SIGNAL_BUSY_GPIO_PORT, SIGNAL_BUSY_PIN, GPIO_PIN_SET);   /* see
      gpio.h's SIGNAL_BUSY_PIN -- brackets just this branch, narrower than
      TICK_ACTIVE_PIN's whole-ISR window */
    int32_t signalValue = SignalTick();
    if (SignalGetTarget() == SIGNAL_TARGET_POSITION) {
      /* Closed-loop position excursion -- runs through the normal cascade
       * so each half-cycle is actively held (see Signal.h). */
      ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
      int32_t positionTarget = sSignalBasePosition + signalValue;
      sLastCommandedPosition = positionTarget;
      int32_t positionActual = EncoderGetPositionCounts();
      torqueCommand = ControlTick(positionTarget, positionActual, velocityActual, dtCycles);
    } else {
      /* Raw open-loop torque -- bypasses the cascade entirely, same as
       * polarity learning above. */
      torqueCommand = signalValue;
      sPositionTargetAccum = EncoderGetPositionCounts();
    }
    if (sConfig.polarityLearned) torqueCommand *= sConfig.polaritySign;
    HAL_GPIO_WritePin(SIGNAL_BUSY_GPIO_PORT, SIGNAL_BUSY_PIN, GPIO_PIN_RESET);
  } else if (sEnabled && !FaultIsLatched()) {
    int32_t positionTarget;
    int64_t referenceVelocityQ8 = 0;
    int positionTrajectoryActive = 0;
    /* hwEnableAsserted (above) is independent of axis.enabled -- when
     * deasserted, this branch still runs; it just overrides the target fed
     * into the cascade, so Control.c keeps running and smoothly
     * decelerates/holds instead of cutting torque outright. See
     * docs/bringup.md's Hardware enable section. */
    if (sMode == SERVO_MODE_STEPDIR) {
      /* No velocity-ramp concept in Mode A -- while hw-disabled, incoming
       * STEP pulses are dropped rather than accumulated. */
      int32_t stepDelta = hwEnableAsserted ? StepDirGetDeltaCounts() : 0;
      sPositionTargetAccum += stepDelta;
      positionTarget = sPositionTargetAccum;
      sLastCommandedPosition = positionTarget;
    } else {
      /* Mode B (host-commanded) or Mode C (analog-commanded) velocity --
       * ramps through the same Ramp.c accel/decel limiter into the shared
       * position-target accumulator. */
      int32_t velocityCmd = ComputeCommandedVelocity();
      int32_t rampedVelocity;
      positionTrajectoryActive = PositionMoveIsActive(&sPositionMove);
      if (positionTrajectoryActive) {
        if (!hwEnableAsserted) {
          RampSetTarget(&sRamp, 0);
          rampedVelocity = RampSmoothTick(&sRamp, dtCycles);
        } else {
          rampedVelocity = PositionMoveTick(&sPositionMove, &sRamp, sPositionTargetAccum,
                                             CommandVelocityLimit(), dtCycles);
        }
      } else {
        if (!hwEnableAsserted) velocityCmd = 0;   /* ramp DOWN to zero, not an instant cut */
        RampSetTarget(&sRamp, velocityCmd);
        rampedVelocity = RampSmoothTick(&sRamp, dtCycles);
      }
      sLastCommandedVelocity = rampedVelocity;
      referenceVelocityQ8 = (int64_t)rampedVelocity *
          (positionTrajectoryActive ? sConfig.positionMoveFeedforwardQ8 : 256);
      sPositionTargetAccum += rampedVelocity;
      positionTarget = sPositionTargetAccum;
      sLastCommandedPosition = positionTarget;
    }
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
    int32_t positionActual = EncoderGetPositionCounts();
    torqueCommand = ControlTickWithReferenceLimitQ8(positionTarget, positionActual, velocityActual,
        referenceVelocityQ8, positionTrajectoryActive ? CommandVelocityLimit() : sConfig.velocityClamp, dtCycles);
    if (sConfig.polarityLearned) torqueCommand *= sConfig.polaritySign;
  } else {
    /* Disabled (and not learning): keep the position target accumulator
     * glued to actual position, so re-enabling doesn't yank the axis back
     * to a stale target accumulated while disabled. */
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
    sPositionTargetAccum = EncoderGetPositionCounts();
  }

  /* Following error is only meaningful when Control.c's cascade actually ran
   * this tick: the normal enabled path, or a SIGNAL_TARGET_POSITION signal. */
  int useRealPositionError = (sEnabled && !sLearningActive &&
                               !AppAutotuneBusy() && !SignalIsActive()) ||
                              (SignalIsActive() && SignalGetTarget() == SIGNAL_TARGET_POSITION);
  int32_t positionError = useRealPositionError ? ControlGetLastPositionError() : 0;
  /* FaultTick needs axis-coordinate torque, before the learned mapping to
   * the physical DAC sign. */
  int32_t axisTorque = torqueCommand;
  if (sConfig.polarityLearned) axisTorque *= sConfig.polaritySign;
  FaultSetPolarityCheckSuppressed(sLearningActive);
  FaultTick(axisTorque, velocityActual, positionError, totalEdges);

  sFreezeTickCounter++;
  int faultNowLatched = FaultIsLatched();
  if (faultNowLatched && !sWasFaultLatched) {
    /* The 0->1 edge only -- capture from the LOCAL variables (this tick's
     * real values), not sLastTorqueCommand/sLastVelocityActual. */
    sFaultFreeze.tick = sFreezeTickCounter;
    sFaultFreeze.reason = FaultGetReason();
    sFaultFreeze.position = EncoderGetPositionCounts();
    sFaultFreeze.velocity = velocityActual;
    sFaultFreeze.torqueCommand = torqueCommand;
    sFaultFreeze.commandedPosition = sLastCommandedPosition;
    sFaultFreeze.commandedVelocity = sLastCommandedVelocity;
    sFaultFreeze.mode = sMode;
    sFaultFreeze.enabled = (uint8_t)sEnabled;
    sFaultFreeze.hwEnableAsserted = (uint8_t)hwEnableAsserted;
    sFaultFreeze.idleReduced = 0;   /* reserved, always 0 */
    sFaultFreeze.synthetic = (uint8_t)(AppPolarityLearnBusy() || AppAutotuneBusy() ||
                                        AppGetSignalActive());
    sFaultFreeze.cycleStart = (uint8_t)AppGetSignalCycleStart();
    sFaultFreeze.posGainQ8 = sConfig.posGainQ8;
    sFaultFreeze.velGainKpQ8 = sConfig.velGainKpQ8;
    sFaultFreeze.velGainKiQ8 = sConfig.velGainKiQ8;
    sFaultFreeze.velocityClamp = sConfig.velocityClamp;
    sFaultFreeze.torqueClampMin = sConfig.torqueClampMin;
    sFaultFreeze.torqueClampMax = sConfig.torqueClampMax;
    sFaultFreezeExists = 1;
  }
  sWasFaultLatched = faultNowLatched;

  if (FaultIsLatched()) {
    SignalStop();     /* a fault mid-test aborts it, rather than leaving it running silently */
    AutotuneStop();   /* ditto for an in-progress autotune */
    OutputForceSafe();
    IndicatorSetState(INDICATOR_FAULT);
    sLastTorqueCommand = 0;
  } else {
    OutputSetTorqueCommand(torqueCommand);
    IndicatorSetState(sEnabled ? INDICATOR_RUNNING : INDICATOR_IDLE);
    sLastTorqueCommand = torqueCommand;
  }
  sLastVelocityActual = velocityActual;
  sControlSample.tick++;
  sControlSample.position = EncoderGetPositionCounts();
  sControlSample.velocity = velocityActual;
  sControlSample.torque = sLastTorqueCommand;
  int samplePositionValid = useRealPositionError && hwEnableAsserted && !FaultIsLatched();
  sControlSample.followingError = samplePositionValid ? positionError : 0;
  ControlVelocitySample velocitySample;
  ControlGetVelocitySample(&velocitySample);
  sControlSample.positionReference = samplePositionValid ? sLastCommandedPosition : 0;
  sControlSample.velocityReference = samplePositionValid ? velocitySample.reference : 0;
  sControlSample.velocityFiltered = samplePositionValid ? velocitySample.filtered : 0;
  sControlSample.velocityError = samplePositionValid ? velocitySample.error : 0;
  sControlSample.velocityControlError = samplePositionValid ? velocitySample.controlError : 0;
  sControlSample.flags = (sEnabled ? 1U : 0U) | (FaultIsLatched() ? 2U : 0U) |
                         (hwEnableAsserted ? 4U : 0U) | (samplePositionValid ? 8U : 0U);

  MonitorTick();

  /* Whole-ISR cycle count, same window TICK_ACTIVE_PIN brackets externally. */
  sLastTickCycles = DWT->CYCCNT - nowCycles;
  if (sLastTickCycles > sMaxTickCycles) sMaxTickCycles = sLastTickCycles;

  HAL_GPIO_WritePin(TICK_ACTIVE_GPIO_PORT, TICK_ACTIVE_PIN, GPIO_PIN_RESET);
}

void AppGetControlSample(AppControlSample *out) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = sControlSample;
  __set_PRIMASK(primask);
}

int32_t AppGetLastVelocityActual(void) { return sLastVelocityActual; }
int32_t AppGetLastTorqueCommand(void) { return sLastTorqueCommand; }
int32_t AppGetLastCommandedPosition(void) { return sLastCommandedPosition; }
int32_t AppGetLastCommandedVelocity(void) { return sLastCommandedVelocity; }

uint32_t AppGetLastTickCycles(void) { return sLastTickCycles; }
uint32_t AppGetMaxTickCycles(void) { return sMaxTickCycles; }

#ifndef SYSCLK_HZ
#error "SYSCLK_HZ must be defined by the env's build_flags (see platformio.ini)"
#endif
int32_t AppGetLastTickLoadPct(void) {
  return (int32_t)((sLastTickCycles * 100U) / (SYSCLK_HZ / 1000U));
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  GpioInit();

  /* Enable the DWT cycle counter -- used for Ramp.c's dtCycles integration. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  ConfigDataSetDefaults(&sConfig);
  if (ConfigStoreLoad(&sConfig) == 0) {
    sMode = (ServoMode)sConfig.servoMode;
  }
  ApplyConfigToModules();

  EncoderInit();
  StepDirInit();
  OutputInit();
  AdcInit();            /* Mode C's ADC input -- after GpioInit() above has already put
                            ANALOG_VELOCITY_ADC_PIN into analog mode */
  ControlInit(NULL);   /* gains already applied via ApplyConfigToModules() above */
  RampInit(&sRamp);
  RampSetAcceleration(&sRamp, sConfig.rampAccelCountsPerTickPerSec);
  RampSetJerk(&sRamp, sConfig.velocityJerk);
  PositionMoveInit(&sPositionMove);
  FaultInit();
  SignalInit();
  AutotuneInit();
  IndicatorInit();
  ProtocolInit();
  MonitorInit();
  UsbCompositeCdcInit();   /* channel 0: CLI (Protocol.c); channel 1: binary
                               telemetry (Monitor.c) -- see docs/monitor-protocol.md */
  UsartInit();

  sPositionTargetAccum = EncoderGetPositionCounts();
  sLastCycles = DWT->CYCCNT;

  /* Headless bring-up (Mode C in particular): a persisted axis.enabled=1
   * (ConfigStoreLoad() above, NOT the runtime `config load` command) arms
   * the servo with no CLI interaction at all. It won't actually move
   * anything yet -- the hardware ENABLE pin (read fresh every control
   * tick) still gates the ramp/target. See docs/bringup.md. Done here
   * (everything else initialized, ISR not running yet) rather than right
   * after ConfigStoreLoad() above, since AppSetEnabled() reads
   * EncoderGetPositionCounts() and EncoderInit() hasn't run that early. */
  if (sConfig.enabled) AppSetEnabled(1);

  ControlTickTimerInit();  /* starts the 1kHz control tick -- do this last, once
                               every other module is ready to be driven by it */

  for (;;) {
    ProtocolPoll();
    IndicatorPoll();
    MonitorPoll();
  }
}
