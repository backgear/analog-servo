/**
 * analog-servo -- closed-loop analog torque-command servo controller
 * firmware (STM32L152/NUCLEO-L152RE, or STM32F103/"Blue Pill" -- this
 * `f103` branch). See docs/architecture.md.
 *
 * Self-contained superloop: no RTOS, HAL polled/superloop, SysTick
 * timebase. This file is also the top-level mode-dispatch coordinator
 * (App* functions declared in main.h) -- that state spans StepDir/Encoder/
 * Ramp/Control/ConfigStore/Fault, no single one of those modules is the
 * natural owner.
 */
#include "main.h"
#include "BuildId.h"
#include "gpio.h"
#include "tim.h"
#include "Encoder.h"
#include "StepDir.h"
#include "Output.h"
#include "Control.h"
#include "Ramp.h"
#include "GainGlide.h"
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
#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
#include "Hall.h"
#include "Commutation.h"
#endif

void Error_Handler(void) { __disable_irq(); while (1) { } }

#if defined(STM32L152xE)
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
#else
/* F103 "Blue Pill": 8 MHz HSE -> PLL x9 -> 72 MHz SYSCLK (standard Blue
 * Pill config). APB1 capped at 36 MHz (div 2); APB2 stays at 72 MHz. USB
 * needs the 48 MHz tap (72 MHz / 1.5), which only lands exactly if HSE
 * really is 8 MHz -- see HSE_VALUE in platformio.ini. Unlike the L152
 * build above, HSE is a real crystal here (RCC_HSE_ON, not _BYPASS) --
 * the Blue Pill has no onboard debugger supplying a driven clock. */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 max 36 MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  RCC_PeriphCLKInitTypeDef usb = {0};
  usb.PeriphClockSelection = RCC_PERIPHCLK_USB;
  usb.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;   /* 72 MHz / 1.5 = 48 MHz */
  if (HAL_RCCEx_PeriphCLKConfig(&usb) != HAL_OK) Error_Handler();
}
#endif

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

static OutputTopology sOutputTopology = OUTPUT_TOPOLOGY_ANALOG_TORQUE;

#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
#define COMMUTATION_LEARN_TEST_TORQUE    (TORQUE_FULL_SCALE / 3)   /* same scale as
                                                                        POLARITY_LEARN_TEST_TORQUE */
#define COMMUTATION_LEARN_DURATION_TICKS 5000   /* ~5s @ 1kHz -- tune on the bench; needs to
                                                     be long enough to observe
                                                     COMMUTATION_LEARN_MIN_TRANSITIONS Hall
                                                     transitions at whatever speed this test
                                                     torque actually produces on the real
                                                     plant, unknown until one is on hand */
static int      sCommLearningActive = 0;
static int32_t  sCommLearningTicksRemaining = 0;
static int      sCommLearningFailed = 0;     /* same "don't silently overwrite a good prior
                                                  calibration" convention as sLearningFailed */
static int      sLastHallSectorSeen = -2;    /* -2 == "no reading yet this attempt" --
                                                  distinct from -1 (HallDecode's own invalid-
                                                  state marker) so the first valid reading of
                                                  an attempt is never mistaken for a transition.
                                                  Shared with the pole-detect branch below --
                                                  the two never run concurrently. */

/* Pole-count/counts-per-elec-rev detection -- see Commutation.h/
 * docs/two-phase-drive.md. Same duration budget as commutation-learn's --
 * both just need enough Hall-sector transitions to average out noise, not
 * a full mechanical revolution -- untuned, same "needs real hardware"
 * caveat as everything else on this branch. */
#define POLE_LEARN_DURATION_TICKS 5000   /* ~5s @ 1kHz -- tune on the bench */
static int      sPoleLearnActive = 0;
static int32_t  sPoleLearnTicksRemaining = 0;
static int      sPoleLearnFailed = 0;
static int32_t  sPoleLearnEncoderCpr = 0;      /* caller's known CPR */
#endif /* ANALOG_SERVO_TWO_PHASE_DRIVE */

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

/* Last electrical angle (degrees, 0..359) the two-phase-output branch's
 * commutation stage computed -- see Commutation.c. Only ever written inside
 * the ANALOG_SERVO_TWO_PHASE_DRIVE branch below; stays at its initialized 0
 * for the lifetime of a build/run that never takes that branch (the
 * default env:analog_servo build, or env:analog_servo_two_phase while
 * output.topology isn't two_phase_analog or a fault is latched) -- same
 * "stale, not wrong" convention as sLastCommandedPosition/Velocity above.
 * Monitor.c's electricalAngleDeg frame field is only meaningful once this
 * has actually been driven by that branch at least once. */
static int32_t sLastElectricalAngleDeg = 0;

/* Raw Hall sector (0..5, or -1 for HallDecode's own invalid-state marker --
 * see Hall.h) for telemetry -- separate from sLastHallSectorSeen above,
 * which is learn-attempt-scoped transition-tracking state (reset to -2 at
 * the start of each commutation-learn/detect-poles attempt) and not
 * suitable for a continuous telemetry readout. Written every tick the top
 * of AppControlTick()'s Hall read runs (output.topology=two_phase_analog
 * only), BEFORE any fault-latch branch -- deliberately visible even while
 * a fault is latched, since a bad Hall wiring/wrong-sequence bring-up
 * problem is exactly the kind of fault this is meant to help diagnose.
 * Stays at its initialized 0 for the lifetime of a build/run that never
 * takes that branch -- same "stale, not wrong" convention as
 * sLastElectricalAngleDeg above; see Monitor.c's hallSector frame field. */
static int32_t sLastHallSectorForTelemetry = 0;

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
  OutputSetZeroTrim(sConfig.zeroTrim);
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

OutputTopology AppGetOutputTopology(void) { return sOutputTopology; }
int AppSetOutputTopology(OutputTopology topology) {
#ifndef ANALOG_SERVO_TWO_PHASE_DRIVE
  /* Not built into this env -- Hall.c/Commutation.c aren't compiled in at
   * all, see platformio.ini's analog_servo vs. analog_servo_two_phase envs. */
  if (topology == OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) return -1;
#endif
  sOutputTopology = topology;
  sConfig.outputTopology = (uint8_t)topology;
  OutputSetTopology(topology);
  return 0;
}
void AppSetCommutationConfig(int32_t polePairs, int32_t countsPerElecRev) {
  sConfig.commutationPolePairs = polePairs;
  sConfig.commutationCountsPerElecRev = countsPerElecRev;
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
      AppAutotuneBusy() || SignalIsActive() ||
      AppCommutationLearnBusy() || AppCommutationDetectPolesBusy()) {
    __set_PRIMASK(primask);
    return -2;
  }
  /* Two-phase commutation uses encoder coordinates for electrical phase.
   * Rebasing those needs a separate phase-preserving origin abstraction --
   * not built, see docs/two-phase-drive.md. */
  if (sOutputTopology == OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) {
    __set_PRIMASK(primask);
    return -3;
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
  sLastElectricalAngleDeg = 0;
  sLastHallSectorForTelemetry = 0;
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
void AppSetZeroTrim(int32_t trimTorqueUnits) {
  sConfig.zeroTrim = trimTorqueUnits;
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
  sOutputTopology = (OutputTopology)sConfig.outputTopology;
  OutputSetTopology(sOutputTopology);
  ApplyConfigToModules();
  return 0;
}

void AppConfigDefaults(void) {
  ConfigDataSetDefaults(&sConfig);
  sMode = (ServoMode)sConfig.servoMode;
  sOutputTopology = (OutputTopology)sConfig.outputTopology;
  OutputSetTopology(sOutputTopology);
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

#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
void AppCommutationLearn(void) {
  if (FaultIsLatched()) return;   /* can't learn while faulted */
  if (sOutputTopology != OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) return;   /* needs the Hall
    pins/angle math this topology wires up -- see docs/two-phase-drive.md */
  FaultClear();   /* see AppPolarityLearn()'s comment -- fresh test, fresh window */
  CommutationLearnReset();
  sCommLearningActive = 1;
  sCommLearningTicksRemaining = COMMUTATION_LEARN_DURATION_TICKS;
  sCommLearningFailed = 0;   /* clear any stale failure from a previous attempt */
  sLastHallSectorSeen = -2;
}
int AppCommutationLearnBusy(void) { return sCommLearningActive; }
int AppGetCommutationLearnFailed(void) { return sCommLearningFailed; }

void AppCommutationDetectPoles(int32_t encoderCpr) {
  if (encoderCpr <= 0) return;   /* caller's known-CPR precondition -- see
    Commutation.h: the Z-index fallback for unknown CPR was designed but
    deliberately not implemented, see docs/two-phase-drive.md */
  if (FaultIsLatched()) return;   /* can't learn while faulted */
  if (sOutputTopology != OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) return;   /* same requirement as
    AppCommutationLearn() -- needs the Hall pins/angle math this topology wires up */
  FaultClear();   /* see AppPolarityLearn()'s comment -- fresh test, fresh window */
  CommutationPoleLearnBySpacingReset();
  sPoleLearnActive = 1;
  sPoleLearnEncoderCpr = encoderCpr;
  sPoleLearnTicksRemaining = POLE_LEARN_DURATION_TICKS;
  sPoleLearnFailed = 0;   /* clear any stale failure from a previous attempt */
  sLastHallSectorSeen = -2;
}
int AppCommutationDetectPolesBusy(void) { return sPoleLearnActive; }
int AppGetCommutationDetectPolesFailed(void) { return sPoleLearnFailed; }
#else
/* Not built into this env -- see AppSetOutputTopology()'s identical
 * comment. Degenerate no-op/always-idle/always-failed implementations so
 * Protocol.c's `commutation.*` commands don't need their own #ifdef. */
void AppCommutationLearn(void) { }
int AppCommutationLearnBusy(void) { return 0; }
int AppGetCommutationLearnFailed(void) { return 0; }
void AppCommutationDetectPoles(int32_t encoderCpr) { (void)encoderCpr; }
int AppCommutationDetectPolesBusy(void) { return 0; }
int AppGetCommutationDetectPolesFailed(void) { return 0; }
#endif /* ANALOG_SERVO_TWO_PHASE_DRIVE */

#ifdef ANALOG_SERVO_HAS_AUTOTUNE
static int32_t ClampToTorqueRange(int32_t amp) {
  if (amp > sConfig.torqueClampMax) amp = sConfig.torqueClampMax;
  if (amp < sConfig.torqueClampMin) amp = sConfig.torqueClampMin;
  return amp;
}
#endif

/* FLASH_IS_SCARCE omits relay autotune and RLS to recover code space.
 * With ANALOG_SERVO_HAS_AUTOTUNE, these bodies call Autotune/RLS/GainGlide;
 * otherwise the stand-ins below keep the coordinator API available while
 * Protocol.c omits the tuning CLI surface. Unused implementations are
 * removed by the linker. */
#ifdef ANALOG_SERVO_HAS_AUTOTUNE
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

/* The control tick is hardware-fixed today (TIM6/TIM1's PSC/ARR -- see
 * tim_l1.c/tim_f1.c), not read back from any register here -- this is the
 * one place in this feature that needs to know it as a number, so it's a
 * single named constant rather than a literal buried in RlsStart()'s call
 * site. Update this if that PSC/ARR configuration ever changes the actual
 * tick rate. */
#define CONTROL_TICK_HZ 1000U

static uint32_t sRlsGlideMs;   /* stashed verbatim -- AppRlsApply() converts this to ticks
                                   using CONTROL_TICK_HZ itself once it actually starts a
                                   glide */
static GainGlideState sGainGlide;   /* driven once per tick from AppControlTick(), see its
                                        top -- shared by any future caller of
                                        GainGlideStart(), today only AppRlsApply() */

void AppRlsStart(int32_t lambdaQ8, uint32_t glideMs) {
  if (FaultIsLatched()) return;   /* same precondition as relay -- see AppAutotuneStartRelay() */
  sRlsGlideMs = glideMs;
  RlsStart(lambdaQ8, CONTROL_TICK_HZ);
}
void    AppRlsStop(void) { RlsStop(); }
int     AppRlsBusy(void) { return RlsIsActive(); }
AutotuneResult AppRlsGetResult(void) { return RlsGetResult(); }
int32_t  AppRlsGetKpResultQ8(void) { return RlsGetKpResultQ8(); }
int32_t  AppRlsGetKiResultQ8(void) { return RlsGetKiResultQ8(); }
int32_t  AppRlsGetThetaAQ8(void) { return RlsGetThetaAQ8(); }
int32_t  AppRlsGetThetaBQ8(void) { return RlsGetThetaBQ8(); }
int64_t  AppRlsGetPTrace(void) { return RlsGetPTrace(); }
uint32_t AppRlsGetTicksObserved(void) { return RlsGetTicksObserved(); }
uint32_t AppRlsGetProgressSeq(void) { return RlsGetProgressSeq(); }
void     AppRlsSetAggressiveness(int32_t extraSquarings) { RlsSetPoleAggressiveness(extraSquarings); }
int32_t  AppRlsGetAggressiveness(void) { return RlsGetPoleAggressiveness(); }
void     AppRlsApply(void) {
  if (RlsGetResult() != AUTOTUNE_RESULT_OK) return;
  /* position.kp is deliberately untouched -- RLS only ever estimates the
   * velocity loop's (a, b), see RLS.h. Glide rather than instant-apply
   * (unlike AppAutotuneApply()'s relay counterpart): a candidate can arrive
   * mid-move since RLS rides along the normal cascade instead of relay's
   * dedicated bypass, so an instant jump would step torque output
   * audibly/mechanically while already tracking. */
  const ControlGains *current = ControlGetGains();
  ControlGains target = *current;
  target.velGainKpQ8 = RlsGetKpResultQ8();
  target.velGainKiQ8 = RlsGetKiResultQ8();
  uint32_t totalTicks = (uint32_t)(((uint64_t)sRlsGlideMs * CONTROL_TICK_HZ) / 1000U);
  GainGlideStart(&sGainGlide, current, &target, totalTicks);
}
#else /* !ANALOG_SERVO_HAS_AUTOTUNE -- degenerate stand-ins, same convention as
         ANALOG_SERVO_TWO_PHASE_DRIVE's #else branch further down */
void    AppAutotuneStartRelay(int32_t amplitude, uint32_t maxTicks) { (void)amplitude; (void)maxTicks; }
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
#endif /* ANALOG_SERVO_HAS_AUTOTUNE */

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

#ifdef ANALOG_SERVO_HAS_AUTOTUNE
  /* AppRlsApply()'s glide, if one is in flight -- applies regardless of
   * which mode branch runs below, same as FaultTick()/MonitorTick() at the
   * bottom of this function. Calls ControlSetGains() directly rather than
   * AppSetGains()/ApplyConfigToModules(), which would redundantly re-touch
   * Fault/Output/Analog/Ramp config every single glide tick for no reason
   * -- only the three gain fields ever change mid-glide. */
  if (GainGlideActive(&sGainGlide)) {
    ControlGains glided;
    if (GainGlideTick(&sGainGlide, &glided)) ControlSetGains(&glided);
  }
#endif

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

#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
  /* Hall sector, output.topology=two_phase_analog only -- read once here,
   * used both by the commutation-learn branch below (transition detection)
   * and the final output-dispatch section at the bottom (angle source).
   * -1 (HallDecode's invalid-state marker) when topology isn't
   * two_phase_analog at all -- harmless, nothing reads it in that case. */
  int hallSector = -1;
  if (sOutputTopology == OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) {
    int ha = (HAL_GPIO_ReadPin(HALL_A_GPIO_PORT, HALL_A_PIN) == GPIO_PIN_RESET);
    int hb = (HAL_GPIO_ReadPin(HALL_B_GPIO_PORT, HALL_B_PIN) == GPIO_PIN_RESET);
    int hc = (HAL_GPIO_ReadPin(HALL_C_GPIO_PORT, HALL_C_PIN) == GPIO_PIN_RESET);
    hallSector = HallDecode(ha, hb, hc);
    sLastHallSectorForTelemetry = hallSector;   /* unconditional, even across a latched
        fault -- see sLastHallSectorForTelemetry's own comment above: this is a
        bring-up diagnostic, not control-loop state. */
  }
#endif

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
#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
  } else if (sCommLearningActive) {
    /* Open-loop two-phase commutation driven directly off the live Hall
     * sector, deliberately NOT offset-corrected -- see
     * docs/two-phase-drive.md's "One-time alignment commissioning". Each
     * observed sector transition feeds CommutationLearnAccumulate() with
     * the raw (uncorrected) encoder angle at that instant; the final
     * output-dispatch section below does the actual Ia/Ib conversion using
     * hallSector (read above) the same way the normal two_phase_analog
     * path does, just with torqueCommand pinned to a small constant
     * instead of whatever the real cascade would produce. */
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
    torqueCommand = COMMUTATION_LEARN_TEST_TORQUE;
    if (hallSector >= 0) {
      if (sLastHallSectorSeen != -2 && hallSector != sLastHallSectorSeen) {
        int32_t rawEncoderAngleDeg = CommutationElectricalAngleFromEncoderDeg(
            EncoderGetPositionCounts(), sConfig.commutationCountsPerElecRev, 0);
        CommutationLearnAccumulate(rawEncoderAngleDeg, hallSector);
      }
      sLastHallSectorSeen = hallSector;
    }
    sCommLearningTicksRemaining--;
    if (sCommLearningTicksRemaining <= 0) {
      int32_t offsetDeg;
      if (CommutationLearnEvaluate(&offsetDeg)) {
        sConfig.commutationLearned = 1;
        sConfig.commutationOffsetDeg = offsetDeg;
        sCommLearningFailed = 0;
      } else {
        /* Too few Hall transitions observed -- don't persist a meaningless
         * offset; leave any previously-learned value untouched, same
         * convention as polarity-learn's rejection path above. */
        sCommLearningFailed = 1;
      }
      sCommLearningActive = 0;
      torqueCommand = 0;
    }
  } else if (sPoleLearnActive) {
    /* Same open-loop two-phase test-pulse drive as commutation-learn above
     * (torqueCommand pinned to a small constant, Ia/Ib synthesized from the
     * live Hall sector in the output-dispatch section below), just feeding
     * CommutationPoleLearnBySpacing* instead -- see Commutation.h's
     * "Pole-count detection" section. */
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
    torqueCommand = COMMUTATION_LEARN_TEST_TORQUE;
    if (hallSector >= 0) {
      if (sLastHallSectorSeen != -2 && hallSector != sLastHallSectorSeen) {
        CommutationPoleLearnBySpacingAccumulate(EncoderGetPositionCounts(), hallSector);
      }
      sLastHallSectorSeen = hallSector;
    }
    sPoleLearnTicksRemaining--;
    if (sPoleLearnTicksRemaining <= 0) {
      int32_t countsPerElecRev, polePairs;
      int ok = CommutationPoleLearnBySpacingEvaluate(sPoleLearnEncoderCpr, &countsPerElecRev, &polePairs);
      if (ok) {
        sConfig.commutationPolePairs = polePairs;
        sConfig.commutationCountsPerElecRev = countsPerElecRev;
        sPoleLearnFailed = 0;
      } else {
        /* Too little data -- don't persist a meaningless value; leave any
         * previously-set config untouched, same convention as the other
         * learn routines' rejection paths. */
        sPoleLearnFailed = 1;
      }
      sPoleLearnActive = 0;
      torqueCommand = 0;
    }
#endif /* ANALOG_SERVO_TWO_PHASE_DRIVE */
  } else if (AppAutotuneBusy()) {
    /* Bypasses the cascade entirely -- the relay drives torque directly off
     * velocityActual's sign, it isn't pursuing a position target. Dead on
     * F103 (AppAutotuneBusy() permanently returns 0 there -- see its own
     * #else stand-in above), but AutotuneTick() itself still needs an
     * #ifdef, not just a dead branch: Autotune.c isn't linked on F103 at
     * all (see platformio.ini), so the real call has to be compiled out,
     * not merely unreached. */
    ControlSetTorqueScale(CONTROL_TORQUE_SCALE_FULL_Q8);
#ifdef ANALOG_SERVO_HAS_AUTOTUNE
    torqueCommand = AutotuneTick(velocityActual);
#endif
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
#ifdef ANALOG_SERVO_HAS_AUTOTUNE
    /* Near-passive observer -- sees the same already-polarity-corrected
     * torque command and measured velocity the cascade above just used,
     * plus a small self-limiting dither while still converging (see RLS.h's
     * "Torque dither" section) -- RlsGetDitherTorque() returns exactly 0
     * once converged or not running, so this is always safe to add in.
     * Re-clamped afterward since dither can push the command slightly past
     * the configured torque limits. RLS.c isn't linked at all without
     * ANALOG_SERVO_HAS_AUTOTUNE (see platformio.ini) -- no observer, no
     * dither, same as F103's degenerate AppRls*() stand-ins above. */
    torqueCommand += RlsGetDitherTorque();
    torqueCommand = ClampToTorqueRange(torqueCommand);
    RlsTick(torqueCommand, velocityActual);
#endif
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
#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
                               !sCommLearningActive && !sPoleLearnActive &&
#endif
                               !AppAutotuneBusy() && !SignalIsActive()) ||
                              (SignalIsActive() && SignalGetTarget() == SIGNAL_TARGET_POSITION);
  int32_t positionError = useRealPositionError ? ControlGetLastPositionError() : 0;
  /* FaultTick needs axis-coordinate torque, before the learned mapping to
   * the physical DAC sign. */
  int32_t axisTorque = torqueCommand;
  if (sConfig.polarityLearned) axisTorque *= sConfig.polaritySign;
  FaultSetPolarityCheckSuppressed(sLearningActive
#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
                                  || sCommLearningActive || sPoleLearnActive
#endif
  );
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
#ifdef ANALOG_SERVO_HAS_AUTOTUNE
    AutotuneStop();   /* ditto for an in-progress autotune */
#endif
    OutputForceSafe();
    IndicatorSetState(INDICATOR_FAULT);
    sLastTorqueCommand = 0;
  }
#ifdef ANALOG_SERVO_TWO_PHASE_DRIVE
  else if (sOutputTopology == OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG) {
    /* Everything above this point computed torqueCommand exactly the same
     * way regardless of topology -- see docs/two-phase-drive.md. Only the
     * final conversion to an electrical angle + Ia/Ib differs. Pre-Z-index
     * (or no valid Hall reading): coarse Hall-sector angle, NOT
     * offset-corrected during sCommLearningActive/sPoleLearnActive (that's
     * what's being measured). Post-Z: continuous encoder-derived angle,
     * offset-corrected. */
    int32_t angleDeg;
    if (sCommLearningActive || sPoleLearnActive || !EncoderZWasSeenSinceEnable()) {
      int sectorForAngle = (hallSector >= 0) ? hallSector :
                            (sLastHallSectorSeen >= 0 ? sLastHallSectorSeen : 0);
      angleDeg = CommutationElectricalAngleFromHallDeg(sectorForAngle);
    } else {
      angleDeg = CommutationElectricalAngleFromEncoderDeg(
          EncoderGetPositionCounts(), sConfig.commutationCountsPerElecRev,
          sConfig.commutationOffsetDeg);
    }
    sLastElectricalAngleDeg = angleDeg;
    int32_t ia, ib;
    CommutationComputePhases(torqueCommand, angleDeg, &ia, &ib);
    OutputSetTwoPhaseCommand(ia, ib);
    IndicatorSetState(sEnabled ? INDICATOR_RUNNING : INDICATOR_IDLE);
    sLastTorqueCommand = torqueCommand;
  }
#endif
  else {
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
int32_t AppGetLastElectricalAngleDeg(void) { return sLastElectricalAngleDeg; }
int32_t AppGetLastHallSector(void) { return sLastHallSectorForTelemetry; }

uint32_t AppGetLastTickCycles(void) { return sLastTickCycles; }
uint32_t AppGetMaxTickCycles(void) { return sMaxTickCycles; }

/* Bounds of the .note.gnu.build-id output section, from ld/STM32*_FLASH.ld. */
extern const uint8_t __build_id_start[], __build_id_end[];

uint32_t AppGetBuildId(const uint8_t **id) {
  return BuildIdFromNote(__build_id_start, (uint32_t)(__build_id_end - __build_id_start), id);
}

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
  sOutputTopology = (OutputTopology)sConfig.outputTopology;
  OutputSetTopology(sOutputTopology);
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
#ifdef ANALOG_SERVO_HAS_AUTOTUNE
  AutotuneInit();
#endif
  IndicatorInit();
  ProtocolInit();
  MonitorInit();
  UsbCompositeCdcInit();   /* channel 0: CLI (Protocol.c); channel 1: binary
                               telemetry (Monitor.c) -- see docs/monitor-protocol.md */
#ifdef ANALOG_SERVO_HAS_UART
  UsartInit();
#endif

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
