#ifndef MAIN_H
#define MAIN_H

#if defined(STM32L152xE)
#include "stm32l1xx_hal.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_hal.h"
#elif defined(ANALOG_SERVO_NATIVE_TEST)
#include "stm32l1xx_hal.h"   /* host tests build against the L1-shaped mock
                               * in test/mocks/, regardless of target board */
#else
#error "main.h: no known MCU family macro defined -- see platformio.ini's build_flags (STM32L152xE / STM32F103xB / ANALOG_SERVO_NATIVE_TEST)"
#endif
#include "ConfigStore.h"
#include "Fault.h"
#include "Signal.h"
#include "Autotune.h"
#include "RLS.h"

void Error_Handler(void);
void SystemClock_Config(void);

/* Top-level coordinator, owned by main.c: mode dispatch (Mode A step/dir vs
 * Mode B velocity-command) spans StepDir/Encoder/Ramp/Control/ConfigStore --
 * no single one of those modules is the natural owner, so it lives here. */
typedef enum {
  SERVO_MODE_STEPDIR = 0,
  SERVO_MODE_VELOCITY = 1,
  SERVO_MODE_ANALOG = 2,
} ServoMode;

ServoMode AppGetServoMode(void);
/* In-RAM only on success -- `config save` persists it. */
int       AppSetServoMode(ServoMode mode);

int  AppGetEnabled(void);
void AppSetEnabled(int enabled);   /* enabling clears Control.c's integrator; does NOT
                                       auto-clear a latched fault */

/* Mode B (velocity command) input -- host-commanded velocity target, ramped
 * by Ramp.c before feeding the shared Control.c cascade. Counts/tick. Also
 * read back (not written) while in Mode C -- AppControlTick() overwrites
 * this every tick with the ADC-derived value in that mode. */
int32_t AppGetCommandedVelocity(void);
void    AppSetCommandedVelocity(int32_t countsPerTick);

/* Mode B (velocity command) alternative: a point-to-point ramped move to
 * an absolute position (encoder counts, same scale as position.raw),
 * instead of a held velocity -- see PositionMove.h. Only takes effect
 * while input.source=host. */
int32_t AppGetPositionCmd(void);
/* Pulse action: 0 success, -2 requires disabled/stationary/no active test. */
int AppResetPosition(void);
void    AppSetPositionCmd(int32_t targetPositionCounts);

/* Mode C (analog velocity) config -- scale/hysteresis applied to the ADC
 * input, see Analog.h. */
void    AppSetAnalogVelocityConfig(int32_t scaleQ8, int32_t hysteresisCounts);
int32_t AppGetAnalogRawAdc(void);
int     AppGetAnalogReversed(void);

/* Hardware ENABLE pin (see gpio.h's HW_ENABLE_PIN) -- snapshotted once per
 * control tick. Deasserted ramps the commanded velocity to zero (Mode
 * B/C) or freezes the step-derived target (Mode A) while the control loop
 * keeps running, rather than cutting torque outright. Independent of
 * axis.enabled (AppSetEnabled): both must be true for the servo to
 * actually track a target. */
int     AppGetHwEnableAsserted(void);

/* Fault "freeze frame" (OBD-II's term for the same idea): a full snapshot
 * of what was going on at the exact tick a fault LATCHED (the 0->1 edge
 * only). Deliberately survives FaultClear() -- only overwritten by the
 * NEXT fault's own latch edge. RAM-only: lost on reboot. See Protocol.c's
 * `fault freeze`. */
typedef struct {
  uint32_t    tick;
  FaultReason reason;
  int32_t     position;
  int32_t     velocity;
  int32_t     torqueCommand;
  int32_t     commandedPosition;
  int32_t     commandedVelocity;
  ServoMode   mode;
  uint8_t     enabled;
  uint8_t     hwEnableAsserted;
  uint8_t     idleReduced;         /* reserved, always 0 */
  uint8_t     synthetic;
  uint8_t     cycleStart;
  int32_t     posGainQ8, velGainKpQ8, velGainKiQ8;
  int32_t     velocityClamp;
  int32_t     torqueClampMin, torqueClampMax;
} FaultFreezeFrame;

int                     AppFaultFreezeExists(void);   /* 0 until the first fault ever
                                                           latches this boot */
const FaultFreezeFrame *AppGetFaultFreeze(void);

/* In-RAM working config -- gains/clamps/following-error/mode/polarity. */
const ConfigData *AppGetConfig(void);
void AppSetGains(int32_t posGainQ8, int32_t velGainKpQ8, int32_t velGainKiQ8);
void AppSetTorqueClamp(int32_t clampMin, int32_t clampMax);
void AppSetVelocityClamp(int32_t clamp);
void AppSetCommandVelocityClamp(int32_t clamp);
void AppSetVelocityFilterShift(int32_t shift);
void AppSetVelocityErrorDeadband(int32_t deadbandCounts);
void AppSetAccelerationFeedforward(int32_t gain);
void AppSetPositionMoveFeedforward(int32_t gain);
void AppSetVelocityJerk(int32_t jerk);
void AppSetRampAccel(int32_t accelCountsPerTickPerSec);
void AppSetFollowingErrorConfig(int32_t limitCounts, int hardFault);
/* FAULT_SUSTAINED_CURRENT config -- see Fault.h. minTorque <= 0 disables
 * the check entirely. */
void AppSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks);
void AppSetZeroTrim(int32_t trimTorqueUnits);   /* see Output.h's OutputSetZeroTrim */

/* Config record persistence. */
int  AppConfigSave(void);      /* 0 ok, -1 flash write failed */
int  AppConfigLoad(void);      /* 0 ok (also applies gains/clamps to Control.c/Fault.c), -1 no valid record */
void AppConfigDefaults(void);  /* reset in-RAM config to firmware defaults, NOT persisted */

/* One-time bench/commissioning step: commands a brief known test torque
 * (bypassing the normal control cascade), observes the encoder's velocity
 * sign, and records it as the learned polarity -- Fault.c's continuous
 * polarity check flags a disagreement with this value. Non-blocking.
 *
 * If the observed movement is too small to trust, the attempt is rejected:
 * polarityLearned/polaritySign are left untouched and
 * AppGetPolarityLearnFailed() reports 1. See docs/primer/why-faults-matter.md
 * for why this check exists. */
void AppPolarityLearn(void);
int  AppPolarityLearnBusy(void);
int  AppGetPolarityLearnFailed(void);
int32_t AppGetPolarityLearnMovedCounts(void);   /* diagnostic: raw counts moved during the last attempt */

/* Relay-feedback velocity-loop autotune (Autotune.c). Refused (no-op) while
 * a hard fault is latched. Doesn't auto-apply its result; AppAutotuneApply()
 * commits the last AUTOTUNE_RESULT_OK result into position.kp/velocity.kp/
 * velocity.ki as an explicit follow-up step. */
void    AppAutotuneStartRelay(int32_t amplitude, uint32_t maxTicks);
void    AppAutotuneStop(void);
int     AppAutotuneBusy(void);
AutotuneMethod AppAutotuneGetMethod(void);
AutotuneResult AppAutotuneGetResult(void);
int32_t  AppAutotuneGetKpResultQ8(void);
int32_t  AppAutotuneGetKiResultQ8(void);
int32_t  AppAutotuneGetPosPResultQ8(void);
uint32_t AppAutotuneGetMeasuredPeriodTicks(void);
int32_t  AppAutotuneGetMeasuredGainQ8(void);
void     AppAutotuneApply(void);

uint32_t AppAutotuneGetRelayProgressSeq(void);
uint32_t AppAutotuneGetRelayProgressTicksElapsed(void);
uint32_t AppAutotuneGetRelayProgressMaxTicks(void);
uint32_t AppAutotuneGetRelayProgressSwitchCount(void);
uint32_t AppAutotuneGetRelayProgressSwitchesNeeded(void);
int32_t  AppAutotuneGetRelayProgressPeakVelocity(void);

/* Online RLS velocity-loop self-tuning (RLS.c) -- the second autotune
 * method, running as a passive observer alongside the normal cascade
 * rather than relay's cascade-bypassing exclusive mode (see RLS.h).
 * Reachable via Protocol.c's `autotune start rls`/`stop`/`apply`, same
 * surface as relay. glideMs is real milliseconds, stashed verbatim by
 * AppRlsStart(); AppRlsApply() converts it to ticks using the live control
 * tick rate at apply time and commits the last AUTOTUNE_RESULT_OK
 * candidate's Kp/Ki into the live velocity gains via a GainGlide.c ramp
 * (position.kp untouched -- RLS only ever estimates the velocity loop),
 * rather than AppAutotuneApply()'s instant jump: a candidate can arrive
 * mid-move here, so an instant jump would step torque gains audibly/
 * mechanically while already tracking. */
void    AppRlsStart(int32_t lambdaQ8, uint32_t glideMs);
void    AppRlsStop(void);
int     AppRlsBusy(void);
AutotuneResult AppRlsGetResult(void);
int32_t  AppRlsGetKpResultQ8(void);
int32_t  AppRlsGetKiResultQ8(void);
int32_t  AppRlsGetThetaAQ8(void);
int32_t  AppRlsGetThetaBQ8(void);
int64_t  AppRlsGetPTrace(void);
uint32_t AppRlsGetTicksObserved(void);
uint32_t AppRlsGetProgressSeq(void);
void     AppRlsApply(void);

/* Opt-in pole-placement tuning knob (RLS.c's RlsSetPoleAggressiveness) --
 * reach for this when the plant's own open-loop pole estimate
 * (autotune.rls_theta_a) sits close to 1 and the default a^2 target
 * converges but tracks noticeably weaker than a known-good hand tune; see
 * RLS.h's own doc comment for the full rationale and the measured sweet
 * spot. Persists across AppRlsStart() (only RlsInit(), at boot, resets it
 * to 0) -- a property of the plant, not a per-attempt value, so a host
 * sets it once via `set autotune.rls_aggressiveness <n>` and leaves it. */
void    AppRlsSetAggressiveness(int32_t extraSquarings);
int32_t AppRlsGetAggressiveness(void);

/* Built-in test-signal generator (Signal.c). Refused (no-op) while a hard
 * fault is latched; a fault that latches mid-signal stops it. */
void       AppSignalStart(SignalType type, SignalTarget target, int32_t amplitude,
                           uint32_t halfPeriodTicks, uint32_t durationTicks);
void       AppSignalStop(void);
int        AppGetSignalActive(void);
SignalType   AppGetSignalType(void);
SignalTarget AppGetSignalTarget(void);
int        AppGetSignalCycleStart(void);

/* Runs one 1kHz control tick: mode dispatch (Mode A/B) -> Control.c ->
 * polarity correction -> Fault.c -> Output.c. Called from TIM6's update
 * ISR (tim_l1.c/stm32l1xx_it.c). */
void AppControlTick(void);

/* One coherent control-tick snapshot; flags: enabled=1, fault=2,
 * hardware-enable=4, following-error-valid=8. */
typedef struct {
  uint32_t tick;
  int32_t position, velocity, torque, followingError;
  int32_t positionReference, velocityReference, velocityFiltered;
  int32_t velocityError, velocityControlError;
  uint32_t flags;
} AppControlSample;
void AppGetControlSample(AppControlSample *out);

/* Telemetry snapshots from the last control tick, for Protocol.c. */
int32_t AppGetLastVelocityActual(void);
int32_t AppGetLastTorqueCommand(void);
int32_t AppGetLastCommandedPosition(void);
int32_t AppGetLastCommandedVelocity(void);

/* 1kHz control-tick CPU headroom -- software-side version of what
 * TICK_ACTIVE_PIN (gpio.h) lets a logic analyzer measure externally. */
uint32_t AppGetLastTickCycles(void);   /* whole-ISR DWT cycles, last tick */
uint32_t AppGetMaxTickCycles(void);    /* peak since boot */
int32_t  AppGetLastTickLoadPct(void);  /* integer percent of the 1kHz period */

#endif /* MAIN_H */
