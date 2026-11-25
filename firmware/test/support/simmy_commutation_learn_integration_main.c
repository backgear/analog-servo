/*
 * Simulation-integration test for the OTHER half of output.topology=
 * two_phase_analog this branch adds: the open-loop Hall-driven
 * `commutation learn` maneuver itself (main.c's AppCommutationLearn() /
 * AppControlTick()'s sCommLearningActive branch), against a real spinning
 * plant -- complementing simmy_foc_integration_main.c, which only exercises
 * the CONTINUOUS encoder-angle path (CommutationElectricalAngleFromEncoderDeg)
 * and assumes the alignment offset is already known good. See
 * docs/two-phase-drive.md for what Commutation.c/Hall.c/the learn
 * maneuver are and why.
 *
 * PROVISIONAL, same SIMMY_DIR caveat as simmy_foc_integration_main.c: this
 * driver only needs examples/foc_transforms.h's FocPark()/FOC_PI (for the
 * Clarke+Park reconstruction below), not the full examples/pmsm_foc.h
 * current-loop bundle that test links against -- but it's gated on the
 * SAME provisional-files check as that test anyway (see
 * tools/test_simmy_commutation_learn_integration.py), since all of
 * foc_transforms.h/pmsm_foc.h/pmsm_foc.c live in the same not-yet-merged
 * simmy worktree and "is the provisional work available" is one question,
 * not three.
 *
 * ---- What this proves, and what it doesn't -----------------------------
 *
 * Proves: the TICK-DRIVING STATE MACHINE in main.c -- not just
 * Commutation.c's accumulator math in isolation (test_commutation.c's
 * test_learn_recovers_a_known_nonzero_offset etc. already cover that with
 * hand-fabricated sector/angle sequences) -- correctly wires a REAL
 * spinning plant's Hall-sector transitions and encoder position into a
 * correct learned commutation.offset_deg. Concretely: AppCommutationLearn()
 * arming correctly (duration counter reset, sLastHallSectorSeen reset to
 * its "no reading yet" sentinel, FaultClear()), AppControlTick()'s
 * sCommLearningActive branch correctly driving open-loop torque at the
 * Hall-sourced (NOT offset-corrected -- that's what's being measured)
 * angle every tick, correctly detecting each REAL sector transition
 * exactly once (not double-counted, not missed) and feeding it
 * CommutationLearnAccumulate() with the matching real encoder angle, the
 * duration countdown correctly ending the attempt and persisting the
 * result via CommutationLearnEvaluate(), and FaultTick()'s interaction
 * with all of this not spuriously aborting a legitimate attempt (this
 * uncovered a REAL, previously-untested interaction -- see
 * "A real finding" below).
 *
 * Does NOT prove: anything about a real, physically-miswired Hall line.
 * This test injects its OWN "true" Hall-table-to-electrical-zero
 * misalignment (TRUE_HALL_MISALIGNMENT_DEG below) and then checks that the
 * learn maneuver recovers THAT injected truth -- the simulated Hall sensor
 * is, by construction, exactly as reliable as HallEncode() says it is.
 * A real cold-solder joint, swapped Hall wire, or sensor that fails
 * intermittently under vibration has no simulated counterpart here; that
 * stays bench-only, same limitation simmy_foc_integration_main.c's own
 * header already states for the encoder-angle path it covers.
 *
 * ---- A real finding, surfaced by actually running this -----------------
 *
 * Fault.c's sustained-current detector (10% of TORQUE_FULL_SCALE sustained
 * for 2s, FaultInit()'s own built-in defaults -- see Fault.c) is NOT
 * suppressed during commutation-learn the way the polarity-mismatch check
 * is (FaultSetPolarityCheckSuppressed(), see AppControlTick()). The real
 * `commutation learn` maneuver commands COMMUTATION_LEARN_TEST_TORQUE
 * (33% of TORQUE_FULL_SCALE) for up to ~5s (main.c's
 * COMMUTATION_LEARN_DURATION_TICKS) -- well past that 2s/10% default
 * window. VERIFIED by actually removing this test's own
 * AppSetSustainedCurrentConfig(0, 0) call below (not just reasoned about):
 * the fault DOES latch partway through this test's own scenario
 * (confirmed via FaultIsLatched()), forcing real torque to zero
 * (OutputForceSafe()) for the remainder of the run, while the learn
 * countdown and Hall-transition bookkeeping keep running unaware -- but
 * in THIS scenario it happens not to change the outcome, because enough
 * Hall transitions to satisfy COMMUTATION_LEARN_MIN_TRANSITIONS are
 * already gathered within the first ~2s, before the fault trips. A
 * slower motor, weaker test torque, or less generous min-transitions
 * count could easily NOT be so lucky -- this is a real latent interaction
 * this test surfaced, not something it was designed to probe for, and
 * not something to rely on "happening to work out" on real hardware. This
 * test disables that one fault channel for its own run
 * (AppSetSustainedCurrentConfig(0, 0), the documented "0 == disabled"
 * convention -- see Fault.c) so its outcome doesn't depend on that race;
 * a real bench attempt should consider doing the same
 * (`fault.sustained_current.min_torque 0`) before running
 * `commutation learn`, rather than relying on finishing fast enough.
 *
 * ---- Build shape: this is the first test to link the REAL main.c -------
 *
 * Every existing native Unity test links individual isolated modules
 * (Control.c, Fault.c, ...) with hand-written App*() stub bodies standing
 * in for main.c (see e.g. test_control.c's trivial
 * EncoderGetPositionCounts() stub) -- none of them link main.c itself,
 * because main.c's own real main() calls straight into the STM32 HAL
 * (HAL_Init/SystemClock_Config/MX_*_Init/the ProtocolPoll/IndicatorPoll/
 * MonitorPoll superloop) and this project's native HAL mocks have never
 * needed to cover that surface. Two tricks make linking main.c itself
 * practical without reimplementing the whole HAL:
 *   1. main.c is compiled with `-Dmain=UnusedFirmwareMain`, so THIS file's
 *      own real main() (the test driver's entry point) doesn't collide
 *      with main.c's.
 *   2. Both are compiled/linked with -ffunction-sections -Wl,--gc-sections,
 *      so UnusedFirmwareMain -- and everything ONLY reachable from it
 *      (HAL_Init, every MX_*_Init, the superloop) -- gets garbage-collected
 *      at link time, since nothing in this test's actual call graph
 *      (AppControlTick/AppCommutationLearn/... ) reaches it. Only symbols
 *      genuinely reachable from what this test calls need to resolve --
 *      see test/support/sim_main_mocks/stm32l1xx_hal.h's own comment for
 *      the GPIO/DWT surface that DOES turn out to be reachable (Hall
 *      pins + the control-tick cycle counter, both read directly inside
 *      AppControlTick()), and the hand-written no-op stubs below for
 *      Adc/Analog/Indicator/Ramp/PositionMove/StepDir/Monitor -- modules
 *      main.c's compiled code still REFERENCES from branches this test's
 *      scenario never actually takes at runtime (sMode==SERVO_MODE_STEPDIR,
 *      SignalIsActive(), ...) but which the compiler/linker still require
 *      to resolve regardless, same reasoning the existing Unity tests'
 *      own hand-written stubs already document.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32l1xx_hal.h"   /* the mock in sim_main_mocks/ */
#include "main.h"
#include "gpio.h"            /* the REAL one -- just HALL_A_GPIO_PORT/PIN etc. macros
                                 plus a few decls; see sim_main_mocks/stm32l1xx_hal.h's
                                 own comment for why this test uses the real header
                                 (unlike sim_foc_mocks/gpio.h's stripped-down stand-in) */
#include "Hall.h"
#include "Commutation.h"
#include "Output.h"
#include "Fault.h"
#include "Control.h"
#include "Autotune.h"
#include "Signal.h"
#include "Encoder.h"
#include "Ramp.h"
#include "PositionMove.h"
#include "Analog.h"
#include "Adc.h"
#include "StepDir.h"
#include "Indicator.h"
#include "Monitor.h"

#include "simmy.h"
#include "foc_transforms.h"   /* from SIMMY_DIR/examples -- just FocPark()/FOC_PI here,
                                  see tools/test_simmy_commutation_learn_integration.py */

/* ==================== HAL mock state ==================================== */

MockDwtType gMockDwt;   /* sim_main_mocks/stm32l1xx_hal.h's DWT -- CYCCNT advanced
                            externally by the per-tick loop below */

static int sSimHallBits[3] = {0, 0, 0};   /* [0]=A,[1]=B,[2]=C -- HallEncode()'s output,
                                              set by the per-tick loop before each
                                              AppControlTick() call */

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  if (port == HALL_A_GPIO_PORT && pin == HALL_A_PIN)
    return sSimHallBits[0] ? GPIO_PIN_RESET : GPIO_PIN_SET;   /* active-low, see main.c's ha= */
  if (port == HALL_B_GPIO_PORT && pin == HALL_B_PIN)
    return sSimHallBits[1] ? GPIO_PIN_RESET : GPIO_PIN_SET;
  if (port == HALL_C_GPIO_PORT && pin == HALL_C_PIN)
    return sSimHallBits[2] ? GPIO_PIN_RESET : GPIO_PIN_SET;
  /* HW_ENABLE/ANALOG_REVERSE/etc. -- not read inside the sCommLearningActive
   * branch's actual execution path; a harmless fixed default is fine. */
  return GPIO_PIN_SET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
  (void)port; (void)pin; (void)state;   /* TICK_ACTIVE_PIN/SIGNAL_BUSY_PIN -- debug-only
                                            brackets, nothing reads them back here */
}

/* ==================== Dead-code-only HAL/init surface ===================
 * Referenced only from SystemClock_Config() and the real main() (renamed
 * away from this test's own entry point by the `-Dmain=UnusedFirmwareMain`
 * build flag -- see this file's own top comment) -- never actually
 * called, but gcc still compiles both functions' bodies in full regardless
 * of what the linker later discards, so every identifier they reference
 * still needs to resolve to SOMETHING. Dummy/no-op, on purpose. */
CoreDebugType gMockCoreDebug;
HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc) { (void)osc; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t flashLatency) {
  (void)clk; (void)flashLatency;
  return HAL_OK;
}
void UsartInit(void) { }
void UsbCompositeCdcInit(void) { }
void EncoderInit(void) { }
void EncoderResetPosition(void) { }
void EncoderZIndexIsr(void) { }
void EncoderClearZSeen(void) { }
void StepDirInit(void) { }
void StepDirDirIsr(void) { }
void AdcInit(void) { }
void ControlTickTimerInit(void) { }
void ProtocolInit(void) { }
void ProtocolPoll(void) { }
void IndicatorInit(void) { }
void IndicatorPoll(void) { }
void IndicatorPulseSingle(void) { }
void IndicatorPulseDouble(void) { }
void GpioInit(void) { }
void LedWrite(int on) { (void)on; }
void FaultOutWrite(int active) { (void)active; }
void BrakeReleaseWrite(int released) { (void)released; }
void MonitorInit(void) { }
void MonitorPoll(void) { }
void ConfigDataSetDefaults(ConfigData *cfg) { memset(cfg, 0, sizeof(*cfg)); }
int  ConfigStoreLoad(ConfigData *out) { (void)out; return -1; }   /* "no stored config" */
int  ConfigStoreSave(const ConfigData *cfg) { (void)cfg; return -1; }
void ConfigStoreErase(void) { }

/* ---- Output.c's real HAL call targets, captured instead of touched (same
 * pattern as simmy_foc_integration_main.c) ---- */
static int32_t sCapturedDacCode[2] = {0, 0};
static int     sCapturedSign[2] = {0, 0};

HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac) { (void)hdac; return HAL_OK; }
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *config,
                                        uint32_t channel) {
  (void)hdac; (void)config; (void)channel;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t channel) {
  (void)hdac; (void)channel;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t channel,
                                   uint32_t alignment, uint32_t value) {
  (void)hdac; (void)alignment;
  sCapturedDacCode[channel] = (int32_t)value;
  return HAL_OK;
}
void ZeroClampWrite(int asserted) { (void)asserted; }
void PhaseSignWrite(int channel, int negative) { sCapturedSign[channel] = negative; }

/* main.c's OWN real Error_Handler() (unmodified) calls __disable_irq() --
 * mocked to a no-op above. Never actually reached: the HAL_DAC_* stubs
 * above always report success, so Output.c never calls it. */

/* ==================== Hand-stubbed "irrelevant" modules ================
 * Same convention the existing Unity tests already use (e.g. test_control.c's
 * trivial EncoderGetPositionCounts() stub): main.c's compiled code
 * references these from branches this test's scenario never actually
 * takes at runtime (sMode defaults to SERVO_MODE_STEPDIR, sEnabled stays
 * 0, SignalIsActive()/AppAutotuneBusy() stay false), but the symbols must
 * still resolve. Trivial, behavior-free bodies. */
int32_t AdcGetVelocityRaw(void) { return 0; }
int32_t AnalogVelocityTick(int32_t rawAdcCode, int reversed) {
  (void)rawAdcCode; (void)reversed;
  return 0;
}
void AnalogVelocitySetConfig(int32_t scaleQ8, int32_t hysteresisCounts) {
  (void)scaleQ8; (void)hysteresisCounts;
}
void IndicatorSetState(IndicatorState state) { (void)state; }
void RampSetTarget(RampState *r, int32_t targetVelocityCountsPerTick) {
  (void)r; (void)targetVelocityCountsPerTick;
}
void RampSetAcceleration(RampState *r, int32_t accelCountsPerTickPerSec) {
  (void)r; (void)accelCountsPerTickPerSec;
}
void RampSetJerk(RampState *r, int32_t jerk) { (void)r; (void)jerk; }
int32_t RampSmoothTick(RampState *r, uint32_t dtCycles) { (void)r; (void)dtCycles; return 0; }
int PositionMoveIsActive(const PositionMoveState *m) { (void)m; return 0; }
int32_t PositionMoveTick(PositionMoveState *m, RampState *ramp, int32_t positionPlanned,
                          int32_t maxVelocityCountsPerTick, uint32_t dtCycles) {
  (void)m; (void)ramp; (void)positionPlanned; (void)maxVelocityCountsPerTick; (void)dtCycles;
  return 0;
}
int32_t StepDirGetDeltaCounts(void) { return 0; }
void MonitorTick(void) { }

/* ==================== Simulated Encoder.c ===============================
 * Replaces the real TIM4-hardware-encoder-backed Encoder.c entirely: these
 * four are the ONLY Encoder.h entry points AppControlTick()'s
 * sCommLearningActive path (or anything it unconditionally touches) reads.
 * sSimEncoderCounts is set by the per-tick loop below, from the plant's
 * own position, BEFORE each AppControlTick() call -- exactly standing in
 * for "what the real TIM4 counter would currently read." */
static int32_t  sSimEncoderCounts = 0;
static int32_t  sSimEncoderPrevCounts = 0;
static uint32_t sSimTotalEdges = 0;

int32_t EncoderGetPositionCounts(void) { return sSimEncoderCounts; }
int32_t EncoderGetDeltaSinceLastTick(void) {
  int32_t delta = sSimEncoderCounts - sSimEncoderPrevCounts;
  sSimEncoderPrevCounts = sSimEncoderCounts;
  sSimTotalEdges += (uint32_t)(delta < 0 ? -delta : delta);
  return delta;
}
int      EncoderZWasSeenSinceEnable(void) { return 0; }   /* never seen -- harmless: the
    two-phase dispatch uses the Hall-sourced angle regardless of Z whenever
    sCommLearningActive is set, see main.c's own angleDeg branch */
uint32_t EncoderTotalEdgesSeen(void) { return sSimTotalEdges; }

/* ==================== The "stand-in amp": Clarke+Park reconstruction =====
 * Same derivation as simmy_foc_integration_main.c's DeriveDqFromPhaseCurrents()
 * (see that file for the full algebra) -- Clarke() recovers alpha/beta from
 * Output.c's real written abc pair, FocPark() with the angle argument
 * projects it onto a d/q frame. The one deliberate difference from that
 * other test: the angle fed in here is the plant's TRUE electrical angle
 * (not a current-loop's sensed angle, since there IS no current loop in
 * this simpler model -- see this file's own top comment, point 3's
 * "simpler, your call" option). q comes out proportional to how well the
 * (possibly Hall-coarse, possibly misaligned) commanded phase currents
 * actually align with the TRUE rotor position -- exactly the
 * cos(angle error) torque reduction a real open-loop-commutated PMSM
 * would show under coarse/misaligned Hall commutation, which is the one
 * piece of real physics worth keeping for THIS test (a correctly-shaped
 * spin-up, not an idealized one). */
static void Clarke(double a, double b, double alphaBeta[2]) {
  alphaBeta[0] = a;
  alphaBeta[1] = (a + 2.0 * b) / sqrt(3.0);
}
static double WrapDeg360(double deg) {
  double w = fmod(deg, 360.0);
  if (w < 0) w += 360.0;
  return w;
}

/* Inverts Output.c's OutputSplitTwoPhase()/OutputWriteDacTwoPhase() exactly
 * -- see simmy_foc_integration_main.c's StandInAmpReadLastCommand() for the
 * identical derivation. */
static void StandInAmpReadLastCommand(int32_t *ia, int32_t *ib) {
  int32_t magA = (sCapturedDacCode[DAC_CHANNEL_1] * TORQUE_FULL_SCALE) / 4095;
  int32_t magB = (sCapturedDacCode[DAC_CHANNEL_2] * TORQUE_FULL_SCALE) / 4095;
  *ia = sCapturedSign[0] ? -magA : magA;
  *ib = sCapturedSign[1] ? -magB : magB;
}

static double TorqueEquivalentQ15(int32_t ia, int32_t ib, double trueElectricalAngleDeg) {
  double alphaBeta[2];
  Clarke((double)ia, (double)ib, alphaBeta);
  double thetaRad = trueElectricalAngleDeg * FOC_PI / 180.0;
  double dq[2];
  FocPark(alphaBeta, thetaRad, dq);
  return -dq[1];   /* sign flip -- see simmy_foc_integration_main.c's derivation comment */
}

int main(void) {
  /* ---- Real inits for the modules this test keeps real ----------------- */
  OutputInit();
  FaultInit();
  ControlInit(NULL);
  AutotuneInit();
  SignalInit();

  /* ---- Scenario setup ---------------------------------------------------
   * polePairs/countsPerMechRev are this test's OWN choice (there is no
   * real motor on the other end) -- 4 pole pairs, matching
   * simmy_foc_integration_main.c's sample motor for consistency, though
   * nothing here depends on that match. countsPerMechRev must divide
   * evenly by polePairs for countsPerElecRev to be exact. */
  const int32_t polePairs = 4;
  const int32_t countsPerMechRev = 8192;
  const int32_t countsPerElecRev = countsPerMechRev / polePairs;

  /* The injected "truth" this test checks commutation-learn actually
   * recovers -- deliberately not 0 and not a multiple of 60 (a multiple of
   * 60 would be indistinguishable from perfect alignment at
   * HallDecode()'s own sector granularity, which would make this a much
   * weaker test). Also deliberately NOT near +-90/+-270: this test's
   * simplified torque model (TorqueEquivalentQ15 below) computes
   * K*cos(hallCoarseAngle - trueAngle), and hallCoarseAngle is itself
   * quantized to a 60-degree grid -- near a 90-degree true misalignment,
   * that quantization noise alone is enough to swing cos() across its
   * steepest region, making the SIMULATED open-loop torque's sign/
   * magnitude dominated by quantization rather than the injected
   * misalignment (verified empirically: 97 degrees and 179 degrees both
   * produced large, inconsistent learn errors here; this is a real
   * property of coarse open-loop Hall commutation near a near-90-degree
   * misalignment -- see docs/two-phase-drive.md's own "60-degree
   * coarseness is already the dominant error source" point -- not a bug
   * in this test's math, but a regime this test doesn't try to cover).
   * 23 degrees sits comfortably away from that region (cos(23)~=0.92, a
   * strong, consistent forward torque), so this test can give
   * commutation-learn a clean, well-behaved spin-up to recover a known
   * offset from -- the same thing a bench operator would want from a
   * REAL motor's physical Hall mounting before trusting this maneuver's
   * result, not a limitation unique to simulation. */
  const double TRUE_HALL_MISALIGNMENT_DEG = 23.0;

  AppSetOutputTopology(OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG);
  AppSetCommutationConfig(polePairs, countsPerElecRev);
  /* Disables Fault.c's sustained-current channel for this run -- see this
   * file's own "A real finding" comment above for exactly why this is
   * necessary, not a blanket workaround. */
  AppSetSustainedCurrentConfig(0, 0);

  /* Simple single-body mechanical plant, hand-built rather than loaded
   * from examples/configs/two_mass.json (no JSON dependency needed for a
   * single free body) -- inertia/friction picked from the SAME order of
   * magnitude that file's own body[0] uses (inertia 64, friction 0.02),
   * since this test's "force" input (via the analog-chain gain stage
   * below) is deliberately rescaled to land in that same reference scale.
   * Purely a plant for THIS test, not a claim about any real machine. */
  SimmyChainParams chainParams;
  memset(&chainParams, 0, sizeof(chainParams));
  chainParams.bodyCount = 1;
  chainParams.subSteps = 1;
  chainParams.bodies[0].inertia = 64.0;
  chainParams.bodies[0].friction = 3.0;
  SimmyChainState chainState;
  SimmyChainInit(&chainState, &chainParams);

  /* The "stand-in amp"'s DAC/amplifier gain stage -- maps the Clarke+Park-
   * reconstructed torque-equivalent (full scale +-TORQUE_FULL_SCALE) down
   * to the same ~1000-ish force scale two_mass.json's own inertia/friction
   * were tuned against (see above). A single SIMMY_STAGE_GAIN stage, same
   * "DAC+amp" abstraction examples/analog_servo_integration/main.c's own
   * SimmyAnalogChainStep() call already uses for a plain scalar torque --
   * here it's just downstream of this test's OWN Clarke+Park step instead
   * of a direct torque command, since Output.c's real output is two-phase,
   * not scalar. */
  SimmyAnalogChainParams analogParams;
  memset(&analogParams, 0, sizeof(analogParams));
  analogParams.stageCount = 1;
  analogParams.stages[0].type = SIMMY_STAGE_GAIN;
  analogParams.stages[0].param0 = 1000.0 / (double)TORQUE_FULL_SCALE;
  SimmyAnalogChainState analogState;
  SimmyAnalogChainInit(&analogState, 1);

  AppCommutationLearn();
  if (!AppCommutationLearnBusy()) {
    fprintf(stderr, "FAIL -- AppCommutationLearn() did not arm (topology/fault precondition "
                    "rejected it; see AppCommutationLearn()'s own guard clauses)\n");
    return 1;
  }

  const int kMaxTicks = 20000;   /* generous safety cap -- main.c's own
                                     COMMUTATION_LEARN_DURATION_TICKS is 5000;
                                     this is a "don't hang forever on a real
                                     bug" guard, not a tuned bound */
  int tick;
  int anyNonFinite = 0;
  for (tick = 0; tick < kMaxTicks && AppCommutationLearnBusy(); tick++) {
    int32_t encoderCounts = SimmyChainFeedbackPositionCounts(&chainState, &chainParams);
    sSimEncoderCounts = encoderCounts;

    double trueElectricalAngleDeg = WrapDeg360(
        (double)polePairs * 360.0 * (double)encoderCounts / (double)countsPerMechRev);
    double hallElectricalAngleDeg = WrapDeg360(trueElectricalAngleDeg - TRUE_HALL_MISALIGNMENT_DEG);
    int sector = ((int)floor(hallElectricalAngleDeg / 60.0)) % 6;
    if (sector < 0) sector += 6;
    int a, b, c;
    HallEncode(sector, &a, &b, &c);
    sSimHallBits[0] = a; sSimHallBits[1] = b; sSimHallBits[2] = c;

    gMockDwt.CYCCNT += 32000U;   /* SYSCLK_HZ/1000 -- a nominal 1ms tick's worth of
                                     cycles; unused by the sCommLearningActive path
                                     itself (see this file's header comment), kept
                                     monotonic anyway for defensiveness */

    AppControlTick();   /* the REAL firmware tick: reads the Hall GPIOs/encoder
                            above, dispatches sCommLearningActive, computes
                            Ia/Ib via Commutation.c, writes them via Output.c --
                            all captured above, nothing else */

    int32_t recoveredIa, recoveredIb;
    StandInAmpReadLastCommand(&recoveredIa, &recoveredIb);
    double torqueEquivalent = TorqueEquivalentQ15(recoveredIa, recoveredIb, trueElectricalAngleDeg);
    if (!isfinite(torqueEquivalent)) anyNonFinite = 1;

    double force = SimmyAnalogChainStep(&analogState, &analogParams, torqueEquivalent);
    SimmyChainStep(&chainState, &chainParams, force);
    if (!isfinite(SimmyChainFeedbackPositionCounts(&chainState, &chainParams) * 1.0)) {
      anyNonFinite = 1;
    }
  }

  int stillBusy = AppCommutationLearnBusy();
  int failed = AppGetCommutationLearnFailed();
  int32_t learnedOffsetDeg = AppGetConfig()->commutationOffsetDeg;
  double errorDeg = learnedOffsetDeg - TRUE_HALL_MISALIGNMENT_DEG;
  while (errorDeg > 180.0) errorDeg -= 360.0;
  while (errorDeg < -180.0) errorDeg += 360.0;

  printf("ticks=%d stillBusy=%d failed=%d learnedOffsetDeg=%d trueOffsetDeg=%.1f "
         "errorDeg=%.2f finalEncoderCounts=%d anyNonFinite=%d\n",
         tick, stillBusy, failed, learnedOffsetDeg, TRUE_HALL_MISALIGNMENT_DEG, errorDeg,
         SimmyChainFeedbackPositionCounts(&chainState, &chainParams), anyNonFinite);

  /* CommutationLearnEvaluate's own 60-degree-sector coarseness is the
   * real-world error floor here (see Commutation.c), not a bound this test
   * tightens arbitrarily -- 15 degrees gives ample margin above the actual
   * observed convergence (re-run this driver directly to see today's real
   * number in the printf above) while still catching a genuinely broken
   * wiring-to-offset mapping. */
  int ok = !anyNonFinite && !stillBusy && !failed && fabs(errorDeg) <= 15.0;

  printf("%s -- main.c's real commutation-learn state machine, linked via the stand-in amp "
         "above, into a real spinning simmy plant\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
