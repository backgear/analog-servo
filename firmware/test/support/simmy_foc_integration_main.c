/*
 * Simulation-integration test for output.topology=two_phase_analog: links
 * this branch's REAL Commutation.c and Output.c against simmy's PMSM FOC
 * example harness (examples/pmsm_foc.h/.c in the sibling simmy checkout,
 * SimmyPmStep + PmFoc) -- same "link the actual firmware code, not a demo
 * stand-in" spirit as simmy's own examples/analog_servo_integration/main.c,
 * which does this for Control.c's Mode A/B stepdir path. See
 * docs/two-phase-drive.md for what Commutation.c/Output.c are and why.
 *
 * PROVISIONAL: as of this writing, examples/pmsm_foc.h/.c/foc_transforms.h
 * are UNCOMMITTED in a temporary simmy worktree, not yet merged into
 * simmy's main branch -- see tools/test_simmy_foc_integration.py's own
 * comment and whatever SIMMY_DIR currently points at. Once that work lands
 * in simmy proper, SIMMY_DIR should go back to pointing at an ordinary
 * simmy checkout; nothing here needs to change for that, since this file
 * only ever reaches those headers through SIMMY_DIR's -I paths.
 *
 * ---- What this proves, and what it doesn't -----------------------------
 *
 * Proves: commanding a signed torque + the REAL encoder-derived electrical
 * angle (Commutation.c's CommutationElectricalAngleFromEncoderDeg(), "the
 * steady-state path that needs no Hall input at all" -- see
 * docs/two-phase-drive.md) through CommutationComputePhases() and then
 * through Output.c's REAL DAC-split/sign-GPIO write path
 * (OutputSetTwoPhaseCommand()) produces a two-phase command that, once
 * reconstructed on the OTHER side of that DAC/GPIO boundary and handed to
 * a PMSM current loop + real R-L-backEMF plant (simmy's SimmyPmStep), spins
 * the simulated motor up in the commanded direction with the commutation
 * staying correctly aligned (small Id, the torque-only MTPA assumption
 * both Commutation.c's sine synthesis and PmFoc's own controller share) --
 * not just that the fixed-point math type-checks.
 *
 * Does NOT prove: anything about a real AMC SX25A or any other specific
 * downstream amplifier's actual behavior, the CD4053 sign-switch hardware
 * (not built on any board -- see docs/two-phase-drive.md), Hall-sector
 * commutation during the pre-Z-index window (deliberately out of scope --
 * see "stand-in amp" note below), or anything about THIS board's own
 * DAC/op-amp analog stage (no HAL/ADC/DAC silicon runs host-side at all).
 *
 * ---- The "stand-in amp" --------------------------------------------------
 *
 * Nobody has built a two-phase-analog-to-PMSM-plant bridge before (there is
 * no real downstream amplifier on hand to characterize -- see
 * docs/two-phase-drive.md's "not built on any board" note), so its shape
 * here is a deliberate, documented choice, not a reference design:
 *
 *   1. HAL_DAC_SetValue()/PhaseSignWrite() below are Output.c's REAL call
 *      targets (via the mock stm32l1xx_hal.h/gpio.h in sim_foc_mocks/,
 *      shadowing the real HAL-coupled headers) -- they just record the two
 *      magnitude codes + two sign bits Output.c actually wrote, instead of
 *      touching silicon. StandInAmpReadLastCommand() below inverts
 *      Output.c's OutputSplitTwoPhase() exactly, recovering the signed
 *      Ia/Ib Output.c ACTUALLY wrote (full 12-bit DAC quantization
 *      included) -- not the pre-quantization ia/ib CommutationComputePhases()
 *      returned. This is the one part of the "amp" that's really just
 *      Output.c's own contract, not a model choice.
 *   2. Ia/Ib are then treated as the amp's commanded STATOR PHASE currents
 *      (abc-style, Ic inferred as -(Ia+Ib), same inference Output.c's own
 *      header comment describes the downstream drive performing) scaled by
 *      STAND_IN_AMP_CURRENT_SCALE: firmware's full-scale Q15 command maps
 *      onto the stand-in amp's own configured peak current -- same
 *      "digital full scale == configured hardware limit" convention a real
 *      amp's current-loop scaling would use, not a claim about any
 *      specific amp's actual scale factor.
 *   3. Clarke() (below -- the one transform examples/foc_transforms.h
 *      doesn't provide; Park/inverse-Park there assume an alpha/beta pair
 *      already) recovers alpha/beta from that abc triple, then
 *      examples/foc_transforms.h's FocPark() maps it into the amp's own
 *      d/q current reference -- using the PLANT's own TRUE electrical
 *      angle (polePairs*mechanicalAngle), deliberately NOT firmware's own
 *      CommutationElectricalAngleFromEncoderDeg() estimate (see the
 *      "IMPORTANT" comment right before main() for exactly why that
 *      distinction matters: using firmware's own angle here would be
 *      circular and could never catch a miscalibrated
 *      commutation.offset_deg). Id lands near 0 and Iq near the
 *      commanded torque WHEN firmware's estimate agrees with the plant's
 *      true angle -- see DeriveDqFromPhaseCurrents()'s comment for the
 *      exact algebra (d=K*sin(e), q=-K*cos(e), e = the misalignment
 *      between them). That is the one piece of real physics this stand-in
 *      amp claims: a no-Hall three-phase amp's whole job is closing a
 *      current loop in a frame that tracks the motor's ACTUAL electrical
 *      angle -- it does not claim anything about how fast or by what means
 *      (Hall interpolation, a resolver, back-EMF sensing...) any REAL such
 *      amp tracks that angle moment to moment.
 *   4. simmy's own PmFoc current-loop PI (PmFocCurrentStep(), already
 *      implemented and used by examples/pmsm_foc.c) then closes that d/q
 *      reference against the plant's measured current and drives
 *      SimmyPmStep() with real alpha/beta voltage -- i.e. everything AFTER
 *      step 3 is simmy's own existing, un-modified PMSM FOC reference
 *      harness, not anything invented here.
 *
 * Explicitly NOT modeled: any amplifier bandwidth/delay between Output.c's
 * DAC write and the "amp" reading it (this driver reads back the same tick
 * it was written, matching Output.c's own call being synchronous), the
 * CD4053 switch's own electrical behavior, and the pre-Z-index Hall-sourced
 * angle path (CommutationElectricalAngleFromHallDeg()) -- this test only
 * exercises the continuous encoder-angle path on purpose (see the big
 * comment above); Hall.c/Commutation.c's Hall-sector and alignment-learning
 * math already has its own direct native unit tests (test_hall.c/
 * test_commutation.c), ported from the two-phase-output branch.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32l1xx_hal.h"   /* the mock in sim_foc_mocks/ -- declares the DAC
                                 shapes this file's own HAL_DAC_*() definitions
                                 below implement */
#include "Commutation.h"
#include "Output.h"

#include "pmsm_foc.h"   /* from SIMMY_DIR/examples -- see tools/test_simmy_foc_integration.py */

/* ---- Output.c's real HAL call targets, captured instead of touched ----- */
static int32_t sCapturedDacCode[2] = {0, 0};   /* index == DAC_CHANNEL_1/2 */
static int     sCapturedSign[2] = {0, 0};      /* index == PhaseSignWrite's channel arg */

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
void Error_Handler(void) {
  fprintf(stderr, "Output.c called Error_Handler() -- aborting\n");
  exit(1);
}

/* Inverts Output.c's OutputSplitTwoPhase()/OutputWriteDacTwoPhase() exactly
 * (see Output.c) -- recovers the signed Ia/Ib (Q15, +-TORQUE_FULL_SCALE)
 * ACTUALLY written this tick, 12-bit DAC quantization included, not the
 * pre-quantization ia/ib CommutationComputePhases() returned. */
static void StandInAmpReadLastCommand(int32_t *ia, int32_t *ib) {
  int32_t magA = (sCapturedDacCode[DAC_CHANNEL_1] * TORQUE_FULL_SCALE) / 4095;
  int32_t magB = (sCapturedDacCode[DAC_CHANNEL_2] * TORQUE_FULL_SCALE) / 4095;
  *ia = sCapturedSign[0] ? -magA : magA;
  *ib = sCapturedSign[1] ? -magB : magB;
}

/* The one transform examples/foc_transforms.h doesn't provide: recovers
 * stationary alpha/beta from an abc triple (c inferred as -(a+b), same
 * balanced-three-phase assumption Output.c's own header comment describes
 * the downstream drive making). Amplitude-invariant, same convention as
 * FocPark/FocInversePark in that header. */
static void Clarke(double a, double b, double alphaBeta[2]) {
  alphaBeta[0] = a;
  alphaBeta[1] = (a + 2.0 * b) / sqrt(3.0);
}

/* Derivation (checked once here, not re-derived at runtime): Commutation.c
 * synthesizes ia = K*sin(theta), ib = K*sin(theta-120deg) for some torque-
 * proportional K and its own electricalAngleDeg theta. Clarke on that pair
 * gives alpha = K*sin(theta), beta = -K*cos(theta) (algebra: ia+2*ib =
 * K*sin(theta) + 2*K*sin(theta-120) = -sqrt(3)*K*cos(theta), divided by
 * sqrt(3)). FocPark([alpha,beta], theta) = [cos*alpha+sin*beta,
 * -sin*alpha+cos*beta] = [K*sin*cos - K*cos*sin, -K*sin^2 - K*cos^2] =
 * [0, -K] for EVERY theta, not just a special case. So: feeding Clarke's
 * output through FocPark with the SAME theta Commutation.c used always
 * lands on d==0 exactly (modulo the real quantization this driver's
 * StandInAmpReadLastCommand() introduces) and q==-K -- hence the sign flip
 * on the q output below. */
static void DeriveDqFromPhaseCurrents(double iaAmps, double ibAmps, double electricalAngleDeg,
                                      double dq[2]) {
  double alphaBeta[2];
  Clarke(iaAmps, ibAmps, alphaBeta);
  double thetaRad = electricalAngleDeg * FOC_PI / 180.0;
  double rawDq[2];
  FocPark(alphaBeta, thetaRad, rawDq);
  dq[0] = rawDq[0];
  dq[1] = -rawDq[1];
}

/* IMPORTANT: this call site always passes the PLANT's own TRUE electrical
 * angle here (polePairs*mechanicalAngle, independent of anything firmware
 * computed), NOT firmware's own CommutationElectricalAngleFromEncoderDeg()
 * estimate -- a real amp has no access to firmware's internal angle
 * estimate or its commutation.offset_deg calibration constant; it only
 * ever sees the resulting Ia/Ib analog signals and tracks the motor's
 * ACTUAL electrical angle by its own independent means (resolver, back-EMF,
 * whatever). Using firmware's own angle here instead would be circular --
 * it would make this test unable to tell a correctly-calibrated
 * commutation.offset_deg from a wrong one (verified: deliberately injecting
 * a wrong offsetDeg below and decoding against firmware's OWN angle instead
 * of this true one still "passed" with tiny Id, because both sides would be
 * shifted by the same wrong amount and cancel out -- exactly the kind of
 * false-pass this derivation is written out above to let a reviewer
 * re-check by hand). With the true angle used here, a wrong offsetDeg
 * shows up as real, nonzero Id -- see DeriveDqFromPhaseCurrents()'s comment
 * above for the algebra: d=K*sin(e), q=-K*cos(e) where e is the
 * misalignment, matching docs/two-phase-drive.md's own point that a wrong
 * alignment costs real torque without being obviously wrong. */

static int IsFinitePair(const double v[2]) { return isfinite(v[0]) && isfinite(v[1]); }

int main(void) {
  OutputInit();
  OutputSetTopology(OUTPUT_TOPOLOGY_TWO_PHASE_ANALOG);

  PmFocTrial t = PmFocTrialInit();
  const double currentScale = t.params.currentLimit;   /* full-scale Q15 == this amp's configured peak current */

  /* Synthetic encoder: motor.polePairs=4 (PmFocTrialInit's sample motor),
   * an arbitrary-but-divisible encoder resolution -- any real board's
   * actual CPR would be a CLI-configured constant the same way
   * commutation.counts_per_elec_rev already is (see Commutation.h). */
  const int32_t countsPerMechRev = 8192;
  const int32_t polePairs = t.motor.polePairs;
  const int32_t countsPerElecRev = countsPerMechRev / polePairs;
  const int32_t offsetDeg = 0;   /* no alignment error in this synthetic rig --
                                     CommutationLearn*'s own math is covered by
                                     test_commutation.c, not re-tested here */

  const int32_t torqueCommandQ = (TORQUE_FULL_SCALE * 3) / 8;   /* ~3 A of the amp's 8 A limit */
  const int ticks = 3000;   /* 3 s @ 1kHz, this driver's own firmware-rate tick */

  double maxAbsIdAfterSettling = 0, maxAbsIq = 0;
  int anyNonFinite = 0;

  for (int tick = 0; tick < ticks; tick++) {
    double mechanicalAngleRad = PmFocTrialAngle(&t);
    int32_t encoderCounts = (int32_t)llround(mechanicalAngleRad / (2.0 * FOC_PI) * countsPerMechRev);

    /* ---- REAL firmware code: Commutation.c + Output.c ------------------ */
    int32_t angleDeg = CommutationElectricalAngleFromEncoderDeg(encoderCounts, countsPerElecRev,
                                                                 offsetDeg);
    int32_t ia, ib;
    CommutationComputePhases(torqueCommandQ, angleDeg, &ia, &ib);
    OutputSetTwoPhaseCommand(ia, ib);   /* -> captured by the mocked HAL above */

    /* ---- Stand-in amp: reconstruct + close the current loop ------------ */
    int32_t recoveredIa, recoveredIb;
    StandInAmpReadLastCommand(&recoveredIa, &recoveredIb);
    double iaAmps = (double)recoveredIa / TORQUE_FULL_SCALE * currentScale;
    double ibAmps = (double)recoveredIb / TORQUE_FULL_SCALE * currentScale;
    /* The plant's own TRUE electrical angle -- NOT firmware's angleDeg
     * estimate above. See the "IMPORTANT" comment right before main()
     * for why this distinction is the whole point of this test. */
    double trueElectricalAngleDeg = fmod((double)polePairs * mechanicalAngleRad * 180.0 / FOC_PI,
                                        360.0);
    if (trueElectricalAngleDeg < 0) trueElectricalAngleDeg += 360.0;
    double dq[2];
    DeriveDqFromPhaseCurrents(iaAmps, ibAmps, trueElectricalAngleDeg, dq);
    if (!IsFinitePair(dq)) anyNonFinite = 1;
    t.controller.currentReference[0] = dq[0];
    t.controller.currentReference[1] = dq[1];

    for (int sub = 0; sub < PMSM_CURRENT_TICKS_PER_SPEED; sub++) {
      double mechanicalAngle = PmFocTrialAngle(&t);
      double sensedAngle = PmResolverSensedAngle(mechanicalAngle, t.motor.polePairs,
                                                 t.motor.electricalOffset, &t.resolver);
      double speed = PmFocTrialSpeed(&t);
      PmFocCurrentStep(&t.controller, &t.params, t.electrical.current, sensedAngle, speed,
                       PMSM_CURRENT_DT);
      t.voltageLimitedTicks += t.controller.voltageLimited;
      SimmyPmOutput output;
      SimmyPmStep(&t.electrical, &t.motor, t.controller.voltage, mechanicalAngle, speed,
                 PMSM_CURRENT_DT, &output);
      SimmyChainStepDt(&t.mechanics, &t.chain, output.torque / t.mount.forcePerNm, 1.0);
      t.currentTicks++;
      if (!isfinite(t.electrical.current[0]) || !isfinite(t.electrical.current[1]) ||
          !isfinite(PmFocTrialSpeed(&t)) || !isfinite(PmFocTrialAngle(&t))) {
        anyNonFinite = 1;
      }
    }

    double finalSpeed = PmFocTrialSpeed(&t);
    double finalId = t.controller.measuredCurrent[0];
    double finalIq = t.controller.measuredCurrent[1];
    if (fabs(finalIq) > maxAbsIq) maxAbsIq = fabs(finalIq);
    if (tick >= 500 && fabs(finalId) > maxAbsIdAfterSettling) maxAbsIdAfterSettling = fabs(finalId);
    (void)finalSpeed;
  }

  double finalSpeed = PmFocTrialSpeed(&t);
  double voltageLimitedPct = 100.0 * t.voltageLimitedTicks / t.currentTicks;

  printf("finalSpeed=%.4f rad/s  maxAbsIq=%.4f A  maxAbsIdAfterSettling=%.4f A  "
         "voltageLimited=%.2f%%  anyNonFinite=%d\n",
         finalSpeed, maxAbsIq, maxAbsIdAfterSettling, voltageLimitedPct, anyNonFinite);

  /* Bounds calibrated against this exact scenario (3 A commanded of an 8 A
   * limit, sample motor from docs/motors/bldc.md, 3 s @ 1 kHz) -- same
   * "tie back to a concrete, reproducible number" convention as simmy's
   * own examples/analog_servo_integration/main.c. Re-derive by running this
   * driver directly if the scenario above ever changes.
   *
   * voltageLimitedPct is NOT bounded tight: this motor's friction is low
   * enough (see docs/motors/bldc.md) that a sustained 3A command's
   * friction-only steady-state speed is far above the ~170rad/s this
   * 24V bus can sustain against this motor's back-EMF (same ceiling
   * examples/pmsm_foc.h's own PmFocStressReference comment calls out) --
   * so spending much of a multi-second constant-current run voltage-
   * saturated while climbing toward that ceiling is EXPECTED physics here,
   * not a fault. 87.96% was this scenario's own measured value (see this
   * function's printf above) when this test was written; the real
   * regression signal is finalSpeed's sign/magnitude and
   * maxAbsIdAfterSettling staying small, not this percentage. */
  int ok = !anyNonFinite
        && finalSpeed > 50.0          /* really spun up, in the commanded (positive) direction */
        && maxAbsIdAfterSettling < 0.75  /* commutation stayed aligned: Id small vs. the ~3A Iq */
        && maxAbsIq < currentScale + 0.5 /* current loop respected its own configured limit */
        && voltageLimitedPct < 95.0;     /* not stuck saturated from tick 0 (a sign/angle bug) */

  printf("%s -- real Commutation.c+Output.c, linked via the stand-in amp above, "
         "into simmy's PMSM FOC plant\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
