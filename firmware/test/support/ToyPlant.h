#ifndef TOY_PLANT_H
#define TOY_PLANT_H

#include <math.h>
#include <stdint.h>

/*
 * Host/native-only toy plant, shared by test_stepdir_control.c's
 * PROTOTYPE sections and stepdir_pulse_demo_main.c.
 *
 * The ORIGINAL toy plant used throughout this test suite (and
 * test_control.c's own RunPlant()) is `velocity += torque/16; position +=
 * velocity` -- an instant, zero-lag integrator with no mechanical
 * smoothing at all. This file adds, as continuous (double-precision)
 * state discretized at the real 1ms tick:
 *   - ampAlpha: one-pole lag between torqueCommand and the torque that
 *     actually reaches the mechanism (amplifier/motor electrical
 *     bandwidth). 1.0 = instant, matching the original toy plant exactly.
 *   - inertia: replaces the original plant's hardcoded 16.
 *   - friction: viscous (velocity-proportional) drag. 0 disables it.
 *   - stictionLimit / coulombLevel: static friction that must be
 *     overcome before breakaway, then a (lower, constant-magnitude)
 *     Coulomb friction while sliding. 0 disables -- the original
 *     viscous-only behavior.
 *   - gravityBias: a constant force subtracted every tick (a vertical-
 *     axis-style load). 0 disables.
 *   - coggingAmplitude / coggingPeriodCounts: a generic POSITION-dependent
 *     (not velocity-dependent) sinusoidal torque ripple, period expressed
 *     in encoder counts. Deliberately simplified rather than modeling the
 *     underlying magnetics/commutation in any detail (saturation, dead-
 *     time, etc.) -- a single sinusoid stands in for whichever periodic
 *     effect is relevant: PM rotor magnetic cogging (longer period, a
 *     few % of available torque is typical) or brushed-DC commutation
 *     ripple (much shorter period/higher frequency, tied to commutator
 *     segment count, usually smaller amplitude) -- same field, caller's
 *     choice of period/amplitude picks which one it represents. 0
 *     disables it entirely.
 *
 * Every new field defaults to 0 via designated initializers, so existing
 * callers (the zero-lag and "realistic" plants from earlier commits) are
 * completely unaffected and produce identical numbers.
 *
 * Still a toy, not a claim about any real electromechanical plant. The
 * stiction model is the textbook "check before integrating" formulation
 * (decide whether the driving force can overcome static friction BEFORE
 * clamping velocity to 0, not after) specifically to avoid the classic
 * simulation bug where a naive post-hoc threshold makes velocity chatter
 * around zero every tick instead of actually sticking.
 */

typedef struct {
  double velocity;        /* true continuous plant velocity, counts/tick -- NOT yet quantized
                              to a whole count the way a real encoder reading would be */
  double position;        /* true continuous plant position, counts */
  double filteredTorque;  /* state for the one-pole amplifier/motor lag -- only updated once per
                              real tick, even when stepped in sub-steps (see CoupledPlantStep) */
} ToyPlantState;

typedef struct {
  double inertia;             /* counts/tick^2 per torque unit -- bigger = heavier/slower to
                                   accelerate. 16 matches the original zero-lag toy plant exactly. */
  double ampAlpha;             /* one-pole lag alpha on torque, 0 < ampAlpha <= 1. 1.0 = instant. */
  double friction;             /* viscous (velocity-proportional) drag coefficient. 0 disables. */
  double stictionLimit;        /* max static-friction holding force before breakaway. 0 disables
                                   stiction entirely (falls back to viscous-only, original
                                   behavior). */
  double coulombLevel;          /* kinetic/Coulomb friction magnitude while sliding, opposing
                                    velocity (or opposing the driving force at the instant of
                                    breakaway). Independent of speed, unlike viscous friction. */
  double gravityBias;           /* constant force subtracted every tick. 0 disables. */
  double coggingAmplitude;      /* generic periodic torque ripple (cogging OR commutation --
                                    caller's choice, see the file header comment). 0 disables
                                    the position-dependent ripple entirely. */
  double coggingPeriodCounts;   /* spatial period of the ripple, in encoder counts. Ignored if
                                    coggingAmplitude is 0. */
  double frictionBumpCenter;    /* position (counts) of a ONE-OFF localized patch of extra
                                    friction -- a dusty/less-greased section of a way or screw,
                                    not a recurring/periodic effect like cogging. */
  double frictionBumpHalfWidth; /* counts -- half-width of the affected window. 0 disables the
                                    bump entirely (frictionBumpCenter/Extra are then ignored). */
  double frictionBumpExtra;     /* added to BOTH stictionLimit and coulombLevel while position is
                                    within the window -- a dustier/drier patch resists both
                                    starting and continuing to slide through it. Works even if
                                    stictionLimit/coulombLevel are otherwise 0 (an axis with no
                                    stiction anywhere except this one patch is a reasonable thing
                                    to model too). */
} ToyPlantParams;

#define TOY_PLANT_PARAMS_IDEALIZED { .inertia = 16.0, .ampAlpha = 1.0, .friction = 0.0 }

static inline void ToyPlantInit(ToyPlantState *s) {
  s->velocity = 0.0;
  s->position = 0.0;
  s->filteredTorque = 0.0;
}

/* "At rest" threshold for the stiction check, counts/tick -- not a
 * physical constant, just small enough that it only matters once the
 * plant has genuinely nearly stopped. */
#define TOY_PLANT_STICTION_VELOCITY_EPSILON 0.05

/* The mechanical part of a step (stiction/friction/cogging/gravity ->
 * velocity/position), given an already-computed driving force (torque
 * after any amplifier lag and any coupling spring/damper reaction) and
 * an explicit dt in ticks (1.0 for a plain single-mass step, 1/N for
 * each of a coupled plant's N sub-steps). Factored out so
 * CoupledPlantStep can reuse the exact same per-mass physics instead of
 * duplicating it. */
static inline void ToyPlantMechanicalStep(ToyPlantState *s, const ToyPlantParams *p,
                                           double drivingForce, double dt) {
  drivingForce -= p->gravityBias;
  if (p->coggingAmplitude != 0.0 && p->coggingPeriodCounts != 0.0) {
    drivingForce -= p->coggingAmplitude *
        sin(2.0 * 3.14159265358979323846 * s->position / p->coggingPeriodCounts);
  }

  double stictionLimit = p->stictionLimit;
  double coulombLevel = p->coulombLevel;
  if (p->frictionBumpHalfWidth > 0.0 &&
      fabs(s->position - p->frictionBumpCenter) <= p->frictionBumpHalfWidth) {
    /* A one-off localized patch (a dusty/less-greased section), not a
     * recurring effect -- resists both starting and continuing to slide
     * through it, on top of whatever friction already applies elsewhere. */
    stictionLimit += p->frictionBumpExtra;
    coulombLevel += p->frictionBumpExtra;
  }

  if (stictionLimit > 0.0 && fabs(s->velocity) < TOY_PLANT_STICTION_VELOCITY_EPSILON &&
      fabs(drivingForce) <= stictionLimit) {
    s->velocity = 0.0;   /* stuck: driving force can't overcome static friction -- don't
                             integrate at all, rather than let it creep/chatter */
    return;
  }

  double frictionForce = p->friction * s->velocity;
  if (coulombLevel > 0.0) {
    double slideDirection = (fabs(s->velocity) < TOY_PLANT_STICTION_VELOCITY_EPSILON)
        ? (drivingForce > 0.0 ? 1.0 : -1.0)      /* breaking away -- follow the driving force */
        : (s->velocity > 0.0 ? 1.0 : -1.0);       /* already sliding -- oppose actual motion */
    frictionForce += coulombLevel * slideDirection;
  }
  s->velocity += (drivingForce - frictionForce) / p->inertia * dt;
  s->position += s->velocity * dt;
}

static inline void ToyPlantStep(ToyPlantState *s, const ToyPlantParams *p, int32_t torqueCommand) {
  s->filteredTorque += ((double)torqueCommand - s->filteredTorque) * p->ampAlpha;
  ToyPlantMechanicalStep(s, p, s->filteredTorque, 1.0);
}

/* Rounded to the nearest whole count -- what a real encoder reading of
 * this continuous state would be, and what Control.c's integer feedback
 * interface expects. */
static inline int32_t ToyPlantPositionCounts(const ToyPlantState *s) {
  return (int32_t)(s->position >= 0 ? s->position + 0.5 : s->position - 0.5);
}
static inline int32_t ToyPlantVelocityCounts(const ToyPlantState *s) {
  return (int32_t)(s->velocity >= 0 ? s->velocity + 0.5 : s->velocity - 0.5);
}

/* ---- Two-mass compliant coupling -------------------------------------
 *
 * A motor-side mass driven directly by torqueCommand (through its own
 * amplifier lag/friction/stiction/cogging, identical physics to the
 * single-mass model above) connected to a load-side mass through a
 * spring+damper, with an optional backlash dead-zone. Kept as a SEPARATE
 * model from the single-mass one above rather than generalizing it into
 * one: a rigid coupling is exactly the existing single-mass model
 * (motor+load as one combined inertia -- physically correct for a truly
 * rigid connection, and numerically cheaper/safer than simulating an
 * infinitely-stiff spring), so the two-mass code below only needs to be
 * reached for a GENUINELY compliant coupling (Lovejoy/jaw, belt -- see
 * MachinePresets.h).
 */

typedef struct {
  double springK;       /* torque per count of relative displacement */
  double springD;       /* torque per count/tick of relative velocity (damping) */
  double backlashGap;   /* counts -- half-width of the dead zone where no torque transmits
                            through the spring. 0 disables (no backlash). */
} CouplingParams;

typedef struct {
  ToyPlantState motor;
  ToyPlantState load;
} CoupledPlantState;

static inline void CoupledPlantInit(CoupledPlantState *s) {
  ToyPlantInit(&s->motor);
  ToyPlantInit(&s->load);
}

/* Sub-steps the 1ms tick for numerical stability: a spring stiff enough
 * to be a realistic coupling is a stiff ODE for plain Euler at only
 * 1kHz. 20 sub-steps was calibrated against the stiffest coupling this
 * file's presets actually use (Lovejoy) -- see MachinePresets.h and the
 * commit history for the calibration run. */
#define COUPLED_PLANT_SUBSTEPS 20

static inline double CouplingRelativeDisplacement(double motorPos, double loadPos, double gap) {
  double rel = motorPos - loadPos;
  if (gap <= 0.0) return rel;
  if (rel > gap) return rel - gap;
  if (rel < -gap) return rel + gap;
  return 0.0;
}

static inline void CoupledPlantStep(CoupledPlantState *s, const ToyPlantParams *motorParams,
                                     const ToyPlantParams *loadParams,
                                     const CouplingParams *coupling, int32_t torqueCommand) {
  /* filteredTorque is a per-TICK amplifier state -- update it once per
   * real tick, not once per sub-step. */
  s->motor.filteredTorque += ((double)torqueCommand - s->motor.filteredTorque) * motorParams->ampAlpha;

  const double dt = 1.0 / COUPLED_PLANT_SUBSTEPS;
  for (int step = 0; step < COUPLED_PLANT_SUBSTEPS; step++) {
    double rel = CouplingRelativeDisplacement(s->motor.position, s->load.position, coupling->backlashGap);
    double relVel = s->motor.velocity - s->load.velocity;
    double springForce = coupling->springK * rel + coupling->springD * relVel;

    ToyPlantMechanicalStep(&s->motor, motorParams, s->motor.filteredTorque - springForce, dt);
    ToyPlantMechanicalStep(&s->load, loadParams, springForce, dt);
  }
}

static inline int32_t CoupledPlantMotorPositionCounts(const CoupledPlantState *s) {
  return ToyPlantPositionCounts(&s->motor);
}
static inline int32_t CoupledPlantMotorVelocityCounts(const CoupledPlantState *s) {
  return ToyPlantVelocityCounts(&s->motor);
}
static inline int32_t CoupledPlantLoadPositionCounts(const CoupledPlantState *s) {
  return ToyPlantPositionCounts(&s->load);
}
static inline int32_t CoupledPlantLoadVelocityCounts(const CoupledPlantState *s) {
  return ToyPlantVelocityCounts(&s->load);
}

#endif /* TOY_PLANT_H */
