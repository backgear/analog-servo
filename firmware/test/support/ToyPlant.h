#ifndef TOY_PLANT_H
#define TOY_PLANT_H

#include <stdint.h>

/*
 * Host/native-only toy plant, shared by test_stepdir_control.c's
 * "realistic plant" PROTOTYPE section and stepdir_pulse_demo_main.c.
 *
 * The ORIGINAL toy plant used throughout this test suite (and
 * test_control.c's own RunPlant()) is `velocity += torque/16; position +=
 * velocity` -- an instant, zero-lag integrator with no mechanical
 * smoothing at all. That's fine for exercising Control.c's own math in
 * isolation, but it understates how much a torque-command's own
 * quantization/chatter actually matters: every tick's torque ripple shows
 * up immediately and fully as velocity ripple, with nothing resembling a
 * real motor+amplifier's own bandwidth limit to filter it first.
 *
 * This model adds the two most relevant missing effects as continuous
 * (double-precision) state, discretized at the real 1ms tick:
 *   - ampAlpha: one-pole lag between torqueCommand and the torque that
 *     actually reaches the mechanism (amplifier/motor electrical
 *     bandwidth). 1.0 = instant, matching the original toy plant exactly;
 *     smaller = slower.
 *   - inertia: replaces the original plant's hardcoded 16 -- bigger means
 *     the same torque produces a smaller velocity change per tick
 *     (a heavier rotor+load).
 *   - friction: velocity-proportional (viscous) drag subtracted from the
 *     filtered torque before it accelerates the inertia. 0 disables it.
 *
 * Still a toy, not a claim about any real electromechanical plant --
 * there is no Coulomb/static friction, no compliance/backlash, no
 * resonance. It is just enough additional fidelity to ask "does torque-
 * command chatter actually matter once something resembling real
 * mechanical/electrical smoothing sits between it and motion," which the
 * zero-lag plant structurally cannot answer.
 *
 * IMPORTANT: changing these parameters changes the plant's dynamics, so a
 * gain set tuned against the zero-lag plant (inertia=16, ampAlpha=1.0,
 * friction=0, the default below) is NOT expected to remain stable against
 * a more realistic one -- see test_stepdir_control.c's PROTOTYPE section
 * for gains hand-tuned specifically against the "realistic" parameters
 * documented there.
 */

typedef struct {
  double velocity;        /* true continuous plant velocity, counts/tick -- NOT yet quantized
                              to a whole count the way a real encoder reading would be */
  double position;        /* true continuous plant position, counts */
  double filteredTorque;  /* state for the one-pole amplifier/motor lag */
} ToyPlantState;

typedef struct {
  double inertia;   /* counts/tick^2 per torque unit -- bigger = heavier/slower to accelerate.
                        16 matches the original zero-lag toy plant exactly. */
  double ampAlpha;  /* one-pole lag alpha on torque, 0 < ampAlpha <= 1. 1.0 = instant
                        (no lag, matches the original toy plant); smaller = slower amp/motor. */
  double friction;  /* viscous (velocity-proportional) drag coefficient. 0 disables it. */
} ToyPlantParams;

#define TOY_PLANT_PARAMS_IDEALIZED { .inertia = 16.0, .ampAlpha = 1.0, .friction = 0.0 }

static inline void ToyPlantInit(ToyPlantState *s) {
  s->velocity = 0.0;
  s->position = 0.0;
  s->filteredTorque = 0.0;
}

static inline void ToyPlantStep(ToyPlantState *s, const ToyPlantParams *p, int32_t torqueCommand) {
  s->filteredTorque += ((double)torqueCommand - s->filteredTorque) * p->ampAlpha;
  double netForce = s->filteredTorque - p->friction * s->velocity;
  s->velocity += netForce / p->inertia;
  s->position += s->velocity;
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

#endif /* TOY_PLANT_H */
