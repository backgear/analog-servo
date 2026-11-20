#ifndef MACHINE_PRESETS_H
#define MACHINE_PRESETS_H

#include "Control.h"
#include "ToyPlant.h"

/*
 * Named, bundled sim parameter sets for two recognizable real mechanisms,
 * built on ToyPlant.h's two-mass compliant-coupling model. Representative
 * ORDER-OF-MAGNITUDE ratios, not measurements of any specific real
 * machine -- same house style as every other number in this sim. The
 * point is to ground the simulation's conclusions in something a reader
 * can picture, not to claim these are correct to any particular catalog
 * part.
 *
 * Both presets feed back the MOTOR-side position/velocity (the common,
 * cheap choice for a hobby retrofit -- a load-side/glass-scale encoder
 * is a plausible future preset, not these two). Each preset's
 * ControlGains were found the same way as every other gain set in this
 * suite: a step-response grid search (target step, looking for no
 * sustained clamp-pegging and reasonable settling) run via a scratch
 * calibration program, not guessed -- and are only valid for THAT
 * preset's own plant, not interchangeable with it or with the other
 * preset's gains.
 *
 *
 * NEMA 23 lead-screw mill X-axis
 * ------------------------------
 * A PM stepper/servo motor driving a ball screw through a Lovejoy
 * (jaw/elastomeric-spider) coupling -- a very common hobby CNC retrofit
 * pairing. Distinguishing physical features relative to the treadmill
 * preset below:
 *   - Cogging torque: present. A NEMA 23 is almost always a PM rotor,
 *     and magnetic cogging is usually its single most distinctive
 *     mechanical signature (often a few percent of available torque).
 *     The treadmill motor (brushed DC) has no equivalent effect.
 *   - Stiction: meaningful fraction of available torque. Modeled on the
 *     LOAD side (way/slide friction, downstream of the screw -- the
 *     textbook source of stick-slip on a mill axis), not the motor.
 *   - Reflected load inertia: kept small relative to the motor. Lead
 *     screws have large mechanical advantage (J_reflected =
 *     m*(lead/2*pi)^2 for a linear load of mass m), so a table's mass
 *     barely shows up at the motor -- the system stays motor-inertia-
 *     dominated even though a real table might be many kg.
 *   - Coupling: Lovejoy is reasonably stiff with real elastomeric
 *     damping and a small amount of backlash (spider fit/wear) --
 *     calibrated so its resonance sits well above this gain set's loop
 *     bandwidth (a well-chosen coupling should be nearly invisible to
 *     the control loop, not a source of trouble).
 *   - Gravity: none (horizontal axis).
 */
static const ToyPlantParams kNema23LeadscrewMotor = {
  .inertia = 64.0, .ampAlpha = 0.6, .friction = 0.02,
  .coggingAmplitude = 60.0, .coggingPeriodCounts = 20.0,
};
static const ToyPlantParams kNema23LeadscrewLoad = {
  .inertia = 8.0, .friction = 0.01, .stictionLimit = 80.0, .coulombLevel = 50.0,
};
static const CouplingParams kNema23LeadscrewCoupling = {
  .springK = 3.16, .springD = 1.0, .backlashGap = 0.5,
};
static const ControlGains kNema23LeadscrewGains = {
  .posGainQ8 = 32, .velGainKpQ8 = 1024, .velGainKiQ8 = 1,
  .velocityClamp = 200000, .torqueClampMin = -1000, .torqueClampMax = 1000,
};

/*
 * Treadmill motor spindle
 * -----------------------
 * A brushed DC motor with its original large integrated flywheel,
 * belt-driven to a spindle/chuck shaft -- the classic hobby lathe/mill
 * spindle-motor conversion, and a direct extension of this project's own
 * (currently stub) worked example at
 * docs/examples/brushed-dc-magnetic-encoder.md.
 *   - Cogging torque: none modeled. Brushed DC motors don't have PM
 *     cogging (commutation ripple is a real, different effect this sim
 *     doesn't model yet).
 *   - Stiction: present but low -- ordinary bearing/brush drag, not a
 *     dry sliding way. Modeled on the load (spindle shaft) side.
 *   - Inertia: very large relative to the NEMA 23 case -- the flywheel
 *     dominates, 2-3 orders of magnitude more than a bare stepper rotor.
 *   - Coupling: a belt is softer than a Lovejoy (lower stiffness,
 *     calibrated so its resonance sits closer to this gain set's loop
 *     bandwidth than the Lovejoy case does to its own -- a belt's
 *     compliance is more likely to actually matter to the tune than a
 *     well-chosen rigid-ish coupling's), with real damping from belt
 *     material hysteresis and little backlash (toothed/synchronous belt
 *     assumed, not a slipping V-belt).
 *   - Gravity: none (horizontal spindle).
 */
static const ToyPlantParams kTreadmillSpindleMotor = {
  .inertia = 4000.0, .ampAlpha = 0.6, .friction = 0.05,
};
static const ToyPlantParams kTreadmillSpindleLoad = {
  .inertia = 200.0, .friction = 0.02, .stictionLimit = 15.0, .coulombLevel = 8.0,
};
static const CouplingParams kTreadmillSpindleCoupling = {
  .springK = 1.69, .springD = 5.0, .backlashGap = 0.1,
};
static const ControlGains kTreadmillSpindleGains = {
  .posGainQ8 = 1, .velGainKpQ8 = 8192, .velGainKiQ8 = 1,
  .velocityClamp = 200000, .torqueClampMin = -1000, .torqueClampMax = 1000,
};

#endif /* MACHINE_PRESETS_H */
