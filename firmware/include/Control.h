#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* Cascaded position(P) -> velocity(PI) control core. Pure fixed-point math,
 * hardware-agnostic -- fed by either Mode A (step/dir) or Mode B (velocity
 * command) via a single "position target" interface (both modes accumulate
 * into a position target upstream, in main.c; this module doesn't know or
 * care which mode produced it). First closed-loop feedback controller
 * anywhere in this repo -- no PI/PID exists elsewhere, this is new code, not
 * ported. See docs/README.md. */

typedef struct {
  int32_t posGainQ8;        /* Q8.8: position error (counts) -> velocity target (counts/tick) */
  int32_t velGainKpQ8;      /* Q8.8: velocity error -> torque (proportional term) */
  int32_t velGainKiQ8;      /* Q8.8: velocity error -> torque, PER TICK integral accumulation
                                (tick rate is hardware-fixed at 1kHz, so dt isn't folded in --
                                see docs/architecture.md's gain-units note) */
  int32_t velocityClamp;    /* combined reference + feedback velocity target cap */
  int32_t torqueClampMin, torqueClampMax;
  int32_t accelerationFeedforwardQ8; /* torque per counts/tick^2, Q8 */
  int32_t velocityFilterShift; /* 0 raw; 1..4 causal IIR alpha=1/(2^shift) */
  int32_t velocityErrorDeadbandCounts; /* counts/tick; |velocity error| at or below this is
                                           treated as exactly 0 for BOTH the P and I terms.
                                           0 disables (default -- existing behavior for anyone
                                           who hasn't configured it). Exists because a single
                                           +-1 count/tick of pure encoder quantization noise at
                                           rest -- unavoidable, not a bug -- gets amplified by
                                           a large velGainKpQ8 into a disproportionate torque
                                           kick with no net motion: real wasted power/heat/
                                           noise. Does NOT suppress a
                                           genuine, larger tracking error -- only the
                                           quantization-noise floor a correctly-settled axis
                                           can never resolve more precisely than one count
                                           anyway, so there is nothing to "correct" there. */
} ControlGains;

/* Readbacks use whole encoder counts/tick, rounded symmetrically from Q8.
 * error is before deadband; controlError is after deadband. */
typedef struct {
  int32_t reference, filtered, error, controlError;
} ControlVelocitySample;
void ControlGetVelocitySample(ControlVelocitySample *out);

void ControlInit(const ControlGains *gains);
void ControlSetGains(const ControlGains *gains);   /* hot-reloadable, e.g. via Protocol.c */
const ControlGains *ControlGetGains(void);

#define CONTROL_TORQUE_SCALE_FULL_Q8 256   /* 1.0 in Q8.8 -- no reduction */

/* Scales torqueClampMin/Max (Q8.8, 256 = 100%) before ControlTick's clamp AND
 * its anti-windup saturation check -- without touching the persisted
 * torqueClampMin/Max gains themselves (`torque.max` over the
 * protocol still reports the real configured value). Defaults to 256 (no
 * scaling) and is reset to 256 by ControlInit()/ControlReset() -- callers
 * that never touch this see identical behavior to before it existed. Callers
 * that DO use it are expected to set it every tick, not just on entry/exit
 * (main.c's AppControlTick does this in every branch), so a stale value can
 * never leak from one branch/mode into another. */
void ControlSetTorqueScale(int32_t scaleQ8);

/* Runs one 1kHz tick of the P->PI cascade. dtCycles is accepted for
 * telemetry/diagnostics parity with Ramp.c's interface, not used in the P/PI
 * math itself (see the gain-units note above). Returns the clamped torque
 * command. Internally retains Q8 velocity and Q16 torque/integral state;
 * integer outputs are rounded symmetrically. */
int32_t ControlTick(int32_t positionTarget, int32_t positionActual,
                     int32_t velocityActual, uint32_t dtCycles);

/* Current trajectory velocity feedforward plus position-error correction.
 * referenceVelocity is this tick's ramp output in encoder coordinates;
 * this uses no future position samples. Combined target is speed-clamped.
 * Optional acceleration feedforward uses a causal mean of ramp-velocity
 * changes, including its torque contribution in anti-windup.
 * ControlTick() is the zero-feedforward compatibility/step-target entry. */
int32_t ControlTickWithVelocity(int32_t positionTarget, int32_t positionActual,
                                int32_t velocityActual, int32_t referenceVelocity,
                                uint32_t dtCycles);

/* Fractional reference entry for tunable feedforward scaling. */
int32_t ControlTickWithReferenceQ8(int32_t positionTarget, int32_t positionActual,
                                   int32_t velocityActual, int64_t referenceVelocityQ8,
                                   uint32_t dtCycles);

/* Per-command velocity cap, bounded by the configured feedback limit. */
int32_t ControlTickWithReferenceLimitQ8(int32_t positionTarget, int32_t positionActual,
                                        int32_t velocityActual, int64_t referenceVelocityQ8,
                                        int32_t velocityLimit, uint32_t dtCycles);

/* Just the velocity(PI) stage ControlTick() itself runs after its
 * position(P) stage -- same gains/anti-windup/clamp, exposed directly for
 * modes with no position/direction to run a P stage against (tach-style
 * velocity-only loops, see docs/modes.md's Tach modes
 * section). ControlGetLastVelocityTarget() reflects velocityTarget as
 * passed in; ControlGetLastPositionError() is left untouched by this entry
 * point. */
int32_t ControlVelocityTick(int32_t velocityTarget, int32_t velocityActual, uint32_t dtCycles);

/* Clears the PI integrator -- call on an enable-edge or after a fault clears,
 * so a stale integral term from before the servo was last disabled/faulted
 * doesn't kick the output on re-enable. */
void ControlReset(void);

/* Diagnostics -- last computed velocity target/error, for `status`/telemetry. */
int32_t ControlGetLastVelocityTarget(void);
int32_t ControlGetLastPositionError(void);

#endif /* CONTROL_H */
