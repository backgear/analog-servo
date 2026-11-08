#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

/* Latched hard-fault state machine. First-to-latch wins; OutputForceSafe()
 * is called immediately on any latch; the control loop is blocked (main.c
 * checks FaultIsLatched()) while latched; LED/comms/telemetry keep running
 * (the "serviceable" latch convention, not a true CPU halt); cleared only by
 * FaultClear() (explicit host command) or reboot. See
 * docs/README.md. */

typedef enum {
  FAULT_NONE = 0,
  FAULT_POLARITY,          /* reversed motor/hall wiring -- distinct from following error */
  FAULT_ZERO_ENCODER,      /* commanded motion, zero encoder edges -- broken/disconnected encoder */
  FAULT_ENDSTOP,           /* endstop switch tripped */
  FAULT_FOLLOWING_ERROR,   /* following error escalated to a hard fault (config-gated, see below) */
  FAULT_SUSTAINED_CURRENT, /* |torqueCommand| pegged high for too long -- see below, on by default */
  FAULT_COUNT
} FaultReason;

/* Single source of truth for FaultReason's wire names, in enum order --
 * Protocol.c's `fault.reason` ENUM_META and Monitor.c's schema JSON both
 * quote this instead of keeping their own copy, so the two can't drift
 * apart as faults are added. No brackets -- callers wrap it in whatever
 * their own JSON shape needs (a "choices" array vs. a bare value list). */
#define FAULT_REASON_NAMES_JSON \
  "\"none\",\"polarity\",\"zero_encoder\",\"endstop\",\"following_error\",\"sustained_current\""

void FaultInit(void);

/* Runs the polarity + zero-encoder-pulse windowed detectors and the
 * following-error check every control tick. No-op (returns immediately) if
 * already latched. torqueCommand must be in encoder/axis coordinates,
 * BEFORE mapping through polaritySign to the physical output. Acceleration
 * is checked against the previous tick's torque using averaged velocity
 * windows; braking is valid. Zero acceleration is inconclusive. */
void FaultTick(int32_t torqueCommand, int32_t velocityActual, int32_t positionError,
               uint32_t totalEdgesSeen);

/* EXTI9 handler body -- immediate latch, no windowing needed. */
void FaultEndstopIsr(void);

int         FaultIsLatched(void);
FaultReason FaultGetReason(void);
void        FaultClear(void);   /* explicit only -- never auto-clears */

/* Following-error config: limitCounts is the |positionError| threshold;
 * hardFault (0/1) selects whether exceeding it latches (via FAULT_COUNT's
 * FAULT_FOLLOWING_ERROR) or only sets the live, auto-clearing warning bit
 * FaultGetFollowingErrorActive() exposes. Defaults to non-latching (0) --
 * following error is expected transiently during aggressive accel/decel;
 * promoting it to a hard fault needs a real, bench-tuned threshold first. */
void FaultSetFollowingErrorConfig(int32_t limitCounts, int hardFault);
int32_t FaultGetFollowingErrorLimitCounts(void);
int     FaultGetFollowingErrorHardFault(void);
int     FaultGetFollowingErrorActive(void);   /* live, non-latched: |positionError| > limit right now */

/* Sustained-high-commanded-current config (FAULT_SUSTAINED_CURRENT) --
 * see docs/two-phase-drive.md's "Polarity faults can't be detected here
 * either" section. minTorque is the |torqueCommand| threshold (same
 * scale as Control.c's torque clamp); windowTicks is how long it has to
 * stay at/above that before latching. Unlike following error above,
 * there's no soft/warn-only mode -- this is always a hard fault when
 * armed. minTorque <= 0 disables the check entirely (never latches),
 * same "0 == disabled" convention as FaultSetFollowingErrorConfig's
 * limitCounts. Defaults (set via ConfigStore.c, applied at boot) are a
 * deliberately conservative guess, not bench-tuned -- see
 * docs/todo.md. */
void    FaultSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks);
int32_t FaultGetSustainedCurrentMinTorque(void);
int32_t FaultGetSustainedCurrentWindowTicks(void);

/* Suppress polarity detection for raw learning probes and feedback without
 * independent direction. Other fault detectors remain armed. Suppression,
 * low torque and torque sign changes reset acceleration evidence; normal
 * braking, signals and autotune reversals need no special exemption. */
void FaultSetPolarityCheckSuppressed(int suppressed);
int  FaultGetPolarityCheckSuppressed(void);

#endif /* FAULT_H */
