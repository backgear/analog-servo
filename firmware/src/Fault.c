/*
 * Latched hard-fault state machine. See Fault.h.
 */
#include "Fault.h"
#include "Output.h"   /* TORQUE_FULL_SCALE (threshold sizing) + OutputForceSafe() */

#define POLARITY_MIN_TORQUE    (TORQUE_FULL_SCALE / 4)
#define POLARITY_SAMPLE_TICKS  20    /* two 10ms velocity averages @ 1kHz */
#define POLARITY_MIN_DELTA_SUM 20    /* >=2 counts/tick change between averages */
#define POLARITY_BAD_WINDOWS   3    /* 60ms of opposite acceleration, after settling */

#define ZERO_PULSE_MIN_TORQUE   (TORQUE_FULL_SCALE / 8)
#define ZERO_PULSE_WINDOW_TICKS 200  /* ~200ms @ 1kHz */

/* Struct-initializer fallback only -- inert in the real firmware, where
 * main.c's boot sequence always calls ApplyConfigToModules() (which calls
 * FaultSetSustainedCurrentConfig() from ConfigStore.c's
 * ConfigDataSetDefaults(), or a loaded flash record) before FaultTick()
 * ever runs -- same convention as Control.c's sGains. On by default
 * (minTorque > 0), configurable, no soft/warn-only mode -- see Fault.h
 * and docs/two-phase-drive.md's "Polarity faults can't be detected here
 * either" section. */
static int32_t sSustainedCurrentMinTorque   = TORQUE_FULL_SCALE / 10;   /* 10% */
static int32_t sSustainedCurrentWindowTicks = 2000;                      /* ~2s @ 1kHz */

static int         sLatched = 0;
static FaultReason sReason = FAULT_NONE;

static int32_t  sPolarityMismatchTicks = 0;
static int32_t  sPreviousAxisTorque = 0;
static int      sPolarityTorqueSign = 0;
static int      sPolaritySampleTicks = 0;
static int      sPolaritySettling = 1;
static int64_t  sPolarityVelocityDifference = 0;

static void ResetPolarityWindow(void) {
  sPolarityMismatchTicks = 0;
  sPolarityTorqueSign = 0;
  sPolaritySampleTicks = 0;
  sPolaritySettling = 1;
  sPolarityVelocityDifference = 0;
}
static int      sPolarityCheckSuppressed = 0;

static int32_t  sZeroPulseTicksElapsed = 0;
static uint32_t sZeroPulseWindowStartEdges = 0;

static int32_t  sSustainedCurrentTicks = 0;

static int32_t sFollowingErrorLimitCounts = 0;   /* 0 == disabled until configured */
static int     sFollowingErrorHardFault = 0;
static int     sFollowingErrorActive = 0;

static void FaultLatch(FaultReason reason) {
  if (sLatched) return;   /* first-to-latch wins */
  sLatched = 1;
  sReason = reason;
  OutputForceSafe();
}

void FaultInit(void) {
  sLatched = 0;
  sReason = FAULT_NONE;
  ResetPolarityWindow();
  sPreviousAxisTorque = 0;
  sPolarityCheckSuppressed = 0;
  sZeroPulseTicksElapsed = 0;
  sZeroPulseWindowStartEdges = 0;
  sSustainedCurrentTicks = 0;
  sFollowingErrorActive = 0;
}

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

void FaultTick(int32_t torqueCommand, int32_t velocityActual, int32_t positionError,
               uint32_t totalEdgesSeen) {
  /* Following error is tracked even while latched (it's informational, not
   * itself the latch source unless configured), so it stays live for
   * telemetry. Everything else below skips once latched -- first-to-latch
   * wins, and there's no value in continuing to evaluate windows once the
   * output is already forced safe. */
  sFollowingErrorActive = (sFollowingErrorLimitCounts > 0) &&
                          (Abs32(positionError) > sFollowingErrorLimitCounts);
  if (sFollowingErrorActive && sFollowingErrorHardFault) {
    FaultLatch(FAULT_FOLLOWING_ERROR);
  }

  if (sLatched) return;

  /* Velocity measured now responds to PREVIOUS output, not the command
   * about to be written. Both inputs use encoder/axis coordinates.
   * Braking (torque opposite velocity) is legitimate. Look instead for
   * sustained acceleration opposite a substantial, constant-sign torque.
   * Average adjacent 10-tick velocity blocks to reject encoder quantization;
   * discard the first 20-tick window after a torque sign/threshold change
   * for drive delay. No observable acceleration is inconclusive, not wrong
   * polarity. External forces can still defeat this heuristic; it is not a
   * substitute for controlled polarity learning or following-error limits. */
  int torqueSign = sPreviousAxisTorque >= POLARITY_MIN_TORQUE ? 1 :
                   sPreviousAxisTorque <= -POLARITY_MIN_TORQUE ? -1 : 0;
  sPreviousAxisTorque = torqueCommand;
  if (sPolarityCheckSuppressed || torqueSign == 0) {
    ResetPolarityWindow();
  } else {
    if (torqueSign != sPolarityTorqueSign) {
      ResetPolarityWindow();
      sPolarityTorqueSign = torqueSign;
    }
    sPolarityVelocityDifference +=
        sPolaritySampleTicks < POLARITY_SAMPLE_TICKS / 2 ?
        -(int64_t)velocityActual : (int64_t)velocityActual;
    if (++sPolaritySampleTicks == POLARITY_SAMPLE_TICKS) {
      if (!sPolaritySettling &&
          sPolarityVelocityDifference * torqueSign <= -POLARITY_MIN_DELTA_SUM) {
        if (++sPolarityMismatchTicks >= POLARITY_BAD_WINDOWS) {
          FaultLatch(FAULT_POLARITY);
          return;
        }
      } else {
        sPolarityMismatchTicks = 0;
      }
      sPolaritySettling = 0;
      sPolaritySampleTicks = 0;
      sPolarityVelocityDifference = 0;
    }
  }

  /* ---- Zero encoder pulses while motion commanded. ---- */
  if (Abs32(torqueCommand) >= ZERO_PULSE_MIN_TORQUE) {
    if (sZeroPulseTicksElapsed == 0) sZeroPulseWindowStartEdges = totalEdgesSeen;
    sZeroPulseTicksElapsed++;
    if (sZeroPulseTicksElapsed >= ZERO_PULSE_WINDOW_TICKS) {
      if (totalEdgesSeen == sZeroPulseWindowStartEdges) {
        FaultLatch(FAULT_ZERO_ENCODER);
        return;
      }
      sZeroPulseTicksElapsed = 0;   /* restart the window with a fresh baseline */
    }
  } else {
    sZeroPulseTicksElapsed = 0;   /* command dropped below threshold -- no longer "should be moving" */
  }

  /* ---- Sustained high commanded current/torque: proxy for bad alignment
   * or any other "producing heat, not useful torque" failure that isn't
   * directly observable without real current sensing (limitations.md).
   * Unlike following error, there's no soft/warn-only mode -- no
   * legitimate operating point holds this high for this long, so it's a
   * direct rule when armed. sSustainedCurrentMinTorque <= 0 disables it
   * entirely (see FaultSetSustainedCurrentConfig). ---- */
  if (sSustainedCurrentMinTorque > 0 && Abs32(torqueCommand) >= sSustainedCurrentMinTorque) {
    sSustainedCurrentTicks++;
    if (sSustainedCurrentTicks >= sSustainedCurrentWindowTicks) {
      FaultLatch(FAULT_SUSTAINED_CURRENT);
      return;
    }
  } else {
    sSustainedCurrentTicks = 0;
  }
}

void FaultEndstopIsr(void) { FaultLatch(FAULT_ENDSTOP); }

int         FaultIsLatched(void) { return sLatched; }
FaultReason FaultGetReason(void) { return sReason; }

void FaultClear(void) {
  sLatched = 0;
  sReason = FAULT_NONE;
  ResetPolarityWindow();
  sPreviousAxisTorque = 0;
  sZeroPulseTicksElapsed = 0;
  sSustainedCurrentTicks = 0;
}

void FaultSetFollowingErrorConfig(int32_t limitCounts, int hardFault) {
  sFollowingErrorLimitCounts = limitCounts;
  sFollowingErrorHardFault = hardFault ? 1 : 0;
}
int32_t FaultGetFollowingErrorLimitCounts(void) { return sFollowingErrorLimitCounts; }
int     FaultGetFollowingErrorHardFault(void) { return sFollowingErrorHardFault; }
int     FaultGetFollowingErrorActive(void) { return sFollowingErrorActive; }

void FaultSetSustainedCurrentConfig(int32_t minTorque, int32_t windowTicks) {
  sSustainedCurrentMinTorque = minTorque;
  sSustainedCurrentWindowTicks = windowTicks;
}
int32_t FaultGetSustainedCurrentMinTorque(void) { return sSustainedCurrentMinTorque; }
int32_t FaultGetSustainedCurrentWindowTicks(void) { return sSustainedCurrentWindowTicks; }

void FaultSetPolarityCheckSuppressed(int suppressed) {
  int next = suppressed ? 1 : 0;
  if (next != sPolarityCheckSuppressed) {
    ResetPolarityWindow();
    sPreviousAxisTorque = 0;
  }
  sPolarityCheckSuppressed = next;
}
int  FaultGetPolarityCheckSuppressed(void) { return sPolarityCheckSuppressed; }
