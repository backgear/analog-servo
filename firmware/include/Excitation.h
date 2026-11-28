#ifndef EXCITATION_H
#define EXCITATION_H
#include <stdint.h>

#ifdef ANALOG_SERVO_HAS_EXCITATION
/* Balanced random torque pairs, added to the holding loop. Not persistent.
 * Result: 0 idle, 1 running, 2 complete, 3 stopped, 4 interlock,
 * 5 travel, 6 speed, 7 output would clip. Single main-loop producer / ISR consumer.
 */
int ExcitationStart(int32_t amplitude, uint32_t hold, uint32_t duration,
                    int32_t travel, int32_t speed, int32_t position);
/* Moving tests accept a fixed nonzero host command (+/-20 counts/tick max).
 * Completion/abort requests that main ramp the host command back to zero. */
int ExcitationStartMoving(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position,
                          int32_t commandedVelocity);
int ExcitationStartTorque(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position);
int ExcitationTorqueMode(void);
int32_t ExcitationExpectedVelocity(void);
int ExcitationTakeMotionStop(void);
void ExcitationStop(int result);
int ExcitationActive(void);
int ExcitationResult(void);
int32_t ExcitationTick(int32_t position, int32_t velocity, int interlock);
void ExcitationRejectClipping(void);
int32_t ExcitationLast(void);
int ExcitationSample(void);

#else /* !ANALOG_SERVO_HAS_EXCITATION -- degenerate always-idle stand-ins, same
       * convention as ANALOG_SERVO_TWO_PHASE_DRIVE's #else branches in main.c:
       * excluded on F103 for flash (Excitation.c itself isn't even compiled
       * in, see platformio.ini), so every call site in main.c/Monitor.c
       * compiles against these instead, completely unchanged. Values are
       * chosen so the real call sites' logic degenerates to "excitation
       * never runs": ExcitationActive()/ExcitationSample()/
       * ExcitationTorqueMode()/ExcitationTakeMotionStop() all read false,
       * ExcitationTick() never injects torque, and Start*() report "stopped"
       * (3) rather than succeeding, since nothing will ever poll them to
       * completion on this build. */
static inline int ExcitationStart(int32_t amplitude, uint32_t hold, uint32_t duration,
                    int32_t travel, int32_t speed, int32_t position) {
  (void)amplitude; (void)hold; (void)duration; (void)travel; (void)speed; (void)position;
  return 3;
}
static inline int ExcitationStartMoving(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position,
                          int32_t commandedVelocity) {
  (void)amplitude; (void)hold; (void)duration; (void)travel; (void)speed; (void)position;
  (void)commandedVelocity;
  return 3;
}
static inline int ExcitationStartTorque(int32_t amplitude, uint32_t hold, uint32_t duration,
                          int32_t travel, int32_t speed, int32_t position) {
  (void)amplitude; (void)hold; (void)duration; (void)travel; (void)speed; (void)position;
  return 3;
}
static inline int ExcitationTorqueMode(void) { return 0; }
static inline int32_t ExcitationExpectedVelocity(void) { return 0; }
static inline int ExcitationTakeMotionStop(void) { return 0; }
static inline void ExcitationStop(int result) { (void)result; }
static inline int ExcitationActive(void) { return 0; }
static inline int ExcitationResult(void) { return 0; }
static inline int32_t ExcitationTick(int32_t position, int32_t velocity, int interlock) {
  (void)position; (void)velocity; (void)interlock;
  return 0;
}
static inline void ExcitationRejectClipping(void) {}
static inline int32_t ExcitationLast(void) { return 0; }
static inline int ExcitationSample(void) { return 0; }
#endif /* ANALOG_SERVO_HAS_EXCITATION */

#endif
