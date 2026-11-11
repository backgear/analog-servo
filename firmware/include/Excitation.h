#ifndef EXCITATION_H
#define EXCITATION_H
#include <stdint.h>
/* Balanced random torque pairs, added to the holding loop. Not persistent.
 * Result: 0 idle, 1 running, 2 complete, 3 stopped, 4 interlock,
 * 5 travel, 6 speed, 7 output would clip. Single main-loop producer / ISR consumer.
 */
int ExcitationStart(int32_t amplitude, uint32_t hold, uint32_t duration,
                    int32_t travel, int32_t speed, int32_t position);
void ExcitationStop(int result);
int ExcitationActive(void);
int ExcitationResult(void);
int32_t ExcitationTick(int32_t position, int32_t velocity, int interlock);
void ExcitationRejectClipping(void);
int32_t ExcitationLast(void);
int ExcitationSample(void);
#endif
