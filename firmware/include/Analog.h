#ifndef ANALOG_H
#define ANALOG_H

#include <stdint.h>

/* Mode C (analog velocity): turns a raw ADC code + a reverse-pin level into
 * a signed velocity command, same units/scale as Mode B's
 * velocity.command (counts/tick) -- main.c's AppControlTick feeds the
 * result straight into Ramp.c exactly like Mode B's host-commanded
 * velocity, no further conversion needed. Pure fixed-point math, no HAL --
 * natively testable, same split as Idle.c/Fault.c/Polarity.c. See Adc.h for
 * where rawAdcCode actually comes from, and
 * docs/modes.md for the hardware (ADC pin +
 * reverse pin) this feeds.
 */

/* scaleQ8: Q8.8 fixed point (256 == gain of 1.0 -- one raw ADC LSB becomes
 * one count/tick of velocity), same convention as Control.c's gains.
 * hysteresisCounts: minimum raw-ADC-code movement (same LSB scale as
 * rawAdcCode, NOT counts/tick) required before the value actually acted on
 * is allowed to change again -- a Schmitt-trigger-style filter so pot
 * wiper/wiring noise doesn't chatter the commanded velocity back and forth
 * by a count or two every tick. <= 0 disables the filter (every tick's raw
 * code is acted on immediately). Neither value is tuned -- see
 * ConfigStore.c's defaults comment. */
void AnalogVelocitySetConfig(int32_t scaleQ8, int32_t hysteresisCounts);

/* Forces the next AnalogVelocityTick() call to act on whatever raw code
 * it's given immediately, instead of first having to clear the hysteresis
 * band around a stale remembered code. Call when entering Mode C (see
 * main.c's AppSetServoMode()) so switching into analog mode never inherits
 * a stale acted-on value from before the switch (or from before boot). */
void AnalogVelocityReset(void);

/* One call per control tick while in Mode C. rawAdcCode is the ADC's latest
 * raw reading (see AdcGetVelocityRaw() in Adc.h); reversed is the
 * reverse-pin's current level (nonzero == reverse). Returns the resulting
 * signed velocity, counts/tick -- apply the configured scale to whichever
 * raw code is currently "acted on" (see hysteresisCounts above), then
 * negate if reversed. */
int32_t AnalogVelocityTick(int32_t rawAdcCode, int reversed);

#endif /* ANALOG_H */
