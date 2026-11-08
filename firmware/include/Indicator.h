#ifndef INDICATOR_H
#define INDICATOR_H

/* HAL_GetTick()-driven pattern state machine, ported from
 * analog_quadrature/ccd_quadrature's Indicator.c shape (renamed states to
 * this project's vocabulary; no buzzer here -- LED (PC13) + FAULT_OUT (PB4)
 * only). */

typedef enum { INDICATOR_IDLE = 0, INDICATOR_RUNNING = 1, INDICATOR_FAULT = 2 } IndicatorState;

void IndicatorInit(void);

/* Base state: IDLE = slow ~1Hz blink (servo disabled), RUNNING = steady on
 * (enabled, no fault), FAULT = fast ~4Hz blink + gpio.c's open-drain
 * FAULT_OUT pin asserted for the duration. main.c decides which condition to
 * report by choosing when to call this; Indicator.c itself doesn't
 * distinguish which of Fault.c's latched reasons is active. */
void IndicatorSetState(IndicatorState state);

/* Transient overlays (interrupt the base pattern, then resume it): single
 * blink = config saved to flash; double blink = save attempted but failed. */
void IndicatorPulseSingle(void);
void IndicatorPulseDouble(void);

/* Call every superloop iteration. */
void IndicatorPoll(void);

#endif /* INDICATOR_H */
