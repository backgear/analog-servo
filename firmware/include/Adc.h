#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* Single-channel ADC1 read for Mode C (analog velocity): free-running
 * (continuous conversion, software-triggered, no DMA/IRQ). AdcGetVelocityRaw()
 * just returns whatever the last completed conversion left in the data
 * register -- it never blocks, and some staleness (at most one ADC
 * conversion time, microseconds) is fine against a 1kHz/1ms control tick
 * reading a slow analog knob. This is deliberately much simpler than
 * analog_quadrature's Adc.c (TIM-triggered dual-channel DMA + oversampling)
 * -- that project's sin/cos interpolation needs a precisely-clocked high
 * sample rate; this is one slow setpoint input. Per-family implementation
 * (adc_f1.c/adc_l1.c/adc_l4.c), same split as gpio.h/tim.h/usart.h. */
void AdcInit(void);

/* Raw ADC code, right-aligned (0..4095 for the 12-bit resolution every
 * family here defaults to). See Analog.h for the scale/hysteresis/
 * reverse-pin math that turns this into a velocity command. */
int32_t AdcGetVelocityRaw(void);

#endif /* ADC_H */
