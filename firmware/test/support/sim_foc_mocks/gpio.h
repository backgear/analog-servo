/*
 * Minimal host-side mock of gpio.h -- ONLY for simmy_foc_integration_main.c's
 * build of the REAL Output.c. The real gpio.h (firmware/include/gpio.h) is
 * all STM32 GPIOx/GPIO_PIN_* port-pin #defines plus a handful of function
 * declarations; Output.c's two-phase path only ever calls two of those
 * functions (ZeroClampWrite/PhaseSignWrite), so this mock drops every pin
 * #define entirely rather than also mocking the GPIO port typedefs they'd
 * otherwise need -- nothing in Output.c references a GPIOx/GPIO_PIN_*
 * symbol directly. See stm32l1xx_hal.h in this same directory for why this
 * is a dedicated mock dir rather than an addition to test/mocks.
 */
#ifndef GPIO_H
#define GPIO_H

/* Active-high: drive HIGH to assert (close) the hardware true-zero clamp,
 * LOW to release it -- see firmware/include/gpio.h's real declaration. */
void ZeroClampWrite(int asserted);

/* channel 0 = phase A, 1 = phase B. Active-high = negative -- see
 * firmware/include/gpio.h's real PHASE_A_SIGN_PIN comment. */
void PhaseSignWrite(int channel, int negative);

#endif /* GPIO_H */
