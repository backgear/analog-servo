#ifndef GPIO_H
#define GPIO_H

#include "stm32l1xx_hal.h"

/* NUCLEO-L152RE, board MB1136 rev C. Single-board firmware -- no family
 * switch here; see docs/bringup.md for the one required board mod
 * (DAC_OUT2/LD2 solder-bridge rework) and docs/architecture.md for the pin
 * map rationale this was carried over from.
 *
 * UNVERIFIED WITHOUT THE L152 DATASHEET AF TABLE IN HAND for a few of these
 * (STEP_PIN, ANALOG_VELOCITY_ADC_PIN's exact ADC channel) -- a one-#define
 * fix if wrong, flagged individually below. */

/* STEP input (Mode A): TIM2_ETR, external clock mode 2. */
#define STEP_GPIO_PORT GPIOA
#define STEP_PIN        GPIO_PIN_0

/* DIR input (Mode A): plain GPIO + EXTI8, ISR flips TIM2->CR1's DIR bit. */
#define DIR_GPIO_PORT GPIOB
#define DIR_PIN        GPIO_PIN_8

/* Encoder A/B: TIM4 hardware encoder mode (TI12, 4x decode). */
#define ENC_A_GPIO_PORT GPIOB
#define ENC_A_PIN        GPIO_PIN_6
#define ENC_B_GPIO_PORT GPIOB
#define ENC_B_PIN        GPIO_PIN_7

/* Encoder Z (index): plain GPIO + EXTI5, latches EncoderZWasSeenSinceEnable. */
#define ENC_Z_GPIO_PORT GPIOB
#define ENC_Z_PIN        GPIO_PIN_5

/* Endstop: plain GPIO + EXTI9, latching hard fault (Fault.c). */
#define ENDSTOP_GPIO_PORT GPIOB
#define ENDSTOP_PIN        GPIO_PIN_9

/* Torque command DAC: DAC_OUT1 (PA4, positive) / DAC_OUT2 (PA5, negative) --
 * two unipolar channels, a downstream differential-input amplifier stage
 * recovers the bipolar result. See Output.c. PA5 requires the LD2
 * solder-bridge rework (see docs/bringup.md) since it doubles as the
 * board's own LED2 by default. */
#define DAC_OUT1_GPIO_PORT GPIOA
#define DAC_OUT1_PIN        GPIO_PIN_4
#define DAC_OUT2_GPIO_PORT GPIOA
#define DAC_OUT2_PIN        GPIO_PIN_5

/* Mode C (analog velocity) input: ADC1 channel for the commanded-velocity
 * magnitude -- see Analog.c/Adc.h. */
#define ANALOG_VELOCITY_ADC_GPIO_PORT GPIOA
#define ANALOG_VELOCITY_ADC_PIN        GPIO_PIN_1

/* Mode C direction: plain GPIO, pull-up, active-LOW. Grounding reverses the
 * commanded velocity's sign; open/high runs forward. */
#define ANALOG_REVERSE_GPIO_PORT GPIOB
#define ANALOG_REVERSE_PIN        GPIO_PIN_0

/* Hardware ENABLE: plain GPIO, pull-up, active-LOW -- a disconnected/broken
 * wire reads HIGH and fails safe. Asserted for normal running; deasserted
 * ramps the commanded velocity to zero (Mode B/C) or freezes the
 * step-derived target (Mode A) -- see docs/bringup.md. Deliberately NOT the
 * same as axis.enabled=0 or a latched fault. */
#define HW_ENABLE_GPIO_PORT GPIOB
#define HW_ENABLE_PIN        GPIO_PIN_10

/* UART (CLI): USART2, Nucleo-64's ST-Link VCP (PA2/PA3) -- free serial
 * console over the same USB cable used to flash. */
#define UART_TX_GPIO_PORT GPIOA
#define UART_TX_PIN        GPIO_PIN_2
#define UART_RX_GPIO_PORT GPIOA
#define UART_RX_PIN        GPIO_PIN_3

/* Onboard LED (LD2), rerouted to PB15 by the DAC_OUT2 solder-bridge rework
 * (see docs/bringup.md) -- active-HIGH. */
#define LED_GPIO_PORT GPIOB
#define LED_PIN        GPIO_PIN_15

/* Fault output, open-drain, active-low -- wire-ORed onto a shared fault bus. */
#define FAULT_OUT_GPIO_PORT GPIOB
#define FAULT_OUT_PIN        GPIO_PIN_4

/* Debug-only: high for the duration of the 1kHz control-tick ISR body --
 * see docs/architecture.md's CPU headroom section. */
#define TICK_ACTIVE_GPIO_PORT GPIOC
#define TICK_ACTIVE_PIN        GPIO_PIN_10

/* Debug-only: high for just the built-in test-signal generator's branch --
 * narrower bracket than TICK_ACTIVE_PIN. */
#define SIGNAL_BUSY_GPIO_PORT GPIOB
#define SIGNAL_BUSY_PIN        GPIO_PIN_13

/* Hardware "true-zero" clamp: drives an external analog switch that shorts
 * the op-amp's differential input pins together, forcing the analog output
 * to true zero regardless of upstream PWM/DAC error. Asserted ONLY on a
 * latched hard fault. Active-high by default -- flip ZeroClampWrite()'s
 * sense if the real switch needs active-low. */
#define ZERO_CLAMP_GPIO_PORT GPIOB
#define ZERO_CLAMP_PIN        GPIO_PIN_1

/* Brake release output: one plain GPIO, downstream hardware (SSR/relay/
 * MOSFET) TBD -- doesn't change the firmware side. Boot default is
 * RELEASED=0 (held/engaged): the common fail-safe brake hardware is
 * spring-engaged/electrically-released, so "safe at boot" means not
 * commanding a release yet. */
#define BRAKE_RELEASE_GPIO_PORT GPIOA
#define BRAKE_RELEASE_PIN        GPIO_PIN_8

void GpioInit(void);
void LedWrite(int on);

/* Open-drain, active-low: drive LOW (sink current) to assert the fault
 * output, release (float high via an external pull-up) to clear it. */
void FaultOutWrite(int active);

/* Active-high: drive HIGH to assert (close) the hardware true-zero clamp,
 * LOW to release it. */
void ZeroClampWrite(int asserted);

/* released=1 commands the brake open (motor free to move); released=0
 * commands it held/engaged -- the boot-time default. Pin polarity is
 * resolved in gpio_l1.c once the downstream driver is chosen. */
void BrakeReleaseWrite(int released);

#endif /* GPIO_H */
