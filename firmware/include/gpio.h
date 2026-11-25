#ifndef GPIO_H
#define GPIO_H

#if defined(STM32L152xE)
#include "stm32l1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

/* Two boards: NUCLEO-L152RE (board MB1136 rev C, `main`'s bench-validated
 * target -- see docs/bringup.md for its one required board mod, the
 * DAC_OUT2/LD2 solder-bridge rework) and STM32F103C8T6 "Blue Pill" (this
 * target -- see README.md for validation limits). Pin numbers
 * are identical on both except where a board comment below says otherwise.
 *
 * UNVERIFIED WITHOUT THE RESPECTIVE DATASHEET AF TABLE IN HAND for a few of
 * these (STEP_PIN, ANALOG_VELOCITY_ADC_PIN's exact ADC channel) -- a
 * one-#define fix if wrong, flagged individually below. */

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

#if defined(STM32L152xE)
/* Torque command DAC: DAC_OUT1 (PA4, positive) / DAC_OUT2 (PA5, negative) --
 * two unipolar channels, a downstream differential-input amplifier stage
 * recovers the bipolar result. See Output.c. PA5 requires the LD2
 * solder-bridge rework (see docs/bringup.md) since it doubles as the
 * board's own LED2 by default. */
#define DAC_OUT1_GPIO_PORT GPIOA
#define DAC_OUT1_PIN        GPIO_PIN_4
#define DAC_OUT2_GPIO_PORT GPIOA
#define DAC_OUT2_PIN        GPIO_PIN_5
#else
/* Torque command PWM (poor-man's DAC): TIM3_CH1 (positive) / TIM3_CH2
 * (negative), same two-unipolar-channel scheme as the DAC above, just
 * RC-filtered PWM instead of a real analog output -- F103 has no on-chip
 * DAC silicon at all. See docs/rc-filter.md and Output.c. */
#define TORQUE_PWM_POS_GPIO_PORT GPIOA
#define TORQUE_PWM_POS_PIN        GPIO_PIN_6
#define TORQUE_PWM_NEG_GPIO_PORT GPIOA
#define TORQUE_PWM_NEG_PIN        GPIO_PIN_7
#endif

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
#if defined(STM32L152xE)
#define HW_ENABLE_GPIO_PORT GPIOB
#define HW_ENABLE_PIN        GPIO_PIN_10
#else
/* F103: moved off PB10 to free PB10/PB11 (USART3 TX/RX) for Tamagawa
 * feedback bring-up, same relocation the tamagawa-feedback branch made
 * for L152 -- not ported here, just keeping the pin free for it. */
#define HW_ENABLE_GPIO_PORT GPIOB
#define HW_ENABLE_PIN        GPIO_PIN_12
#endif

#if defined(STM32L152xE)
/* UART (CLI): USART2, Nucleo-64's ST-Link VCP (PA2/PA3) -- free serial
 * console over the same USB cable used to flash. */
#define UART_TX_GPIO_PORT GPIOA
#define UART_TX_PIN        GPIO_PIN_2
#define UART_RX_GPIO_PORT GPIOA
#define UART_RX_PIN        GPIO_PIN_3
#else
/* UART (CLI): USART1 (PA9/PA10) -- F103 "Blue Pill" has no onboard debugger
 * with a VCP, needs an external USB-serial adapter. */
#define UART_TX_GPIO_PORT GPIOA
#define UART_TX_PIN        GPIO_PIN_9
#define UART_RX_GPIO_PORT GPIOA
#define UART_RX_PIN        GPIO_PIN_10
#endif

#if defined(STM32L152xE)
/* Onboard LED (LD2), rerouted to PB15 by the DAC_OUT2 solder-bridge rework
 * (see docs/bringup.md) -- active-HIGH. */
#define LED_GPIO_PORT GPIOB
#define LED_PIN        GPIO_PIN_15
#else
/* Onboard LED (PC13) -- active-LOW (Blue Pill wiring). */
#define LED_GPIO_PORT GPIOC
#define LED_PIN        GPIO_PIN_13
#endif

/* Fault output, open-drain, active-low -- wire-ORed onto a shared fault bus. */
#define FAULT_OUT_GPIO_PORT GPIOB
#define FAULT_OUT_PIN        GPIO_PIN_4

#if defined(STM32L152xE)
/* Debug-only: high for the duration of the 1kHz control-tick ISR body --
 * see docs/architecture.md's CPU headroom section. */
#define TICK_ACTIVE_GPIO_PORT GPIOC
#define TICK_ACTIVE_PIN        GPIO_PIN_10
#else
#define TICK_ACTIVE_GPIO_PORT GPIOB
#define TICK_ACTIVE_PIN        GPIO_PIN_15
#endif

/* Debug-only: high for just the built-in test-signal generator's branch --
 * narrower bracket than TICK_ACTIVE_PIN. Same pin on both boards -- F103
 * moved off PB11 to free PB10/PB11 (USART3) for Tamagawa feedback. */
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

/* Hall sensor inputs, output.topology=two_phase_analog only -- see
 * docs/two-phase-drive.md (`two-phase-output` draft-PR branch). Polled,
 * not EXTI -- see that doc's "EXTI vs. polling" analysis. Pull-up assumed;
 * NOT confirmed against a real Hall sensor's actual output type. Harmless
 * to configure even on the default `analog_servo` build, which never
 * reads them -- AppSetOutputTopology() rejects two_phase_analog there. */
#define HALL_A_GPIO_PORT GPIOC
#define HALL_A_PIN        GPIO_PIN_0
#define HALL_B_GPIO_PORT GPIOC
#define HALL_B_PIN        GPIO_PIN_1
#define HALL_C_GPIO_PORT GPIOC
#define HALL_C_PIN        GPIO_PIN_2

/* Phase sign bits, output.topology=two_phase_analog's magnitude+sign-switch
 * scheme -- see docs/two-phase-drive.md. Each drives an external CD4053's
 * SEL pins. Active-HIGH = negative, arbitrary convention -- flip
 * PhaseSignWrite()'s sense if the real switch needs the opposite. A real
 * board modification, not yet built on any board. */
#define PHASE_A_SIGN_GPIO_PORT GPIOB
#define PHASE_A_SIGN_PIN        GPIO_PIN_2
#define PHASE_B_SIGN_GPIO_PORT GPIOB
#define PHASE_B_SIGN_PIN        GPIO_PIN_3

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

/* channel 0 = phase A, 1 = phase B. Active-high = negative -- see
 * PHASE_A_SIGN_PIN's comment. output.topology=two_phase_analog only. */
void PhaseSignWrite(int channel, int negative);

#endif /* GPIO_H */
