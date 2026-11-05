/*
 * HAL configuration — trimmed from STM32Cube FW_F1's stm32f1xx_hal_conf_template.h
 * to the modules torque_servo actually uses: RCC/GPIO/CORTEX/FLASH (clock + pin
 * bring-up, FLASH also used by ConfigStore.c's flash persistence), TIM (encoder,
 * STEP/DIR, PWM output, control tick), PCD (USB CDC transport), UART (USART1,
 * plain point-to-point). HAL_DAC_MODULE_ENABLED is conditional on
 * TORQUE_SERVO_USE_DAC -- Output.c's DAC path is an untested skeleton for a
 * future DAC-equipped-part respin, not built by default.
 */
#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* NOTE: this file is shadowed on the real build -- the vendored
 * framework-stm32cubef1 package ships its own (fully-enabled)
 * Inc/stm32f1xx_hal_conf.h right next to stm32f1xx_hal.h, and a quote-form
 * #include resolves against the including file's own directory before ever
 * consulting -Iinclude, so that sibling file wins over this one, every
 * module-enable #define below is dead at actual compile time (confirmed via
 * `arm-none-eabi-gcc -E`, added when adc_f1.c's HAL_ADC_MODULE_ENABLED
 * turned out not to be needed here, unlike adc_l4.c's -- see
 * stm32l4xx_hal_conf.h, whose framework package has no such sibling). Kept
 * trimmed anyway (harmless, and correct if that package-layout quirk is
 * ever fixed) -- see torque_servo_design.md's Mode C section. */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#ifdef TORQUE_SERVO_USE_DAC
#define HAL_DAC_MODULE_ENABLED
#endif

/* Blue Pill crystal is 8 MHz (see platformio.ini -D HSE_VALUE); guarded so the
 * build-flag definition wins, same gotcha class as app/'s F411 HSE_VALUE fix. */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE 8000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 40000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif

#define VDD_VALUE 3300U
#define TICK_INT_PRIORITY 0x0FU
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U

#define USE_HAL_PCD_REGISTER_CALLBACKS 0U
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U
#define USE_HAL_UART_REGISTER_CALLBACKS 0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f1xx_hal_pcd.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f1xx_hal_uart.h"
#endif
#ifdef HAL_DAC_MODULE_ENABLED
#include "stm32f1xx_hal_dac.h"
#endif

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
