/*
 * HAL configuration — trimmed from STM32Cube FW_L1's stm32l1xx_hal_conf_template.h
 * to the modules torque_servo actually uses: RCC/GPIO/EXTI/CORTEX/FLASH/PWR (clock
 * bring-up incl. voltage-scale range1 for 32 MHz, pin bring-up, FLASH also used by
 * ConfigStore.c's flash persistence), TIM (encoder, STEP/DIR, control tick on TIM6 --
 * L152 has no TIM1/TIM8, see tim.h), PCD (USB CDC transport), UART (USART1, plain
 * point-to-point), DAC (env:torque_servo_l152 always defines TORQUE_SERVO_USE_DAC --
 * L152RE has a real DAC1, unlike the F103). Mirrors stm32f1xx_hal_conf.h's module
 * selection 1:1 plus PWR (needed for HAL_PWREx voltage-scale config, no F103
 * equivalent call exists).
 */
#ifndef __STM32L1xx_HAL_CONF_H
#define __STM32L1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* NOTE: shadowed on the real build by framework-stm32cubel1's own sibling
 * Inc/stm32l1xx_hal_conf.h (quote-include resolves against the including
 * file's directory before -Iinclude) -- see stm32f1xx_hal_conf.h's NOTE,
 * same package-layout quirk on this family too. Kept trimmed anyway. */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#ifdef TORQUE_SERVO_USE_DAC
#define HAL_DAC_MODULE_ENABLED
#endif

/* ST-Link's own MCO, taken as HSE via RCC_HSE_BYPASS (see main.c's
 * SystemClock_Config) -- see platformio.ini -D HSE_VALUE. Guarded so the
 * build-flag definition wins, same gotcha class as app/'s F411 HSE_VALUE fix. */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif
#if !defined(MSI_VALUE)
#define MSI_VALUE 2097000U
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 37000U
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
#include "stm32l1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32l1xx_hal_gpio.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32l1xx_hal_exti.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32l1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32l1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32l1xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32l1xx_hal_pwr.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32l1xx_hal_tim.h"
#endif
#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32l1xx_hal_pcd.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32l1xx_hal_uart.h"
#endif
#ifdef HAL_DAC_MODULE_ENABLED
#include "stm32l1xx_hal_dac.h"
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

#endif /* __STM32L1xx_HAL_CONF_H */
