/*
 * Minimal host-side mock of the STM32 HAL -- only enough for main.h's
 * `#include "stm32l1xx_hal.h"` to resolve. Nothing in the natively-built
 * sources actually calls into the HAL -- Protocol.c/Monitor.c only reach
 * HAL-touching modules through their pure-stdint function signatures,
 * stubbed directly in the test file.
 */
#ifndef MOCK_STM32L1XX_HAL_H
#define MOCK_STM32L1XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

#endif /* MOCK_STM32L1XX_HAL_H */
