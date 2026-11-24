#ifndef USART_H
#define USART_H

#if defined(STM32L152xE)
#include "stm32l1xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

/* Plain point-to-point UART (NOT RS485) -- USART2 (PA2/PA3) on the
 * NUCLEO-L152RE (the Nucleo-64's ST-Link VCP, SB13/SB14 ON by default, free
 * serial console over the same USB cable already used for flashing), or
 * USART1 (PA9/PA10) on F103 "Blue Pill" (needs an external USB-serial
 * adapter -- no onboard debugger with a VCP). Always 115200 8N1.
 *
 * A second, dedicated, much-faster UART for Monitor telemetry (for setups
 * that can't or don't want to use composite USB) and a third for Tamagawa
 * absolute-encoder feedback are both draft-PR branches (`monitor-usart`,
 * `tamagawa-feedback`), not built here -- see AGENTS.md. */

extern UART_HandleTypeDef hUart1;

void UsartInit(void);

/* Blocking transmit (a fixed timeout, not indefinite) -- simplicity is fine
 * here: low-rate CLI traffic, not a timing-critical path. */
void UsartTransmit(const uint8_t *data, uint16_t len);

#endif /* USART_H */
