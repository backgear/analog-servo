/*
 * Minimal host-side mock of usart.h -- shadows the real (HAL-dependent)
 * header for the native test build. Protocol.c only needs UsartTransmit;
 * the test file provides the actual implementation (captures transmitted
 * bytes for assertions).
 */
#ifndef USART_H
#define USART_H

#include <stdint.h>

void UsartTransmit(const uint8_t *data, uint16_t len);

#endif /* USART_H */
