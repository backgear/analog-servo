/*
 * Minimal host-side mock of usart.h -- shadows the real (HAL-dependent)
 * header for simmy_commutation_learn_integration_main.c's build of the
 * real main.c. Extends test/mocks/usart.h's own minimal shape (which only
 * declares UsartTransmit, since no existing Unity test calls UsartInit())
 * with UsartInit() too -- main.c's own real main() (never actually
 * called, but still fully compiled -- see this driver's own top comment)
 * calls it directly.
 */
#ifndef USART_H
#define USART_H

#include <stdint.h>

void UsartInit(void);
void UsartTransmit(const uint8_t *data, uint16_t len);

#endif /* USART_H */
