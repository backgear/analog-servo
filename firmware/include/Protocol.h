#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* Line protocol: command in (text, \r/\n/both terminated) -> key=value lines
 * out, terminated by an empty line; errors carry `error=<reason>`. Same
 * framing/registry convention as every other sibling project's Protocol.c,
 * but made transport-agnostic (two instances -- one bound to the single USB
 * CDC-ACM port, one to USART1 -- sharing one registry table and one
 * dispatcher) instead of hardcoding a single transport, since this project
 * has both. A dual-CDC "monitor" port is planned but deferred (see
 * docs/README.md); when it's added it becomes a third
 * instance, or a separate binary-framed stream -- not wired through this
 * text-command registry either way. */

void ProtocolInit(void);

/* Called from USB IRQ context (usbd_cdc_if.c CDC_Receive_FS) -- copies into
 * the USB instance's ring buffer only, no parsing/TX here. */
void ProtocolRxFromIsrUsb(const uint8_t *data, uint32_t len);

/* Called from UART RX ISR (usart.c's HAL_UART_RxCpltCallback) -- same, for
 * the UART instance's ring buffer. */
void ProtocolRxFromIsrUart(const uint8_t *data, uint32_t len);

/* Called from the main superloop: services both instances, assembling/
 * dispatching at most one complete line per instance per call, so it never
 * blocks the caller for long. */
void ProtocolPoll(void);

#endif /* PROTOCOL_H */
