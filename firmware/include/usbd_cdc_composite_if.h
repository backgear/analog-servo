#ifndef USBD_CDC_COMPOSITE_IF_H
#define USBD_CDC_COMPOSITE_IF_H

#include <stdint.h>
#include "usbd_def.h"   /* USBD_StatusTypeDef (USBD_OK/BUSY/FAIL) for the Transmit return */

/* The one physical USB device -- composite dual-CDC is the only USB
 * transport this repo builds (see usbd_cdc_composite.h/.c). Channel 0 is
 * Protocol.c's CLI; channel 1 is Monitor.c's binary telemetry stream,
 * OUT-only from the host's perspective (incoming bytes on it are drained
 * and discarded, not parsed) -- see docs/monitor-protocol.md. */
extern USBD_HandleTypeDef hUsbDeviceFS;

void UsbCompositeCdcInit(void);

/* Queue Buf[0..Len) for transmit over channel 0's data-IN endpoint. */
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len);

#endif /* USBD_CDC_COMPOSITE_IF_H */
