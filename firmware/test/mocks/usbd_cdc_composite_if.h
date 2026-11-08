/*
 * Minimal host-side mock of usbd_cdc_composite_if.h -- shadows the real
 * (USB-stack-dependent) header for the native test build. Monitor.c/
 * Protocol.c only need hUsbDeviceFS.pClassData and
 * CDCComposite_Transmit_Ch0; the test file provides the actual
 * implementation (captures transmitted bytes for assertions).
 */
#ifndef USBD_CDC_COMPOSITE_IF_H
#define USBD_CDC_COMPOSITE_IF_H

#include <stdint.h>

#define USBD_OK   0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

typedef struct { void *pClassData; } USBD_HandleTypeDef;
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len);

#endif /* USBD_CDC_COMPOSITE_IF_H */
