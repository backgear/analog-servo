/*
 * Minimal host-side mock of usbd_cdc_composite_if.h -- shadows the real
 * (USB-stack-dependent) header for simmy_commutation_learn_integration_main.c's
 * build of the real main.c. Extends test/mocks/usbd_cdc_composite_if.h's
 * own minimal shape with UsbCompositeCdcInit() too -- main.c's own real
 * main() (never actually called, but still fully compiled -- see this
 * driver's own top comment) calls it directly.
 */
#ifndef USBD_CDC_COMPOSITE_IF_H
#define USBD_CDC_COMPOSITE_IF_H

#include <stdint.h>

#define USBD_OK   0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

typedef struct { void *pClassData; } USBD_HandleTypeDef;
extern USBD_HandleTypeDef hUsbDeviceFS;

void    UsbCompositeCdcInit(void);
uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len);

#endif /* USBD_CDC_COMPOSITE_IF_H */
