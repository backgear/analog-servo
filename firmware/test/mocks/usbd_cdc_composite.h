/*
 * Minimal host-side mock of usbd_cdc_composite.h -- shadows the real
 * (USB-stack-dependent) header for the native test build. Monitor.c only
 * needs the Set/TransmitPacket pair for channel 1; the test file provides
 * the actual implementation.
 */
#ifndef USBD_CDC_COMPOSITE_H
#define USBD_CDC_COMPOSITE_H

#include <stdint.h>
#include "usbd_cdc_composite_if.h"   /* reuses USBD_HandleTypeDef/USBD_OK/BUSY/FAIL */

uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff, uint16_t length);
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel);

#endif /* USBD_CDC_COMPOSITE_H */
