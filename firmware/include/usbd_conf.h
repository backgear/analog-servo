/*
 * USB Device middleware config -- CubeMX-style glue for the vendored
 * lib/USB_Device (Core + CDC).
 */
#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES     4U   /* 2 channels x comm+data, see usbd_cdc_composite.h */
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       0x100U
#define USBD_SUPPORT_USER_STRING_DESC 0U
#define USBD_SELF_POWERED           1U
#define USBD_DEBUG_LEVEL           0U

#define USBD_CDC_INTERVAL          2000U

#define DEVICE_FS 0U   /* USBD_Init's "id" param: single device, full-speed only */

#define USBD_malloc  malloc
#define USBD_free    free
#define USBD_memset  memset
#define USBD_memcpy  memcpy

#define USBD_UsrLog(...) do {} while (0)
#define USBD_ErrLog(...) do {} while (0)
#define USBD_DbgLog(...) do {} while (0)

#endif /* USBD_CONF_H */
