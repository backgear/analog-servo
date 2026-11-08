/*
 * USB device/string descriptors -- CDC-ACM, single virtual COM port.
 * VID/PID: ST's standard VCP demo IDs (0483:5740), fine for an internal gadget.
 */
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"

#define USBD_VID     0x0483U
#define USBD_PID     0x5740U
#define USBD_LANGID_STRING 0x0409U   /* en-US */
#define USBD_MANUFACTURER_STRING "drDRO"
#define USBD_PRODUCT_STRING_FS   "torque_servo"
#define USBD_CONFIGURATION_STRING_FS "torque_servo Config"
#define USBD_INTERFACE_STRING_FS     "torque_servo CDC"

/* Factory-programmed 96-bit unique ID ("Device electronic signature"),
 * gives every board a distinct iSerialNumber so a host script can address
 * one specific unit when more than one is plugged in -- same technique
 * every other sibling project's usbd_desc.c uses. Base address differs by
 * family (F103: RM0008, medium-density, 0x1FFFF7E8; L152: RM0038, Cat.3-6,
 * 0x1FF800D0) -- CMSIS's own per-device UID_BASE macro already has the
 * right value on both families (confirmed against the vendored
 * stm32f103xb.h and stm32l152xe.h), so use that directly instead of a
 * second hardcoded constant that could silently drift from it. */
#define STM32_UID_BASE UID_BASE

static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] = {
  0x12, USB_DESC_TYPE_DEVICE,
  0x00, 0x02,          /* bcdUSB = 2.00 */
  /* 0xEF/0x02/0x01 ("Interface Association Descriptor" device class) is
   * what tells Windows to group the composite config descriptor's IAD
   * blocks into separate virtual COM ports instead of enumerating four
   * loose CDC interfaces -- see usbd_cdc_composite.c. Linux/macOS parse IAD
   * either way, Windows specifically needs this at the device-descriptor
   * level. This repo builds composite USB unconditionally (see
   * AGENTS.md/monitor-protocol.md), so this is always the IAD class, not
   * gated behind a flag. */
  0xEF, 0x02, 0x01,    /* bDeviceClass/SubClass/Protocol: Misc/Common/IAD */
  0x40,                /* bMaxPacketSize0 = 64 */
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00, 0x02,          /* bcdDevice = 2.00 */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION
};

static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
  USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  static char uidHex[25];   /* 12 bytes -> 24 hex chars + NUL */
  static const char hex[] = "0123456789ABCDEF";
  const uint8_t *uid = (const uint8_t *)STM32_UID_BASE;
  for (int i = 0; i < 12; i++) {
    uidHex[i * 2]     = hex[uid[i] >> 4];
    uidHex[i * 2 + 1] = hex[uid[i] & 0xFU];
  }
  uidHex[24] = '\0';
  USBD_GetString((uint8_t *)uidHex, USBD_StrDesc, length);
  return USBD_StrDesc;
}
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed;
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
};
