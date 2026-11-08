/*
 * Composite dual-CDC device glue -- the only USB transport in this repo.
 * Channel 0 bridges to Protocol.c's CLI (an ISR feeds a ring, ProtocolPoll()
 * drains it). Channel 1 is Monitor.c's binary telemetry: almost entirely
 * OUT-only from the host's perspective (Monitor.c pushes frames
 * independently, from the main superloop -- see Monitor.h), but DOES accept
 * a tiny one-byte in-band command set the same ISR-feeds-a-ring-then-poll-
 * drains-it way channel 0 does -- see Monitor.h's MonitorRxFromIsrUsb().
 */
#include "usbd_cdc_composite_if.h"
#include "usbd_cdc_composite.h"
#include "usbd_desc.h"
#include "Protocol.h"
#include "Monitor.h"

USBD_HandleTypeDef hUsbDeviceFS;

#define COMPOSITE_RX_BUF_SIZE 64U
#define COMPOSITE_TX_BUF_SIZE 64U

static uint8_t sCh0RxBuf[COMPOSITE_RX_BUF_SIZE];
static uint8_t sCh1RxBuf[COMPOSITE_RX_BUF_SIZE];

static int8_t Ch0_Init(void) {
  USBD_COMPOSITE_CDC_SetTxBuffer(&hUsbDeviceFS, 0U, sCh0RxBuf, 0U);
  USBD_COMPOSITE_CDC_SetRxBuffer(&hUsbDeviceFS, 0U, sCh0RxBuf);
  return USBD_OK;
}
static int8_t Ch1_Init(void) {
  USBD_COMPOSITE_CDC_SetTxBuffer(&hUsbDeviceFS, 1U, sCh1RxBuf, 0U);
  USBD_COMPOSITE_CDC_SetRxBuffer(&hUsbDeviceFS, 1U, sCh1RxBuf);
  return USBD_OK;
}
static int8_t Ch_DeInit(void) { return USBD_OK; }
static int8_t Ch_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  (void)cmd; (void)pbuf; (void)length;
  return USBD_OK;   /* ack everything, same as usbd_cdc_if.c's single-CDC build */
}

static int8_t Ch0_Receive(uint8_t *Buf, uint32_t *Len) {
  ProtocolRxFromIsrUsb(Buf, *Len);
  USBD_COMPOSITE_CDC_SetRxBuffer(&hUsbDeviceFS, 0U, sCh0RxBuf);
  USBD_COMPOSITE_CDC_ReceivePacket(&hUsbDeviceFS, 0U);
  return USBD_OK;
}
/* Channel 1 is telemetry-out mostly -- but see Monitor.h's
 * MonitorRxFromIsrUsb() for the small in-band command set it does accept. */
static int8_t Ch1_Receive(uint8_t *Buf, uint32_t *Len) {
  MonitorRxFromIsrUsb(Buf, *Len);
  USBD_COMPOSITE_CDC_SetRxBuffer(&hUsbDeviceFS, 1U, sCh1RxBuf);
  USBD_COMPOSITE_CDC_ReceivePacket(&hUsbDeviceFS, 1U);
  return USBD_OK;
}

static USBD_CDC_ItfTypeDef sCh0Fops = { Ch0_Init, Ch_DeInit, Ch_Control, Ch0_Receive };
static USBD_CDC_ItfTypeDef sCh1Fops = { Ch1_Init, Ch_DeInit, Ch_Control, Ch1_Receive };

void UsbCompositeCdcInit(void) {
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_COMPOSITE_CDC);
  USBD_COMPOSITE_CDC_RegisterInterfaces(&hUsbDeviceFS, &sCh0Fops, &sCh1Fops);
  USBD_Start(&hUsbDeviceFS);
}

uint8_t CDCComposite_Transmit_Ch0(uint8_t *Buf, uint16_t Len) {
  USBD_COMPOSITE_CDC_SetTxBuffer(&hUsbDeviceFS, 0U, Buf, Len);
  return USBD_COMPOSITE_CDC_TransmitPacket(&hUsbDeviceFS, 0U);
}
