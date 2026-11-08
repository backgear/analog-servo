#ifndef USBD_CDC_COMPOSITE_H
#define USBD_CDC_COMPOSITE_H

/*
 * Composite USB device: two independent CDC-ACM ("virtual COM port")
 * interfaces on one device, built by hand -- the vendored lib/USB_Device
 * (ST's older single-CDC-only library, no composite-device builder) can't
 * do this out of the box, see docs/monitor-protocol.md
 * section. Gated behind TORQUE_SERVO_COMPOSITE_USB (off by default, not
 * wired into the real firmware yet) -- per that section's own advice, this
 * is meant to be bench-verified in isolation first (channel 1 here is a
 * bare echo, NOT the real telemetry monitor port) before anything depends
 * on it.
 *
 * Channel 0 keeps the exact same endpoint numbers/interface numbers as the
 * existing single-CDC build (usbd_cdc_if.c via lib/USB_Device's USBD_CDC),
 * so it can still carry Protocol.c's CLI unchanged. Channel 1 is new:
 * interfaces 2 (comm) + 3 (data), endpoints 0x83 (data IN) / 0x03 (data
 * OUT) / 0x84 (notification IN) -- 5 non-EP0 channels total (EP1-EP4), see
 * the design doc's PMA budget math.
 */
#include "usbd_ioreq.h"
#include "usbd_cdc.h"   /* reuses USBD_CDC_HandleTypeDef/USBD_CDC_ItfTypeDef --
                            same per-instance state shape as the single-CDC
                            driver, just two of them here. */

#define CDCC_CH0_COMM_ITF   0U
#define CDCC_CH0_DATA_ITF   1U
#define CDCC_CH1_COMM_ITF   2U
#define CDCC_CH1_DATA_ITF   3U

#define CDCC_CH0_IN_EP      0x81U
#define CDCC_CH0_OUT_EP     0x01U
#define CDCC_CH0_CMD_EP     0x82U
#define CDCC_CH1_IN_EP      0x83U
#define CDCC_CH1_OUT_EP     0x03U
#define CDCC_CH1_CMD_EP     0x84U

#define CDCC_DATA_PACKET_SIZE 64U   /* FS bulk max, same as the single-CDC build */
#define CDCC_CMD_PACKET_SIZE  8U
#define CDCC_FS_BINTERVAL     0x10U

extern USBD_ClassTypeDef USBD_COMPOSITE_CDC;

/* Same two-callback shape as USBD_CDC_RegisterInterface, but takes both
 * channels' fops at once -- there's only one pUserData slot on
 * USBD_HandleTypeDef, so the composite class owns pairing them up
 * internally rather than exposing pUserData directly. */
uint8_t USBD_COMPOSITE_CDC_RegisterInterfaces(USBD_HandleTypeDef *pdev,
                                               USBD_CDC_ItfTypeDef *ch0Fops,
                                               USBD_CDC_ItfTypeDef *ch1Fops);

/* channel selects 0 or 1 -- everything below mirrors usbd_cdc.h's
 * USBD_CDC_Set{Tx,Rx}Buffer/Transmit/ReceivePacket, just channel-scoped. */
uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff, uint16_t length);
uint8_t USBD_COMPOSITE_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff);
uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel);
uint8_t USBD_COMPOSITE_CDC_ReceivePacket(USBD_HandleTypeDef *pdev, uint8_t channel);

#endif /* USBD_CDC_COMPOSITE_H */
