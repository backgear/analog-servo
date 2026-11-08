/*
 * Composite USB device: two CDC-ACM interfaces on one device. See
 * usbd_cdc_composite.h for why this is hand-rolled instead of reusing the
 * vendored lib/USB_Device's USBD_CDC (that driver hardcodes its endpoint
 * numbers as file-scope macros -- fine for one instance, unusable for two).
 * Structure mirrors lib/USB_Device/src/usbd_cdc.c closely (same
 * Init/DeInit/Setup/DataIn/DataOut/EP0_RxReady shape) with everything
 * parameterized per-channel instead of hardcoded, and one extra descriptor
 * ingredient the single-CDC driver never needed: an Interface Association
 * Descriptor (IAD, 0x0B) ahead of each channel's comm+data interface pair,
 * required for composite CDC devices so Windows groups the right two
 * interfaces into one virtual COM port instead of enumerating four loose
 * interfaces.
 */
#include "usbd_cdc_composite.h"
#include "usbd_ctlreq.h"

#define USB_DESC_TYPE_IAD 0x0BU

typedef struct {
  USBD_CDC_HandleTypeDef ch0;
  USBD_CDC_HandleTypeDef ch1;
  USBD_CDC_ItfTypeDef *ch0Fops;
  USBD_CDC_ItfTypeDef *ch1Fops;
} USBD_COMPOSITE_CDC_HandleTypeDef;

static uint8_t USBD_COMPOSITE_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_COMPOSITE_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t *USBD_COMPOSITE_CDC_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_COMPOSITE_CDC_GetDeviceQualifierDesc(uint16_t *length);

USBD_ClassTypeDef USBD_COMPOSITE_CDC = {
  USBD_COMPOSITE_CDC_Init,
  USBD_COMPOSITE_CDC_DeInit,
  USBD_COMPOSITE_CDC_Setup,
  NULL,                             /* EP0_TxSent */
  USBD_COMPOSITE_CDC_EP0_RxReady,
  USBD_COMPOSITE_CDC_DataIn,
  USBD_COMPOSITE_CDC_DataOut,
  NULL, NULL, NULL,                 /* SOF, IsoIN/OUTIncomplete */
  USBD_COMPOSITE_CDC_GetFSCfgDesc,  /* GetHSConfigDescriptor -- FS-only device
                                        (DEVICE_FS), never actually invoked at
                                        HS, but populated (not NULL) for the
                                        same defensive-completeness reason
                                        usbd_cdc_if.c's single-CDC setup does. */
  USBD_COMPOSITE_CDC_GetFSCfgDesc,
  USBD_COMPOSITE_CDC_GetFSCfgDesc,  /* GetOtherSpeedConfigDescriptor, ditto */
  USBD_COMPOSITE_CDC_GetDeviceQualifierDesc,
};

#define USBD_CDCC_CONFIG_DESC_SIZ 141U

/* One channel block, expanded twice below (comments only on the first) --
 * IAD + comm interface (header/call-mgmt/ACM/union functional descriptors +
 * notification EP) + data interface (OUT/IN bulk EPs). 66 bytes/channel. */
__ALIGN_BEGIN static uint8_t USBD_CDCC_CfgFSDesc[USBD_CDCC_CONFIG_DESC_SIZ] __ALIGN_END = {
  /* Configuration Descriptor */
  0x09, USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(USBD_CDCC_CONFIG_DESC_SIZ), HIBYTE(USBD_CDCC_CONFIG_DESC_SIZ),
  0x04,                     /* bNumInterfaces: 4 (2 channels x comm+data) */
  0x01,                     /* bConfigurationValue */
  0x00,                     /* iConfiguration */
  0xC0,                     /* bmAttributes: self powered */
  0x32,                     /* bMaxPower */

  /* ---- Channel 0 (interfaces 0/1, endpoints 0x81/0x01/0x82) -- same
   * addresses as the single-CDC build, so this channel can carry
   * Protocol.c's CLI unchanged if this ever replaces the single-CDC path. */
  0x08, USB_DESC_TYPE_IAD, CDCC_CH0_COMM_ITF, 0x02, 0x02, 0x02, 0x01, 0x00,

  0x09, USB_DESC_TYPE_INTERFACE, CDCC_CH0_COMM_ITF, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
  0x05, 0x24, 0x00, 0x10, 0x01,                              /* Header Functional */
  0x05, 0x24, 0x01, 0x00, CDCC_CH0_DATA_ITF,                  /* Call Management */
  0x04, 0x24, 0x02, 0x02,                                     /* ACM Functional */
  0x05, 0x24, 0x06, CDCC_CH0_COMM_ITF, CDCC_CH0_DATA_ITF,      /* Union Functional */
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH0_CMD_EP, 0x03,
  LOBYTE(CDCC_CMD_PACKET_SIZE), HIBYTE(CDCC_CMD_PACKET_SIZE), CDCC_FS_BINTERVAL,

  0x09, USB_DESC_TYPE_INTERFACE, CDCC_CH0_DATA_ITF, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH0_OUT_EP, 0x02,
  LOBYTE(CDCC_DATA_PACKET_SIZE), HIBYTE(CDCC_DATA_PACKET_SIZE), 0x00,
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH0_IN_EP, 0x02,
  LOBYTE(CDCC_DATA_PACKET_SIZE), HIBYTE(CDCC_DATA_PACKET_SIZE), 0x00,

  /* ---- Channel 1 (interfaces 2/3, endpoints 0x83/0x03/0x84) -- new, the
   * "monitor" port. Echo-only at the application layer for now, see
   * usbd_cdc_composite_bringup.c's fops. */
  0x08, USB_DESC_TYPE_IAD, CDCC_CH1_COMM_ITF, 0x02, 0x02, 0x02, 0x01, 0x00,

  0x09, USB_DESC_TYPE_INTERFACE, CDCC_CH1_COMM_ITF, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
  0x05, 0x24, 0x00, 0x10, 0x01,
  0x05, 0x24, 0x01, 0x00, CDCC_CH1_DATA_ITF,
  0x04, 0x24, 0x02, 0x02,
  0x05, 0x24, 0x06, CDCC_CH1_COMM_ITF, CDCC_CH1_DATA_ITF,
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH1_CMD_EP, 0x03,
  LOBYTE(CDCC_CMD_PACKET_SIZE), HIBYTE(CDCC_CMD_PACKET_SIZE), CDCC_FS_BINTERVAL,

  0x09, USB_DESC_TYPE_INTERFACE, CDCC_CH1_DATA_ITF, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH1_OUT_EP, 0x02,
  LOBYTE(CDCC_DATA_PACKET_SIZE), HIBYTE(CDCC_DATA_PACKET_SIZE), 0x00,
  0x07, USB_DESC_TYPE_ENDPOINT, CDCC_CH1_IN_EP, 0x02,
  LOBYTE(CDCC_DATA_PACKET_SIZE), HIBYTE(CDCC_DATA_PACKET_SIZE), 0x00,
};

__ALIGN_BEGIN static uint8_t USBD_CDCC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
  USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

static uint8_t *USBD_COMPOSITE_CDC_GetFSCfgDesc(uint16_t *length) {
  *length = sizeof(USBD_CDCC_CfgFSDesc);
  return USBD_CDCC_CfgFSDesc;
}

static uint8_t *USBD_COMPOSITE_CDC_GetDeviceQualifierDesc(uint16_t *length) {
  *length = sizeof(USBD_CDCC_DeviceQualifierDesc);
  return USBD_CDCC_DeviceQualifierDesc;
}

/* channel: 0 or 1 -> the right USBD_CDC_HandleTypeDef inside pClassData. */
static USBD_CDC_HandleTypeDef *ChannelState(USBD_HandleTypeDef *pdev, uint8_t channel) {
  USBD_COMPOSITE_CDC_HandleTypeDef *h = (USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData;
  return channel == 0U ? &h->ch0 : &h->ch1;
}

static uint8_t USBD_COMPOSITE_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  (void)cfgidx;

  USBD_LL_OpenEP(pdev, CDCC_CH0_IN_EP, USBD_EP_TYPE_BULK, CDCC_DATA_PACKET_SIZE);
  pdev->ep_in[CDCC_CH0_IN_EP & 0xFU].is_used = 1U;
  USBD_LL_OpenEP(pdev, CDCC_CH0_OUT_EP, USBD_EP_TYPE_BULK, CDCC_DATA_PACKET_SIZE);
  pdev->ep_out[CDCC_CH0_OUT_EP & 0xFU].is_used = 1U;
  USBD_LL_OpenEP(pdev, CDCC_CH0_CMD_EP, USBD_EP_TYPE_INTR, CDCC_CMD_PACKET_SIZE);
  pdev->ep_in[CDCC_CH0_CMD_EP & 0xFU].is_used = 1U;

  USBD_LL_OpenEP(pdev, CDCC_CH1_IN_EP, USBD_EP_TYPE_BULK, CDCC_DATA_PACKET_SIZE);
  pdev->ep_in[CDCC_CH1_IN_EP & 0xFU].is_used = 1U;
  USBD_LL_OpenEP(pdev, CDCC_CH1_OUT_EP, USBD_EP_TYPE_BULK, CDCC_DATA_PACKET_SIZE);
  pdev->ep_out[CDCC_CH1_OUT_EP & 0xFU].is_used = 1U;
  USBD_LL_OpenEP(pdev, CDCC_CH1_CMD_EP, USBD_EP_TYPE_INTR, CDCC_CMD_PACKET_SIZE);
  pdev->ep_in[CDCC_CH1_CMD_EP & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof(USBD_COMPOSITE_CDC_HandleTypeDef));
  if (pdev->pClassData == NULL) return 1U;

  USBD_COMPOSITE_CDC_HandleTypeDef *h = (USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData;
  h->ch0Fops = NULL;
  h->ch1Fops = NULL;
  h->ch0.TxState = 0U;
  h->ch0.RxState = 0U;
  h->ch0.CmdOpCode = 0xFFU;   /* "no class request pending" -- the vendored
                                  single-CDC USBD_CDC_Init never sets this
                                  explicitly (relies on the heap happening to
                                  be zeroed), which is a latent gap: bRequest
                                  0x00 (CDC_SEND_ENCAPSULATED_COMMAND) is a
                                  valid opcode, so an uninitialized 0 reads as
                                  "pending" and could fire one spurious
                                  Control() call at boot. Harmless here since
                                  Control() acks everything and ignores
                                  content, but explicit costs nothing. */
  h->ch1.TxState = 0U;
  h->ch1.RxState = 0U;
  h->ch1.CmdOpCode = 0xFFU;
  /* pUserData was set by USBD_COMPOSITE_CDC_RegisterInterfaces() before
   * USBD_Start() -- stash it back into the per-channel fops pointers here,
   * since pUserData only has room for one of the two. See that function. */
  USBD_CDC_ItfTypeDef **fopsPair = (USBD_CDC_ItfTypeDef **)pdev->pUserData;
  h->ch0Fops = fopsPair[0];
  h->ch1Fops = fopsPair[1];

  h->ch0Fops->Init();
  h->ch1Fops->Init();

  USBD_LL_PrepareReceive(pdev, CDCC_CH0_OUT_EP, h->ch0.RxBuffer, CDCC_DATA_PACKET_SIZE);
  USBD_LL_PrepareReceive(pdev, CDCC_CH1_OUT_EP, h->ch1.RxBuffer, CDCC_DATA_PACKET_SIZE);
  return 0U;
}

static uint8_t USBD_COMPOSITE_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  (void)cfgidx;

  USBD_LL_CloseEP(pdev, CDCC_CH0_IN_EP);  pdev->ep_in[CDCC_CH0_IN_EP & 0xFU].is_used = 0U;
  USBD_LL_CloseEP(pdev, CDCC_CH0_OUT_EP); pdev->ep_out[CDCC_CH0_OUT_EP & 0xFU].is_used = 0U;
  USBD_LL_CloseEP(pdev, CDCC_CH0_CMD_EP); pdev->ep_in[CDCC_CH0_CMD_EP & 0xFU].is_used = 0U;
  USBD_LL_CloseEP(pdev, CDCC_CH1_IN_EP);  pdev->ep_in[CDCC_CH1_IN_EP & 0xFU].is_used = 0U;
  USBD_LL_CloseEP(pdev, CDCC_CH1_OUT_EP); pdev->ep_out[CDCC_CH1_OUT_EP & 0xFU].is_used = 0U;
  USBD_LL_CloseEP(pdev, CDCC_CH1_CMD_EP); pdev->ep_in[CDCC_CH1_CMD_EP & 0xFU].is_used = 0U;

  if (pdev->pClassData != NULL) {
    USBD_COMPOSITE_CDC_HandleTypeDef *h = (USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData;
    h->ch0Fops->DeInit();
    h->ch1Fops->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  return 0U;
}

/* Interface number -> channel: 0/1 (channel 0's comm/data interfaces) or
 * 2/3 (channel 1's). Class requests (SET_LINE_CODING etc.) target the comm
 * interface specifically, but routing by either interface of a channel is
 * harmless -- the original single-channel driver didn't distinguish comm
 * vs. data interface at all. */
static uint8_t ChannelForInterface(uint16_t wIndex) {
  return (LOBYTE(wIndex) <= CDCC_CH0_DATA_ITF) ? 0U : 1U;
}

static uint8_t USBD_COMPOSITE_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS: {
      uint8_t channel = ChannelForInterface(req->wIndex);
      USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
      USBD_CDC_ItfTypeDef *fops = channel == 0U
        ? ((USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData)->ch0Fops
        : ((USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData)->ch1Fops;
      if (req->wLength) {
        if (req->bmRequest & 0x80U) {
          fops->Control(req->bRequest, (uint8_t *)(void *)hcdc->data, req->wLength);
          USBD_CtlSendData(pdev, (uint8_t *)(void *)hcdc->data, req->wLength);
        } else {
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t)req->wLength;
          USBD_CtlPrepareRx(pdev, (uint8_t *)(void *)hcdc->data, req->wLength);
        }
      } else {
        fops->Control(req->bRequest, (uint8_t *)(void *)req, 0U);
      }
      break;
    }

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;
        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, &ifalt, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;
        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state != USBD_STATE_CONFIGURED) {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }
  return ret;
}

static uint8_t USBD_COMPOSITE_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  if (pdev->pClassData == NULL) return USBD_FAIL;

  uint8_t channel = (epnum == (CDCC_CH1_IN_EP & 0xFU)) ? 1U : 0U;
  USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if ((pdev->ep_in[epnum].total_length > 0U) &&
      ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U)) {
    pdev->ep_in[epnum].total_length = 0U;
    USBD_LL_Transmit(pdev, epnum, NULL, 0U);   /* ZLP */
  } else {
    hcdc->TxState = 0U;
  }
  return USBD_OK;
}

static uint8_t USBD_COMPOSITE_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  if (pdev->pClassData == NULL) return USBD_FAIL;

  uint8_t channel = (epnum == (CDCC_CH1_OUT_EP & 0xFU)) ? 1U : 0U;
  USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
  USBD_CDC_ItfTypeDef *fops = channel == 0U
    ? ((USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData)->ch0Fops
    : ((USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData)->ch1Fops;

  hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);
  fops->Receive(hcdc->RxBuffer, &hcdc->RxLength);
  return USBD_OK;
}

static uint8_t USBD_COMPOSITE_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev) {
  if (pdev->pClassData == NULL) return USBD_OK;
  USBD_COMPOSITE_CDC_HandleTypeDef *h = (USBD_COMPOSITE_CDC_HandleTypeDef *)pdev->pClassData;

  /* Whichever channel's CmdOpCode is pending (the other's stays 0xFF, its
   * reset/idle value -- see USBD_COMPOSITE_CDC_RegisterInterfaces). */
  if (h->ch0.CmdOpCode != 0xFFU) {
    h->ch0Fops->Control(h->ch0.CmdOpCode, (uint8_t *)(void *)h->ch0.data, (uint16_t)h->ch0.CmdLength);
    h->ch0.CmdOpCode = 0xFFU;
  }
  if (h->ch1.CmdOpCode != 0xFFU) {
    h->ch1Fops->Control(h->ch1.CmdOpCode, (uint8_t *)(void *)h->ch1.data, (uint16_t)h->ch1.CmdLength);
    h->ch1.CmdOpCode = 0xFFU;
  }
  return USBD_OK;
}

/* Storage for the fops pair passed to USBD_COMPOSITE_CDC_RegisterInterfaces
 * before USBD_Start() -- pdev->pUserData is a single pointer, so it points
 * here rather than at either fops struct directly; USBD_COMPOSITE_CDC_Init
 * reads it back out into pClassData's ch0Fops/ch1Fops once pClassData
 * exists (Init runs after pUserData is set, during USBD_Start -> Init). */
static USBD_CDC_ItfTypeDef *sFopsPair[2];

uint8_t USBD_COMPOSITE_CDC_RegisterInterfaces(USBD_HandleTypeDef *pdev,
                                               USBD_CDC_ItfTypeDef *ch0Fops,
                                               USBD_CDC_ItfTypeDef *ch1Fops) {
  if (ch0Fops == NULL || ch1Fops == NULL) return USBD_FAIL;
  sFopsPair[0] = ch0Fops;
  sFopsPair[1] = ch1Fops;
  pdev->pUserData = sFopsPair;
  return USBD_OK;
}

uint8_t USBD_COMPOSITE_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel,
                                        uint8_t *pbuff, uint16_t length) {
  USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;
  return USBD_OK;
}

uint8_t USBD_COMPOSITE_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t channel, uint8_t *pbuff) {
  ChannelState(pdev, channel)->RxBuffer = pbuff;
  return USBD_OK;
}

uint8_t USBD_COMPOSITE_CDC_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t channel) {
  USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
  if (pdev->pClassData == NULL) return USBD_FAIL;
  if (hcdc->TxState != 0U) return USBD_BUSY;

  hcdc->TxState = 1U;
  uint8_t inEp = channel == 0U ? CDCC_CH0_IN_EP : CDCC_CH1_IN_EP;
  pdev->ep_in[inEp & 0xFU].total_length = hcdc->TxLength;
  USBD_LL_Transmit(pdev, inEp, hcdc->TxBuffer, (uint16_t)hcdc->TxLength);
  return USBD_OK;
}

uint8_t USBD_COMPOSITE_CDC_ReceivePacket(USBD_HandleTypeDef *pdev, uint8_t channel) {
  if (pdev->pClassData == NULL) return USBD_FAIL;
  USBD_CDC_HandleTypeDef *hcdc = ChannelState(pdev, channel);
  uint8_t outEp = channel == 0U ? CDCC_CH0_OUT_EP : CDCC_CH1_OUT_EP;
  USBD_LL_PrepareReceive(pdev, outEp, hcdc->RxBuffer, CDCC_DATA_PACKET_SIZE);
  return USBD_OK;
}
