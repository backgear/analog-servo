/*
 * USB Device low-level driver glue (HAL_PCD <-> USBD_LL_*), L152 build.
 * Mostly mirrors usbd_conf_f1.c -- the "basic" USB FS peripheral (register
 * set, PMA/BTABLE layout, USB_CfgTypeDef fields) is the same IP block on
 * F103 and L152, not the OTG core F4/L4 use, so HAL_PCD_* is essentially
 * unchanged across the two. NOT 1:1 though, and this bit otherwise cost a
 * full bring-up session to find: L152's D+ pull-up is a separate
 * SYSCFG->PMC bit (USB_PU) that must be explicitly enabled by software --
 * F103's equivalent is baked into the peripheral's own power-up sequence
 * automatically, so code that only worked on F103 silently never attaches
 * on L152 at all, no matter how correct the wiring is, until
 * HAL_PCD_MspInit() below also sets it. Other differences: no CAN on
 * L152, so the IRQ is USB_LP_IRQn/USB_LP_IRQHandler, not
 * USB_LP_CAN1_RX0_IRQn (see stm32l1xx_it.c).
 */
#include "usbd_conf.h"
#include "usbd_core.h"

PCD_HandleTypeDef hpcd_USB_FS;

/* ---- MSP: clock + NVIC for the USB peripheral ---------------------------- */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_LP_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USB_LP_IRQn);

  /* L1-specific, NOT carried over from usbd_conf_f1.c despite this file's
   * own header comment claiming the two are the same IP block: F103's USB
   * macrocell has its D+ pull-up baked into the peripheral's own power-up
   * sequence (BCDR-equivalent), enabled automatically -- no GPIO/SYSCFG
   * step needed, which is why reusing this composite-USB code unmodified
   * worked fine on F103. L1's D+ pull-up is a SEPARATE bit
   * (SYSCFG->PMC's USB_PU) that software must explicitly set; without it,
   * the host never sees a device attach at all (ST's own L15x errata also
   * notes the fallback/internal pull-up measures ~0.8-0.95k against the
   * 1.5k nominal spec even once enabled -- confirm against a real host if
   * enumeration is still flaky after this). */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_USBPULLUP_ENABLE();
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(USB_LP_IRQn);
}

/* ---- HAL_PCD_*Callback -> USBD_LL_* --------------------------------------- */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef *)hpcd->pData;
  USBD_LL_SetSpeed(pdev, USBD_SPEED_FULL);
  USBD_LL_Reset(pdev);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* ---- USBD_LL_* : bridge to HAL_PCD_* -------------------------------------- */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) return USBD_FAIL;

  /* PMA layout (512 B total): EP0 OUT/IN (8 B each), CDC_IN/CDC_CMD/CDC_OUT --
   * standard CubeMX CDC-VCP placement, same as usbd_conf_f1.c (same PMA size
   * on this USB IP block across F103/L152). This repo builds composite USB
   * unconditionally (two CDC-ACM channels -- see AGENTS.md/monitor-protocol.md:
   * there's no single-CDC fallback and no build flag to toggle it off), so the
   * layout below is always the composite one, not gated behind a flag.
   *
   * Activating EP3/EP4 for channel 1 grows BTABLE from 3 to 5 endpoints'
   * worth (0x00-0x27), which would eat into a single-CDC EP0 OUT placement
   * at 0x18 -- that address is exactly channel 1's data-endpoint BTABLE
   * entry, so it would get scrambled by every EP0 control OUT transfer.
   * EP0 OUT lives at 0x28 instead, clear of the bigger BTABLE. */
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x28);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x58);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, 0xC0);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x01, PCD_SNG_BUF, 0x110);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82, PCD_SNG_BUF, 0x100);
  /* Channel 1's 3 extra endpoints -- required, not optional, since channel 1
   * (binary telemetry) is always present. */
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x83, PCD_SNG_BUF, 0x150);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x03, PCD_SNG_BUF, 0x190);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x84, PCD_SNG_BUF, 0x1D0);

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
  (void)pdev;
  return (HAL_PCD_DeInit(&hpcd_USB_FS) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
  (void)pdev;
  return (HAL_PCD_Start(&hpcd_USB_FS) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
  (void)pdev;
  return (HAL_PCD_Stop(&hpcd_USB_FS) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
  (void)pdev;
  return (HAL_PCD_EP_Open(&hpcd_USB_FS, ep_addr, ep_mps, ep_type) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  return (HAL_PCD_EP_Close(&hpcd_USB_FS, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  return (HAL_PCD_EP_Flush(&hpcd_USB_FS, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  return (HAL_PCD_EP_SetStall(&hpcd_USB_FS, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  return (HAL_PCD_EP_ClrStall(&hpcd_USB_FS, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  if ((ep_addr & 0x80U) != 0U) return hpcd_USB_FS.IN_ep[ep_addr & 0x7FU].is_stall;
  return hpcd_USB_FS.OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
  (void)pdev;
  return (HAL_PCD_SetAddress(&hpcd_USB_FS, dev_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  (void)pdev;
  return (HAL_PCD_EP_Transmit(&hpcd_USB_FS, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  (void)pdev;
  return (HAL_PCD_EP_Receive(&hpcd_USB_FS, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  (void)pdev;
  return HAL_PCD_EP_GetRxCount(&hpcd_USB_FS, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay) { HAL_Delay(Delay); }
