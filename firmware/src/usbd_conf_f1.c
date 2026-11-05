/*
 * USB Device low-level driver glue (HAL_PCD <-> USBD_LL_*), F103 build.
 * Mirrors usbd_conf_l1.c -- same underlying USB FS IP block on F103/L152
 * (not the OTG core F4/L4 use), so HAL_PCD_* is essentially unchanged
 * across the two. Difference: F103 has CAN, so the shared vector is
 * USB_LP_CAN1_RX0_IRQn, not L152's USB_LP_IRQn (see stm32f1xx_it.c); and
 * F103's USB D+ pull-up is baked into the peripheral's own power-up
 * sequence automatically, unlike L152's separate SYSCFG->PMC bit.
 */
#include "usbd_conf.h"
#include "usbd_core.h"

PCD_HandleTypeDef hpcd_USB_FS;

/* ---- MSP: clock + NVIC for the USB peripheral ---------------------------- */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
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

  /* PMA layout (512 B total). Composite is the only mode this repo builds
   * (5 active endpoints: EP0 + channel 0's 2 + channel 1's 2), so this
   * layout is unconditional -- not an #ifdef'd alternative to a single-CDC
   * layout that no env here ever selects. BTABLE needs 5*8=40B (0x00-0x27)
   * for those 5 endpoints, so EP0's OUT buffer moves to 0x28, clear of it
   * (placing it at the single-CDC-era 0x18 would alias channel 1's data
   * endpoint's BTABLE entry, corrupting its ADDR_RX/COUNT_RX on every EP0
   * control OUT transfer -- see usbd_conf_l1.c's identical layout/comment). */
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x28);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x58);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, 0xC0);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x01, PCD_SNG_BUF, 0x110);
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82, PCD_SNG_BUF, 0x100);
  /* Channel 1's 3 extra endpoints (0x83/0x03/0x84), placed right after
   * channel 0's block above (which ends at 0x150). */
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
