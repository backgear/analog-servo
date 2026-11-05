/*
 * USART1 -- plain point-to-point UART, byte-IT RX feeding Protocol.c (same
 * "ISR feeds, superloop drains" split as usbd_cdc_if.c's CDC_Receive_FS).
 */
#include "usart.h"
#include "main.h"
#include "Protocol.h"

UART_HandleTypeDef hUart1;
static uint8_t sRxByte;

void UsartInit(void) {
  __HAL_RCC_USART1_CLK_ENABLE();

  hUart1.Instance = USART1;
  hUart1.Init.BaudRate = 115200U;
  hUart1.Init.WordLength = UART_WORDLENGTH_8B;
  hUart1.Init.StopBits = UART_STOPBITS_1;
  hUart1.Init.Parity = UART_PARITY_NONE;
  hUart1.Init.Mode = UART_MODE_TX_RX;
  hUart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hUart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&hUart1) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_UART_Receive_IT(&hUart1, &sRxByte, 1);
}

void UsartTransmit(const uint8_t *data, uint16_t len) {
  HAL_UART_Transmit(&hUart1, (uint8_t *)data, len, 100U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  ProtocolRxFromIsrUart(&sRxByte, 1U);
  HAL_UART_Receive_IT(&hUart1, &sRxByte, 1);
}
