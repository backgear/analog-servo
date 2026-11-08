/*
 * USART2 -- plain point-to-point UART, byte-IT RX feeding Protocol.c (same
 * "ISR feeds, superloop drains" split as the USB CDC path). USART2, not
 * USART1: the Nucleo-64's ST-Link VCP is hardwired to USART2 (PA2/PA3).
 */
#include "usart.h"
#include "main.h"
#include "Protocol.h"

UART_HandleTypeDef hUart1;
static uint8_t sRxByte;

void UsartInit(void) {
  __HAL_RCC_USART2_CLK_ENABLE();

  hUart1.Instance = USART2;
  hUart1.Init.BaudRate = 115200U;
  hUart1.Init.WordLength = UART_WORDLENGTH_8B;
  hUart1.Init.StopBits = UART_STOPBITS_1;
  hUart1.Init.Parity = UART_PARITY_NONE;
  hUart1.Init.Mode = UART_MODE_TX_RX;
  hUart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hUart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&hUart1) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  HAL_UART_Receive_IT(&hUart1, &sRxByte, 1);
}

void UsartTransmit(const uint8_t *data, uint16_t len) {
  HAL_UART_Transmit(&hUart1, (uint8_t *)data, len, 100U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART2) return;
  ProtocolRxFromIsrUart(&sRxByte, 1U);
  HAL_UART_Receive_IT(&hUart1, &sRxByte, 1);
}
