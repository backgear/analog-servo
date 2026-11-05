/*
 * ADC1 setup, F103 build. Single channel (PA1, see gpio.h's
 * ANALOG_VELOCITY_ADC_PIN), free-running continuous conversion, no DMA/IRQ
 * -- see Adc.h. Channel/divider choices mirror analog_quadrature's Adc.c
 * (same family, same ADC peripheral), minus the DMA/dual-mode machinery
 * that project needs and this one doesn't.
 */
#include "Adc.h"
#include "gpio.h"
#include "main.h"

static ADC_HandleTypeDef hadc1;

void AdcInit(void) {
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);   /* 72 MHz / 6 = 12 MHz, <= 14 MHz max --
                                                 same divider as analog_quadrature's Adc.c */

  hadc1.Instance = ADC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;   /* free-running -- AdcGetVelocityRaw() just reads
                                                whatever's in the data register, never blocks */
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1U;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  ADC_ChannelConfTypeDef ch = {0};
  ch.Channel = ADC_CHANNEL_1;   /* PA1 -- UNVERIFIED against the real F103 channel table,
                                    same caveat as every other pin map in this project; a
                                    one-#define-plus-one-line fix (gpio.h + this Channel) if
                                    wrong. */
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
}

int32_t AdcGetVelocityRaw(void) { return (int32_t)HAL_ADC_GetValue(&hadc1); }
