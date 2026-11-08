/*
 * ADC1 setup, L152 build. Single channel (PA1, see gpio.h's
 * ANALOG_VELOCITY_ADC_PIN), free-running continuous conversion, no DMA/IRQ
 * -- see Adc.h.
 *
 * L152's ADC has only one clock source: the chip's main HSI 16 MHz RC
 * oscillator (ADC_CLOCK_ASYNC_* -- confirmed against the vendored
 * stm32l1xx_hal_adc.c, no synchronous/PCLK option exists on this family,
 * unlike F1/L4). AdcInit() turns HSI on without touching the HSE-derived
 * SYSCLK/PLL config main.c's SystemClock_Config already has running --
 * HAL_RCC_OscConfig() only acts on the oscillator type(s) named in
 * OscillatorType, so this is safe to call after SystemClock_Config() without
 * re-deriving SYSCLK.
 *
 * Also unlike F1/L4, this generation's ADC has no self-calibration routine
 * at all (confirmed: no HAL_ADCEx_Calibration_Start() exists in the
 * vendored stm32l1xx_hal_adc_ex.h) -- HAL_ADC_Start() is the first and only
 * thing needed to begin converting.
 */
#include "Adc.h"
#include "gpio.h"
#include "main.h"

static ADC_HandleTypeDef hadc1;

void AdcInit(void) {
  RCC_OscInitTypeDef osc = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc1.Init.ContinuousConvMode = ENABLE;   /* free-running -- AdcGetVelocityRaw() just reads
                                                whatever's in the data register, never blocks */
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1U;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  ADC_ChannelConfTypeDef ch = {0};
  ch.Channel = ADC_CHANNEL_1;   /* PA1 -- UNVERIFIED against the real L152 channel table,
                                    same caveat as every other pin map in this project. */
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_96CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();

  HAL_ADC_Start(&hadc1);
}

int32_t AdcGetVelocityRaw(void) { return (int32_t)HAL_ADC_GetValue(&hadc1); }
