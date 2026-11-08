/*
 * Timer setup, NUCLEO-L152RE. See tim.h for the role breakdown.
 */
#include "main.h"
#include "tim.h"

/* ---- Control tick: TIM6, 1kHz -----------------------------------------------
 * TIM6CLK = 32 MHz (APB1, undivided -- SYSCLK=32MHz, APB1CLKDivider=DIV1, see
 * main.c's SystemClock_Config). PSC=31 -> 1 MHz, ARR=999 -> 1 kHz update
 * event. TIM6 is a basic timer (update-event-only) -- exactly what a plain
 * periodic tick needs. */
void ControlTickTimerInit(void) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  TIM6->PSC = 31U;
  TIM6->ARR = 999U;
  TIM6->CNT = 0U;
  TIM6->SR  = 0U;                 /* clear any stale UIF before enabling the IRQ */
  TIM6->DIER |= TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);   /* higher than USB -- the control tick
                                               is the timing-critical path */
  HAL_NVIC_EnableIRQ(TIM6_IRQn);

  TIM6->CR1 |= TIM_CR1_CEN;
}

/* ---- TIM2: STEP (ETR, external clock mode 2) + DIR (CR1.DIR) -------------- */
void Tim2StepDirInit(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1 = 0;                 /* DIR=0 (up) initially, counter stopped */
  TIM2->ARR = 0xFFFFU;           /* free-running (16-bit) */
  TIM2->PSC = 0;
  TIM2->CNT = 0;

  TIM2->SMCR = 0;
  TIM2->SMCR |= (0U << TIM_SMCR_ETPS_Pos);   /* ETPS=00: no ETR prescaler, count every edge */
  TIM2->SMCR |= (0U << TIM_SMCR_ETF_Pos);    /* ETF=0000: no digital filter */
  TIM2->SMCR |= TIM_SMCR_ECE;                /* external clock mode 2 */
  TIM2->CR1 |= TIM_CR1_CEN;
}

uint16_t Tim2GetCount(void) { return (uint16_t)TIM2->CNT; }

void Tim2SetDirectionUp(int up) {
  if (up) TIM2->CR1 |= TIM_CR1_DIR;
  else    TIM2->CR1 &= ~TIM_CR1_DIR;
}

/* ---- TIM4: hardware quadrature encoder (A=PB6/TI1, B=PB7/TI2) ------------- */
static TIM_HandleTypeDef sHtim4;

void Tim4EncoderInit(void) {
  __HAL_RCC_TIM4_CLK_ENABLE();

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  sHtim4.Instance = TIM4;
  sHtim4.Init.Prescaler = 0;
  sHtim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  sHtim4.Init.Period = 65535;
  sHtim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sHtim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&sHtim4, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&sHtim4, &sMasterConfig) != HAL_OK) Error_Handler();

  if (HAL_TIM_Encoder_Start(&sHtim4, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
}

uint16_t Tim4GetCount(void) { return (uint16_t)__HAL_TIM_GET_COUNTER(&sHtim4); }
