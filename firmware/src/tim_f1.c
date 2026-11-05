/*
 * Timer setup. See tim.h for the timer-role breakdown.
 */
#include "main.h"
#include "tim.h"

/* ---- TIM1: 1kHz control tick ----------------------------------------------
 * TIM1CLK = 72 MHz (APB2, undivided in this project's SystemClock_Config).
 * PSC=71 -> 1 MHz, ARR=999 -> 1 kHz update event. Update-interrupt only (no
 * PWM channels needed on TIM1 here). L152 has no TIM1 at all -- see
 * tim_l1.c's ControlTickTimerInit(), which uses TIM6 instead; see tim.h. */
void ControlTickTimerInit(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM1->PSC = 71U;
  TIM1->ARR = 999U;
  TIM1->CNT = 0U;
  TIM1->SR  = 0U;                 /* clear any stale UIF before enabling the IRQ */
  TIM1->DIER |= TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);   /* higher than USB -- the control tick
                                                  is the timing-critical path */
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
}

/* ---- TIM2: STEP (ETR, external clock mode 2) + DIR (CR1.DIR) --------------
 * TIM2 on F103 is a 16-bit general-purpose timer (CNT/ARR/PSC all 16-bit),
 * unlike a 32-bit free-running timer -- StepDir.c reads/deltas the counter
 * the same wraparound-safe way Encoder.c does for TIM4.
 *
 * UNVERIFIED WITHOUT THE DATASHEET IN HAND: whether CR1.DIR applies to the
 * very next counted edge or is shadow/preload-buffered until an update
 * event, and whether DIR is writable while CEN=1. Cheap bench check before
 * trusting this: hold DIR high, pulse PA0 a known number of times while
 * reading TIM2->CNT over SWD, flip DIR, pulse again, confirm clean
 * increment/decrement with no missed/extra count at the transition. */
void Tim2StepDirInit(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1 = 0;                 /* DIR=0 (up) initially, counter stopped */
  TIM2->ARR = 0xFFFFU;           /* free-running (16-bit -- see above) */
  TIM2->PSC = 0;
  TIM2->CNT = 0;

  TIM2->SMCR = 0;
  TIM2->SMCR |= (0U << TIM_SMCR_ETPS_Pos);   /* ETPS=00: no ETR prescaler, count every edge */
  TIM2->SMCR |= (0U << TIM_SMCR_ETF_Pos);    /* ETF=0000: no digital filter */
  /* TIM2->SMCR |= TIM_SMCR_ETP; */          /* set only if STEP's active edge needs inverting */
  TIM2->SMCR |= TIM_SMCR_ECE;                /* external clock mode 2 -- counts ETRF edges
                                                 directly, independent of the SMS trigger mux */
  TIM2->CR1 |= TIM_CR1_CEN;
}

uint16_t Tim2GetCount(void) { return (uint16_t)TIM2->CNT; }

void Tim2SetDirectionUp(int up) {
  if (up) TIM2->CR1 |= TIM_CR1_DIR;
  else    TIM2->CR1 &= ~TIM_CR1_DIR;
}

/* ---- TIM3: torque-command PWM (poor-man's DAC), two unipolar channels ----
 * TIM3CLK = 72 MHz (APB1 x2 doubling). PSC=0, ARR=2249 (tim.h) -> 32 kHz
 * carrier, ~11-bit (2250-step) duty resolution per channel. Bipolar torque
 * is encoded as two unipolar channels (CH1/PA6 = positive, CH2/PA7 =
 * negative -- see Output.c's OutputSplit()), each sweeping its own full
 * [0, ARR] range rather than one channel biased around mid-scale. This is
 * the only output path on F103 -- no on-chip DAC on this part at all, so
 * unlike the source project there's no TORQUE_SERVO_USE_DAC toggle here. */
static TIM_HandleTypeDef sHtim3;

void Tim3PwmInit(void) {
  __HAL_RCC_TIM3_CLK_ENABLE();

  sHtim3.Instance = TIM3;
  sHtim3.Init.Prescaler = 0U;
  sHtim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  sHtim3.Init.Period = TIM3_PWM_ARR;
  sHtim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sHtim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&sHtim3) != HAL_OK) Error_Handler();

  /* Both channels start at 0% duty (unipolar zero) -- "zero torque" is not
   * mid-scale in this split-channel scheme, see Output.c's OutputSplit(). */
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&sHtim3, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&sHtim3, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  HAL_TIM_PWM_Start(&sHtim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&sHtim3, TIM_CHANNEL_2);
}

void Tim3PwmSetDuty(uint32_t ccrPos, uint32_t ccrNeg) {
  __HAL_TIM_SET_COMPARE(&sHtim3, TIM_CHANNEL_1, ccrPos);
  __HAL_TIM_SET_COMPARE(&sHtim3, TIM_CHANNEL_2, ccrNeg);
}

/* ---- TIM4: hardware quadrature encoder (A=PB6/TI1, B=PB7/TI2) -------------
 * Only the GPIO mode differs from the L152 build (plain INPUT, not AF_PP;
 * see gpio_f1.c) -- HAL_TIM_Encoder_Init itself is portable HAL. */
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
