#include "gpio.h"

void GpioInit(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();   /* needed for EXTI line routing (AFIO->EXTICR) */

  GPIO_InitTypeDef g = {0};

  /* STEP (TIM2_ETR) + encoder A/B (TIM4 TI1/TI2): peripheral inputs, plain
   * GPIO_MODE_INPUT with a pull -- F103 has no per-pin AF number for inputs
   * (see gpio.h's STEP_PIN comment). Pulled up so an unconnected/idle input
   * doesn't float and generate spurious edges. */
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = STEP_PIN;
  HAL_GPIO_Init(STEP_GPIO_PORT, &g);
  g.Pin   = ENC_A_PIN | ENC_B_PIN;
  HAL_GPIO_Init(ENC_A_GPIO_PORT, &g);   /* ENC_A/ENC_B/DIR/ENDSTOP/ENC_Z share GPIOB */

  /* DIR (EXTI8): both edges -- StepDirDirIsr reads the level, doesn't care
   * which edge fired, it just wants to be notified on every change. */
  g.Mode  = GPIO_MODE_IT_RISING_FALLING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = DIR_PIN;
  HAL_GPIO_Init(DIR_GPIO_PORT, &g);

  /* Encoder Z index (EXTI5): rising edge -- adjust polarity to the real Z
   * pulse on the bench. */
  g.Mode  = GPIO_MODE_IT_RISING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = ENC_Z_PIN;
  HAL_GPIO_Init(ENC_Z_GPIO_PORT, &g);

  /* Endstop (EXTI9): falling edge -- assumes an active-low (normally-closed
   * to ground) switch; flip to IT_RISING if the real switch is active-high. */
  g.Mode  = GPIO_MODE_IT_FALLING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = ENDSTOP_PIN;
  HAL_GPIO_Init(ENDSTOP_GPIO_PORT, &g);

  /* All three EXTI lines above (5, 8, 9) share the EXTI9_5 NVIC vector --
   * one enable covers all of them (see stm32f1xx_it.c's EXTI9_5_IRQHandler). */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* Torque PWM (TIM3_CH1 positive / CH2 negative): AF push-pull, driven by
   * tim_f1.c. Both channels share GPIOA and identical pad settings, so one
   * HAL_GPIO_Init covers both pins. F103 has no on-chip DAC at all -- this
   * is the only output path on this board, not a toggle (see Output.c). */
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin   = TORQUE_PWM_POS_PIN | TORQUE_PWM_NEG_PIN;
  HAL_GPIO_Init(TORQUE_PWM_POS_GPIO_PORT, &g);

  /* Mode C (analog velocity) ADC input: analog mode, pin only -- the ADC1
   * peripheral itself is configured by adc_f1.c, not here. */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin  = ANALOG_VELOCITY_ADC_PIN;
  HAL_GPIO_Init(ANALOG_VELOCITY_ADC_GPIO_PORT, &g);

  /* Mode C reverse pin: pull-up, active-low (see gpio.h). */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin  = ANALOG_REVERSE_PIN;
  HAL_GPIO_Init(ANALOG_REVERSE_GPIO_PORT, &g);

  /* Hardware ENABLE: pull-up, active-low -- open-collector-style external
   * drive (a switch or an open-collector/open-drain output to ground), same
   * convention as DIR/ENDSTOP/ANALOG_REVERSE above. Fails safe
   * (disconnected reads HIGH = "not enabled"). See gpio.h. */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin  = HW_ENABLE_PIN;
  HAL_GPIO_Init(HW_ENABLE_GPIO_PORT, &g);

  /* USART1: TX is an AF output, RX is a plain peripheral input (F103
   * convention, see STEP_PIN comment) -- point-to-point UART, no DE/RE pin. */
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin   = UART_TX_PIN;
  HAL_GPIO_Init(UART_TX_GPIO_PORT, &g);
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = UART_RX_PIN;
  HAL_GPIO_Init(UART_RX_GPIO_PORT, &g);

  /* Fault output: open-drain, active-low, released (floating high, pulled up
   * externally) at boot -- no fault asserted until Fault.c says otherwise. */
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = FAULT_OUT_PIN;
  HAL_GPIO_Init(FAULT_OUT_GPIO_PORT, &g);
  FaultOutWrite(0);

  /* True-zero clamp control: push-pull, released (low) at boot -- Output.c's
   * OutputForceSafe()/OutputSetTorqueCommand() are the only things that ever
   * touch this afterward, and neither runs before this point in main(). */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = ZERO_CLAMP_PIN;
  HAL_GPIO_Init(ZERO_CLAMP_GPIO_PORT, &g);
  ZeroClampWrite(0);

  /* Tick-active + signal-busy debug pins + LED: plain push-pull outputs. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin   = TICK_ACTIVE_PIN;
  HAL_GPIO_Init(TICK_ACTIVE_GPIO_PORT, &g);
  HAL_GPIO_WritePin(TICK_ACTIVE_GPIO_PORT, TICK_ACTIVE_PIN, GPIO_PIN_RESET);
  g.Pin   = SIGNAL_BUSY_PIN;
  HAL_GPIO_Init(SIGNAL_BUSY_GPIO_PORT, &g);
  HAL_GPIO_WritePin(SIGNAL_BUSY_GPIO_PORT, SIGNAL_BUSY_PIN, GPIO_PIN_RESET);

  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = LED_PIN;
  HAL_GPIO_Init(LED_GPIO_PORT, &g);
  LedWrite(0);

  /* Brake release: push-pull, released=0 (held/engaged) at boot -- see
   * gpio.h's BRAKE_RELEASE_PIN comment for why that's the safe default. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = BRAKE_RELEASE_PIN;
  HAL_GPIO_Init(BRAKE_RELEASE_GPIO_PORT, &g);
  BrakeReleaseWrite(0);
}

/* Active-low (Blue Pill wiring): drive LOW to light, HIGH to turn off. */
void LedWrite(int on) {
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Open-drain, active-low: drive LOW (sink current) to assert, release (float
 * high via external pull-up) to clear. */
void FaultOutWrite(int active) {
  HAL_GPIO_WritePin(FAULT_OUT_GPIO_PORT, FAULT_OUT_PIN, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* Active-high: drive HIGH to assert (close) the true-zero clamp, LOW to
 * release it. */
void ZeroClampWrite(int asserted) {
  HAL_GPIO_WritePin(ZERO_CLAMP_GPIO_PORT, ZERO_CLAMP_PIN, asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Active-high = released -- arbitrary convention pending the real SSR/
 * relay/MOSFET driver choice, same "flip the sense if wrong" spirit as
 * ZeroClampWrite() above. See gpio.h's BRAKE_RELEASE_PIN comment. */
void BrakeReleaseWrite(int released) {
  HAL_GPIO_WritePin(BRAKE_RELEASE_GPIO_PORT, BRAKE_RELEASE_PIN, released ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
