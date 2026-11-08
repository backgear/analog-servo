/*
 * GPIO setup, NUCLEO-L152RE (the only board `main` supports -- see
 * docs/architecture.md). UNVERIFIED WITHOUT THE L152 DATASHEET AF TABLE IN
 * HAND for STEP_PIN's exact AF number; a wrong AF macro name is a compile
 * error, not a silent mis-wire, so worst case this needs a pin changed.
 */
#include "gpio.h"
#include "main.h"

void GpioInit(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  /* STEP (TIM2_ETR): AF push-pull with the TIM2 AF number. */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF1_TIM2;
  g.Pin       = STEP_PIN;
  HAL_GPIO_Init(STEP_GPIO_PORT, &g);

  /* Encoder A/B: TIM4 hardware encoder mode (TI12, 4x decode). */
  g.Alternate = GPIO_AF2_TIM4;
  g.Pin       = ENC_A_PIN | ENC_B_PIN;
  HAL_GPIO_Init(ENC_A_GPIO_PORT, &g);

  /* DIR (EXTI): plain input + interrupt, both edges. */
  g.Mode  = GPIO_MODE_IT_RISING_FALLING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = DIR_PIN;
  HAL_GPIO_Init(DIR_GPIO_PORT, &g);

  /* Encoder Z (index): rising edge -- adjust polarity on the bench. */
  g.Mode  = GPIO_MODE_IT_RISING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = ENC_Z_PIN;
  HAL_GPIO_Init(ENC_Z_GPIO_PORT, &g);

  /* Endstop: falling edge -- assumes an active-low (normally-closed to
   * ground) switch; flip to IT_RISING if the real switch is active-high. */
  g.Mode  = GPIO_MODE_IT_FALLING;
  g.Pull  = GPIO_PULLUP;
  g.Pin   = ENDSTOP_PIN;
  HAL_GPIO_Init(ENDSTOP_GPIO_PORT, &g);

  /* DIR/Z/endstop share EXTI9_5 (pins 5, 8, 9). */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* DAC1 output, both channels: PA4 (positive) + PA5 (negative). Analog
   * mode, no AF number -- DAC output bypasses the digital pad entirely. */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin  = DAC_OUT1_PIN | DAC_OUT2_PIN;
  HAL_GPIO_Init(DAC_OUT1_GPIO_PORT, &g);

  /* Mode C (analog velocity) ADC input: analog mode, pin only -- the ADC1
   * peripheral itself is configured by adc_l1.c. */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin  = ANALOG_VELOCITY_ADC_PIN;
  HAL_GPIO_Init(ANALOG_VELOCITY_ADC_GPIO_PORT, &g);

  /* Mode C reverse pin: pull-up, active-low. */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin  = ANALOG_REVERSE_PIN;
  HAL_GPIO_Init(ANALOG_REVERSE_GPIO_PORT, &g);

  /* Hardware ENABLE: pull-up, active-low -- fails safe (disconnected reads
   * HIGH = "not enabled"). */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin  = HW_ENABLE_PIN;
  HAL_GPIO_Init(HW_ENABLE_GPIO_PORT, &g);

  /* USART2: TX and RX both AF push-pull. */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF7_USART2;
  g.Pin       = UART_TX_PIN;
  HAL_GPIO_Init(UART_TX_GPIO_PORT, &g);
  g.Pull      = GPIO_PULLUP;
  g.Pin       = UART_RX_PIN;
  HAL_GPIO_Init(UART_RX_GPIO_PORT, &g);

  /* Fault output: open-drain, active-low, released at boot. */
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = FAULT_OUT_PIN;
  HAL_GPIO_Init(FAULT_OUT_GPIO_PORT, &g);
  FaultOutWrite(0);

  /* True-zero clamp control: push-pull, released (low) at boot. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = ZERO_CLAMP_PIN;
  HAL_GPIO_Init(ZERO_CLAMP_GPIO_PORT, &g);
  ZeroClampWrite(0);

  /* Tick-active + signal-busy debug pins: plain push-pull outputs. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin   = TICK_ACTIVE_PIN;
  HAL_GPIO_Init(TICK_ACTIVE_GPIO_PORT, &g);
  HAL_GPIO_WritePin(TICK_ACTIVE_GPIO_PORT, TICK_ACTIVE_PIN, GPIO_PIN_RESET);
  g.Pin   = SIGNAL_BUSY_PIN;
  HAL_GPIO_Init(SIGNAL_BUSY_GPIO_PORT, &g);
  HAL_GPIO_WritePin(SIGNAL_BUSY_GPIO_PORT, SIGNAL_BUSY_PIN, GPIO_PIN_RESET);

  /* LD2 (PB15, rerouted via the SB20/SB21 jumper rework -- see
   * docs/bringup.md) -- Nucleo's own onboard LED, active-HIGH. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = LED_PIN;
  HAL_GPIO_Init(LED_GPIO_PORT, &g);
  LedWrite(0);

  /* Brake release: push-pull, released=0 (held/engaged) at boot. */
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = BRAKE_RELEASE_PIN;
  HAL_GPIO_Init(BRAKE_RELEASE_GPIO_PORT, &g);
  BrakeReleaseWrite(0);
}

/* Active-HIGH (Nucleo LED2 wiring). */
void LedWrite(int on) {
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void FaultOutWrite(int active) {
  HAL_GPIO_WritePin(FAULT_OUT_GPIO_PORT, FAULT_OUT_PIN, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void ZeroClampWrite(int asserted) {
  HAL_GPIO_WritePin(ZERO_CLAMP_GPIO_PORT, ZERO_CLAMP_PIN, asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BrakeReleaseWrite(int released) {
  HAL_GPIO_WritePin(BRAKE_RELEASE_GPIO_PORT, BRAKE_RELEASE_PIN, released ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
