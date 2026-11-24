#include "main.h"
#include "stm32f1xx_it.h"
#include "gpio.h"
#include "Encoder.h"
#include "StepDir.h"
#include "Fault.h"
#include "usart.h"
#include "usbd_cdc_composite_if.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

void NMI_Handler(void) { while (1) { } }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }

/* No RTOS here (bare superloop + ISRs) -- SysTick is ours. */
void SysTick_Handler(void) { HAL_IncTick(); }

/* 1kHz control tick (see main.c's AppControlTick) -- direct register clear +
 * straight to the handler, same convention as the L152 build's
 * TIM6_IRQHandler. TIM1, not TIM6 -- F103 has a real TIM1, see tim.h. */
void TIM1_UP_IRQHandler(void) {
  TIM1->SR = 0U;   /* clear UIF */
  AppControlTick();
}

/* DIR (EXTI8), Z index (EXTI5), and endstop (EXTI9) all share this vector --
 * identical to the L152 build's EXTI9_5_IRQHandler (same HAL_GPIO_EXTI_*
 * API on both families). */
void EXTI9_5_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(ENC_Z_PIN);
  HAL_GPIO_EXTI_IRQHandler(DIR_PIN);
  HAL_GPIO_EXTI_IRQHandler(ENDSTOP_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    case ENC_Z_PIN:      EncoderZIndexIsr(); break;
    case DIR_PIN:        StepDirDirIsr();    break;
    case ENDSTOP_PIN:    FaultEndstopIsr();  break;
    default: break;
  }
}

#ifdef ANALOG_SERVO_HAS_UART
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&hUart1); }
#endif   /* without it, USART1's NVIC vector just keeps the startup file's weak
            default handler -- harmless, since UsartInit() (main.c) never
            unmasks that interrupt on this build either (see its own #ifdef) */

/* F103 has CAN, so this vector is shared (USB_LP_CAN1_RX0_IRQn), unlike the
 * L152 build's USB_LP_IRQn -- no CAN peripheral used here either way. */
void USB_LP_CAN1_RX0_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }
