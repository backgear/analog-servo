#ifndef STM32L1XX_IT_H
#define STM32L1XX_IT_H

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void TIM6_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void USART2_IRQHandler(void);
void USB_LP_IRQHandler(void);

#endif /* STM32L1XX_IT_H */
