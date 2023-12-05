/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 ******************************************************************************
  */

#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USB_LP_CAN_RX0_IRQHandler(void);
void TIM2_IRQHandler(void);


void WWDG_IRQHandler(void);           			/* Window Watchdog interrupt                        */
void PVD_IRQHandler(void);            			/* PVD through EXTI line detection interrupt        */
void TAMPER_IRQHandler(void);         			/* Tamper interrupt                                 */
void RTC_IRQHandler(void);            			/* RTC global interrupt                             */
void FLASH_IRQHandler(void);          			/* Flash global interrupt                           */
void RCC_IRQHandler(void);            			/* RCC global interrupt                             */
void EXTI0_IRQHandler(void);          			/* EXTI Line0 interrupt                             */
void EXTI1_IRQHandler(void);          			/* EXTI Line1 interrupt                             */
void EXTI2_IRQHandler(void);          			/* EXTI Line2 interrupt                             */
void EXTI3_IRQHandler(void);          			/* EXTI Line3 interrupt                             */
void EXTI4_IRQHandler(void);          			/* EXTI Line4 interrupt                             */
void DMA1_Channel1_IRQHandler(void);  			/* DMA1 Channel1 global interrupt                   */
void DMA1_Channel2_IRQHandler(void);  			/* DMA1 Channel2 global interrupt                   */
void DMA1_Channel3_IRQHandler(void);  			/* DMA1 Channel3 global interrupt                   */
void DMA1_Channel4_IRQHandler(void);  			/* DMA1 Channel4 global interrupt                   */
void DMA1_Channel5_IRQHandler(void);  			/* DMA1 Channel5 global interrupt                   */
void DMA1_Channel6_IRQHandler(void);  			/* DMA1 Channel6 global interrupt                   */
void DMA1_Channel7_IRQHandler(void);  			/* DMA1 Channel7 global interrupt                   */
void ADC1_2_IRQHandler(void);         			/* ADC1 and ADC2 global interrupt                   */
void USB_HP_CAN_TX_IRQHandler(void);  			/* USB High Priority or CAN TX interrupts           */
void CAN_RX1_IRQHandler(void);        			/* CAN RX1 interrupt                                */
void CAN_SCE_IRQHandler(void);        			/* CAN SCE interrupt                                */
void EXTI9_5_IRQHandler(void);        			/* EXTI Line[9:5] interrupts                        */
void TIM1_BRK_IRQHandler(void);       			/* TIM1 Break interrupt                             */
void TIM1_UP_IRQHandler(void);        			/* TIM1 Update interrupt                            */
void TIM1_TRG_COM_IRQHandler(void);   			/* TIM1 Trigger and Commutation interrupts          */
void TIM1_CC_IRQHandler(void);        			/* TIM1 Capture Compare interrupt                   */
void TIM3_IRQHandler(void);           			/* TIM3 global interrupt                            */
void TIM4_IRQHandler(void);           			/* TIM4 global interrupt                            */




  // .word	I2C1_EV_IRQHandler        			/* I2C1 event interrupt                             */
  // .word	I2C1_ER_IRQHandler        			/* I2C1 error interrupt                             */
  // .word	I2C2_EV_IRQHandler        			/* I2C2 event interrupt                             */
  // .word	I2C2_ER_IRQHandler        			/* I2C2 error interrupt                             */
  // .word	SPI1_IRQHandler           			/* SPI1 global interrupt                            */
  // .word	SPI2_IRQHandler           			/* SPI2 global interrupt                            */
  // .word	USART1_IRQHandler         			/* USART1 global interrupt                          */
  // .word	USART2_IRQHandler         			/* USART2 global interrupt                          */
  // .word	USART3_IRQHandler         			/* USART3 global interrupt                          */
  // .word	EXTI15_10_IRQHandler      			/* EXTI Line[15:10] interrupts                      */
  // .word	RTCAlarm_IRQHandler       			/* RTC Alarms through EXTI line interrupt           */
  // .word	0                         			/* Reserved                                         */
  // .word	TIM8_BRK_IRQHandler       			/* TIM8 Break interrupt                             */
  // .word	TIM8_UP_IRQHandler        			/* TIM8 Update interrupt                            */
  // .word	TIM8_TRG_COM_IRQHandler   			/* TIM8 Trigger and Commutation interrupts          */
  // .word	TIM8_CC_IRQHandler        			/* TIM8 Capture Compare interrupt                   */
  // .word	ADC3_IRQHandler           			/* ADC3 global interrupt                            */
  // .word	FSMC_IRQHandler           			/* FSMC global interrupt                            */
  // .word	SDIO_IRQHandler           			/* SDIO global interrupt                            */
  // .word	TIM5_IRQHandler           			/* TIM5 global interrupt                            */
  // .word	SPI3_IRQHandler           			/* SPI3 global interrupt                            */
  // .word	UART4_IRQHandler          			/* UART4 global interrupt                           */
  // .word	UART5_IRQHandler          			/* UART5 global interrupt                           */
  // .word	TIM6_IRQHandler           			/* TIM6 global interrupt                            */
  // .word	TIM7_IRQHandler           			/* TIM7 global interrupt                            */
  // .word	DMA2_Channel1_IRQHandler  			/* DMA2 Channel1 global interrupt                   */
  // .word	DMA2_Channel2_IRQHandler  			/* DMA2 Channel2 global interrupt                   */
  // .word	DMA2_Channel3_IRQHandler  			/* DMA2 Channel3 global interrupt                   */
  // .word	DMA2_Channel4_5_IRQHandler			/* DMA2 Channel4 and DMA2 Channel5 global interrupt */

void I2C1_EV_IRQHandler(void);       			/* I2C1 event interrupt                             */
void I2C1_ER_IRQHandler(void);       			/* I2C1 error interrupt                             */
void I2C2_EV_IRQHandler(void);       			/* I2C2 event interrupt                             */
void I2C2_ER_IRQHandler(void);       			/* I2C2 error interrupt                             */
void SPI1_IRQHandler(void);          			/* SPI1 global interrupt                            */
void SPI2_IRQHandler(void);          			/* SPI2 global interrupt                            */
void USART1_IRQHandler(void);        			/* USART1 global interrupt                          */
void USART2_IRQHandler(void);        			/* USART2 global interrupt                          */
void USART3_IRQHandler(void);        			/* USART3 global interrupt                          */
void EXTI15_10_IRQHandler(void);     			/* EXTI Line[15:10] interrupts                      */
void RTCAlarm_IRQHandler(void);      			/* RTC Alarms through EXTI line interrupt           */
void TIM8_BRK_IRQHandler(void);      			/* TIM8 Break interrupt                             */
void TIM8_UP_IRQHandler(void);       			/* TIM8 Update interrupt                            */
void TIM8_TRG_COM_IRQHandler(void);  			/* TIM8 Trigger and Commutation interrupts          */
void TIM8_CC_IRQHandler(void);       			/* TIM8 Capture Compare interrupt                   */
void ADC3_IRQHandler(void);          			/* ADC3 global interrupt                            */
void FSMC_IRQHandler(void);          			/* FSMC global interrupt                            */
void SDIO_IRQHandler(void);          			/* SDIO global interrupt                            */
void TIM5_IRQHandler(void);          			/* TIM5 global interrupt                            */
void SPI3_IRQHandler(void);          			/* SPI3 global interrupt                            */
void UART4_IRQHandler(void);         			/* UART4 global interrupt                           */
void UART5_IRQHandler(void);         			/* UART5 global interrupt                           */
void TIM6_IRQHandler(void);          			/* TIM6 global interrupt                            */
void TIM7_IRQHandler(void);          			/* TIM7 global interrupt                            */
void DMA2_Channel1_IRQHandler(void); 			/* DMA2 Channel1 global interrupt                   */
void DMA2_Channel2_IRQHandler(void); 			/* DMA2 Channel2 global interrupt                   */
void DMA2_Channel3_IRQHandler(void); 			/* DMA2 Channel3 global interrupt                   */
void DMA2_Channel4_5_IRQHandler(void);			/* DMA2 Channel4 and DMA2 Channel5 global interrupt */





#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
