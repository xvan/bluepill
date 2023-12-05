/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_adc.h"

//
void Error_Handler(void);


#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

//#define LED2_PIN                           LL_GPIO_PIN_13
//#define LED2_GPIO_PORT                     GPIOC
//#define LED2_GPIO_CLK_ENABLE()             LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC)

/* TIM2 update interrupt processing */
void TimerUpdate_Callback(void);
void AdcDmaTransferComplete_Callback(void);
void AdcDmaTransferError_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
