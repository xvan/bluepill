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
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_exti.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines related to Clock configuration */
/* Uncomment to enable the adequate Clock Source */
/* #define RTC_CLOCK_SOURCE_LSI */
#define RTC_CLOCK_SOURCE_LSE

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

extern I2C_HandleTypeDef I2cHandle;
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

//
void Error_Handler(void);


#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

#define LED_BAR_GPIO_Port GPIOB
#define LED_BAR_Pin0 GPIO_PIN_9
#define LED_BAR_Pin1 GPIO_PIN_8
#define LED_BAR_Pin2 GPIO_PIN_5
#define LED_BAR_Pin3 GPIO_PIN_4
#define LED_BAR_Pin4 GPIO_PIN_3

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

#define CURRENT_SENSOR_GPIO_PORT GPIOA
#define CURRENT_SENSOR_PIN GPIO_PIN_0

#define POW_SENSE_GPIO_PORT GPIOA
#define POW_SENSE_PIN GPIO_PIN_2

#define BAT_SENSE_GPIO_PORT GPIOA
#define BAT_SENSE_PIN GPIO_PIN_1

#define ALARM_GPIO_Port GPIOC
#define ALARM_PIN GPIO_PIN_15

#define CLK_OUT_GPIO_Port GPIOA
#define CLK_OUT_PIN GPIO_PIN_8

#define SWITCH_GPIO_PORT GPIOB
#define SWITH_DISCHARGE_PIN GPIO_PIN_10
#define SWITH_CHARGE_PIN GPIO_PIN_11

#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t) 4300)        /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t) 1430)        /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25_TEMP   ((int32_t)   25)
#define INTERNAL_TEMPSENSOR_V25_VREF   ((int32_t) 3300)

#define USE_TIMEOUT       0

/* TIM2 update interrupt processing */
void TimerUpdate_Callback(void);
void AdcDmaTransferComplete_Callback(void);
void AdcDmaTransferError_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
