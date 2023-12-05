/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void init_timer(void);

TIM_HandleTypeDef    TimHandle;
uint32_t uwPrescalerValue = 0;

void medicion(void);


typedef enum state_enum {
  STATE_MEDICION,
  STATE_MEDIA,
  STATE_ESTIMACION,
  STATE_TIEMPOREMANENTE,
  STATE_DATALOG,
  STATE_REPOSO,
  STATE_ZONASEGURA,
  STATE_ALARMA
} state_enum;


state_enum estado = STATE_MEDICION;
/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  init_timer();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  // Read buffer
  uint8_t rxData[8];
  memset(rxData, 0, 8);

  // Flash LED briefly on reset
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  // while (1)
  // {  
  //   switch (estado)
  //   {//complete with all cases
  //     case STATE_MEDICION:
  //       //adquiere continuamente la tension y corriente. c/21 muestras calcula la mediana
  //       medicion();
  //       break;
  //     case STATE_MEDIA:
  //       // promedio de las medianas
  //       break;
  //     case STATE_ESTIMACION:
  //       // filtro de kalman 
  //       break;
  //     case STATE_TIEMPOREMANENTE:
  //       // calculo de tiempo remanente (ahora esta sin uso)
  //       break;
  //     case STATE_DATALOG:
  //       // guarda en la SD
  //       break;
  //     case STATE_REPOSO:
  //       // estima el soc y x con la EMF inversa
  //       break;
  //     case STATE_ZONASEGURA:
  //       // verifica que los valores adquiridos esten dentro de los limites seguros
  //       break; 
  //     case STATE_ALARMA:
  //       // activa los pines que activaran la llave de corte
  //       break;          
  //     default:
  //       //Standby
  //       break;
  //   }    

    //switch (estado) {
    // case 1:
    //   medicion(); //adquiere continuamente la tension y corriente. c/21 muestras calcula la mediana
    //   break;
    // case 2:
    //   media();  // promedio de las medianas
    //   break;
    // case 3:
    //   estimacion(); // filtro de kalman 
    //   break;
    // case 4:
    //   TiempoRemanente(); // calculo de tiempo remanente (ahora esta sin uso)
    //   break;
    // case 5:
    //   datalog(); // guarda en la SD
    //   break;
    // case 6:
    //   reposo(); // estima el soc y x con la EMF inversa
    //   break;
    // case 7:
    //   ZonaSegura(); // verifica que los valores adquiridos esten dentro de los limites seguros
    //   break; 
    // case 8:
    //   Alarma();  // activa los pines que activaran la llave de corte
    //   break;          
    //default:
      //Standby
    //  break;
    //}
    
    
  /* Infinite loop */
  while (1)
  {    
    // Echo data
    uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
    if (bytesAvailable > 0) {
    	uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
    	if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
            while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    	}
    }

  }
}


void medicion(){
  
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void init_timer(void){
/*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is set to 4 (0x100).
       TIM3CLK = PCLK1*2
       PCLK1   = HCLK/2
    => TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 10000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
