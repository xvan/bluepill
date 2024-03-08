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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "cdc_console.h"

/* Private function prototypes -----------------------------------------------*/
typedef enum {
  STATE_IDLE,
  STATE_CAPTURING,
  STATE_TRANSMITTING,
  STATE_ERROR,
} STATE;

typedef enum {
  CMD_QUIT,
  CMD_HELP,
  CMD_START,
  CMD_STOP,
  CMD_UNKNOWN
} CMD;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void ShowState(STATE);

int command_parser(int argc, char **argv, void (* cli_print)(char * str));
int parse_capture(int argc, char **argv, void (* cli_print)(char * str));
int parse_timetest(int argc, char **argv, void (* cli_print)(char * str));

bool parse_integer(char * str, int * value);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Initial autoreload value */
static uint32_t InitialAutoreload = 0;


/* TIM2 Clock */
static uint32_t TimOutClock = 1;

/* Private function prototypes -----------------------------------------------*/
void     Configure_TIM2TimeBase(int);
void     Configure_TIM3TimeBase(void);
  
/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();  

  cdc_console_init();

  ShowState(STATE_IDLE);
  
  while( cdc_console_parse(command_parser) != CMD_QUIT);

  ShowState(STATE_TRANSMITTING);
  while(1);
}

int command_parser(int argc, char **argv, void (* cli_print)(char * str)){
  if (argc == 0)  return CMD_UNKNOWN;

  if ((strcmp(argv[0], "quit") == 0))
    return CMD_QUIT;
  
  if ((strcmp(argv[0], "capture") == 0))
    return parse_capture(argc, argv, cli_print);  

  if ((strcmp(argv[0], "timetest") == 0))
    return parse_timetest(argc, argv, cli_print);
  return CMD_UNKNOWN;
}

int parse_timetest(int argc, char **argv, void (* cli_print)(char * str)){
  if(argc == 2){        
    int timeout;
    if(! parse_integer(argv[1], &timeout)){
      cli_print("Invalid timeout value\r\n");
      return CMD_UNKNOWN;
    }
    
    ShowState(STATE_CAPTURING);

    char str[32];
    int cnt;

    cli_print("Starting timer test\r\n");
    Configure_TIM3TimeBase();
    
    LL_TIM_SetCounter(TIM3, 0);

    cnt = LL_TIM_GetCounter(TIM3);
    sprintf(str, "Pre Counter: %d\r\n", cnt);
    cli_print(str);
    
    LL_TIM_EnableCounter(TIM3);
    HAL_Delay(timeout);
    LL_TIM_DisableCounter(TIM3);
    
    cnt = LL_TIM_GetCounter(TIM3);
    sprintf(str, "Counter: %d\r\n", cnt);
    cli_print(str);

    ShowState(STATE_IDLE);
    return CMD_START;
  }
  else{
    cli_print("Usage: timetest <timeout in milliseconds>\r\n");
    return CMD_UNKNOWN;
  }
  return CMD_UNKNOWN;
}

int parse_capture(int argc, char **argv, void (* cli_print)(char * str)){
  if(argc == 2){        
    int timeout;
    if(! parse_integer(argv[1], &timeout)){
      cli_print("Invalid timeout value\r\n");
      return CMD_UNKNOWN;
    }

    ShowState(STATE_CAPTURING);
    HAL_Delay(timeout);
    ShowState(STATE_IDLE);
    return CMD_START;
  }
  else{
    cli_print("Usage: capture <timeout in milliseconds>\r\n");
    return CMD_UNKNOWN;
  }
  return CMD_UNKNOWN;
}

void ShowState(STATE state) {
  switch (state) {
    case STATE_IDLE:
      Configure_TIM2TimeBase(1);
      break;
    case STATE_CAPTURING:
      LL_TIM_DisableCounter(TIM2);      
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      break;
    case STATE_TRANSMITTING:
      Configure_TIM2TimeBase(3);      
      break;
    case STATE_ERROR:
      Configure_TIM2TimeBase(10);      
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

//frequency in hertz
void  Configure_TIM2TimeBase(int frequency)
{
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  //LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz      */
  /*
    In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM2CLK = PCLK1
      PCLK1 = HCLK
      => TIM2CLK = HCLK = SystemCoreClock
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
  */
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));
  
  /* Set the auto-reload value to have an initial update event frequency */
    /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
  TimOutClock = SystemCoreClock/2;
  
  InitialAutoreload = __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM2), frequency);
  LL_TIM_SetAutoReload(TIM2, InitialAutoreload);
  
  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM2);
  
  /* Configure the NVIC to handle TIM2 update interrupt */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM2);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM2);
}

void  Configure_TIM3TimeBase()
{
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  //LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz      */
  /*
    In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM2CLK = PCLK1
      PCLK1 = HCLK
      => TIM2CLK = HCLK = SystemCoreClock
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
  */

  LL_TIM_SetPrescaler(TIM3, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000));
  
  /* Set the auto-reload value to have an initial update event frequency */
    /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
  // TimOutClock = SystemCoreClock/2;
  
  // InitialAutoreload = __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM2), frequency);
  LL_TIM_SetAutoReload(TIM3, 0xFFFF);
  LL_TIM_EnableMasterSlaveMode(TIM3);


  // LL_TIM_SetPrescaler(TIM3, 0);
  
  // /* Set the auto-reload value to have an initial update event frequency */
  //   /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
  // TimOutClock = SystemCoreClock/2;
  
  // LL_TIM_SetAutoReload(TIM3, 0xFFFF);
  
  /* Enable the update interrupt */
  // LL_TIM_EnableIT_UPDATE(TIM3);
  
  /* Configure the NVIC to handle TIM2 update interrupt */
  // NVIC_SetPriority(TIM3_IRQn, 0);
  // NVIC_EnableIRQ(TIM3_IRQn);
  // /* Force update generation */
  // LL_TIM_GenerateEvent_UPDATE(TIM3);
  
  /* Enable counter */
  //LL_TIM_EnableCounter(TIM3);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  ShowState(STATE_ERROR);
  while (1);  

}

/**
  * @brief  Timer update interrupt processing
  * @param  None
  * @retval None
  */
void TimerUpdate_Callback(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}


bool parse_integer(char * str, int * value){
  char * endptr;
  *value = strtol(str, &endptr, 10);
  return (*endptr == '\0');
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
