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


//#define MASTER_BOARD
//#define SLAVEA
#define SLAVEB

#if defined(MASTER_BOARD) + defined(SLAVEA) + defined(SLAVEB) != 1
#error "Only one of MASTER_BOARD, SLAVEA, or SLAVEB can be defined"
#endif



#define I2C_ADDRESS_A        0x311
#define I2C_ADDRESS_B        0x313

#ifdef SLAVEA
#define I2C_ADDRESS        I2C_ADDRESS_A
#endif

#ifdef SLAVEB
#define I2C_ADDRESS        I2C_ADDRESS_B
#endif

#ifdef MASTER_BOARD
#define I2C_ADDRESS        0x30F
#endif

/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32F1xx*/
#define I2C_SPEEDCLOCK   100000
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****El Mismo Mensaje****\r\n";


/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

I2C_HandleTypeDef I2cHandle;

/* Private function prototypes -----------------------------------------------*/

typedef enum {
  CMD_QUIT,
  CMD_HELP,
  CMD_START,
  CMD_STOP,
  CMD_UNKNOWN,
  CMD_OK,
} CMD;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void Config_I2C(void);
void  Configure_TIMTimeBase(void);
void  slave_manager(void);


int command_parser(int argc, char **argv, void (* cli_print)(char * str));
int parse_read(int argc, char **argv, void (* cli_print)(char * str));
int parse_write(int argc, char **argv, void (* cli_print)(char * str));
int parse_led(int argc, char **argv, void (* cli_print)(char * str));
int parse_reset(int argc, char **argv, void (* cli_print)(char * str));

void EnableLedBlink(void);
void DisableLedBlink(void);
int led_commands(char * cmd);

bool parse_integer(char * str, int * value);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Initial autoreload value */
  
/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */


 void Config_I2C(void){
    /*##-1- Configure the I2C peripheral ######################################*/
    I2cHandle.Instance             = I2Cx;
    I2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;
    I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.OwnAddress2     = 0xFF;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  
    
    if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
 }


int main(void){

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  Config_I2C();  

  
  Configure_TIMTimeBase();

  #ifdef MASTER_BOARD       
    cdc_console_init();
    while(1){
      cdc_console_parse(command_parser);
    }
  #else
    slave_manager();
  #endif
}



void slave_manager(void){
  while(true){
    for(int i = 0; i < RXBUFFERSIZE; i++)
      aRxBuffer[i] = 0;

    if (HAL_I2C_Slave_Receive(&I2cHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 20000) != HAL_OK)
    {
      /* Transfer error in reception process */
      Error_Handler();
    }
    
    if (strcmp((char *)aRxBuffer, "on") == 0){
      DisableLedBlink();    
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    else if (strcmp((char *)aRxBuffer, "off") == 0){
      DisableLedBlink();    
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    else if (strcmp((char *)aRxBuffer, "toggle") == 0){
      DisableLedBlink();    
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    else if (strcmp((char *)aRxBuffer, "blink") == 0){
      EnableLedBlink();
    }
    else if (strcmp((char *)aRxBuffer, "sender_mode") == 0){   
      while (1)
      {
        if(HAL_I2C_Slave_Transmit(&I2cHandle, (uint8_t*)aTxBuffer, strlen(aTxBuffer), 10000)!= HAL_OK)
        { 
          /* Transfer error in transmission process */
          Error_Handler();    
        }
      }
    }        
  }
}

int command_parser(int argc, char **argv, void (* cli_print)(char * str)){
  if (argc == 0)  return CMD_UNKNOWN;
  
  if ((strcmp(argv[0], "read") == 0))
    return parse_read(argc, argv, cli_print);  

  if ((strcmp(argv[0], "write") == 0))
    return parse_write(argc, argv, cli_print);
  
  if ((strcmp(argv[0], "led") == 0))
    return parse_led(argc, argv, cli_print);
  
  if ((strcmp(argv[0], "reset_i2c") == 0))
    return parse_reset(argc, argv, cli_print);
    
  return CMD_UNKNOWN;
}

int parse_reset(int argc, char **argv, void (* cli_print)(char * str)){
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(2);
  __HAL_RCC_I2C1_RELEASE_RESET();
}

int parse_led(int argc, char **argv, void (* cli_print)(char * str)){
  if (argc == 1){
    cli_print("led [on|off|toggle|blink]");
    return CMD_UNKNOWN;
  }

  return led_commands(argv[1]);

}

int led_commands(char * cmd){
  if ((strcmp(cmd, "on") == 0)){
    DisableLedBlink();    
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    return CMD_UNKNOWN;    
  }
  if ((strcmp(cmd, "off") == 0)){
    DisableLedBlink();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    return CMD_UNKNOWN;    
  }
  if ((strcmp(cmd, "toggle") == 0)){
    DisableLedBlink();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    return CMD_UNKNOWN;
  }
  if ((strcmp(cmd, "blink") == 0)){    
    EnableLedBlink();
    return CMD_UNKNOWN;
  }

}

int parse_read(int argc, char **argv, void (* cli_print)(char * str)){
  //read i2c from master
  uint8_t rxbuffer[20] = {0};
  while(1){
    for(int i = 0; i < 20; i++)
      rxbuffer[i] = 0;
    while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)rxbuffer, 20, 10000) != HAL_OK)
      {
        /* Error_Handler() function is called when Timeout error occurs.
          When Acknowledge failure occurs (Slave don't acknowledge it's address)
          Master restarts communication */
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
          Error_Handler();
          continue;
        }
      }
      
    float *pf = (float *) rxbuffer;
    uint32_t *pi = (uint32_t *) rxbuffer;


    float V1 = pf[0];
    float V2 = pf[1];
    
    
    float SOC1 = pf[2];
    float SOC2 = pf[3];

    int state = pi[4];
    
    //format data into string
    char str[100];
    sprintf(str, "V1: %.2f, V2: %.2f, SOC1: %.2f, SOC2: %.2f, State: %d\r\n", V1, V2, SOC1, SOC2, state);
    cli_print(str);
  }
}

int parse_write(int argc, char **argv, void (* cli_print)(char * str)){   

  if (argc < 3){
    cli_print("write [slave 0|1] [value]");
    return CMD_UNKNOWN;
  }

  int slave;
  if (! parse_integer(argv[1], &slave)){
    cli_print("Invalid slave number");
    return CMD_UNKNOWN;
  }

  uint16_t target_address;
  switch (slave)
  {
  case 0:
    target_address = I2C_ADDRESS_A;
    break;
  case 1:
    target_address = I2C_ADDRESS_B;
    break;  
  default:
    cli_print("Invalid slave number");
    return CMD_UNKNOWN;
  }

  while(HAL_I2C_Master_Transmit(&I2cHandle, target_address, (uint8_t *)argv[2], strlen(argv[2]), 10000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
      When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */          
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      EnableLedBlink();
      cli_print("Error writting to I2C");
      return CMD_UNKNOWN;
    }
  }
  return CMD_OK;
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

void  Configure_TIMTimeBase(void)
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
  
  /* Set the auto-reload value to have an initial update event frequency of 10 Hz */
    /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)                 */

  LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_ARR(SystemCoreClock/2, LL_TIM_GetPrescaler(TIM2), 10));
  
  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM2);
  
  /* Configure the NVIC to handle TIM2 update interrupt */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
  
  /* Enable counter */
  
  
  /* Force update generation */
  // LL_TIM_GenerateEvent_UPDATE(TIM2);
}


void EnableLedBlink(){
  LL_TIM_EnableCounter(TIM2);
}

void DisableLedBlink(){
  LL_TIM_DisableCounter(TIM2);
}



static void GPIO_Port_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t GPIO_Mode, uint32_t GPIO_Pull)
{  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_Mode;
  GPIO_InitStruct.Pull = GPIO_Pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();




  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  
  GPIO_Port_Init(LED_GPIO_Port, LED_Pin, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  #ifdef SLAVEA
  HAL_GPIO_WritePin(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHA_SLAVEA, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHB_SLAVEA, GPIO_PIN_RESET);

  GPIO_Port_Init(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHA_SLAVEA, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  GPIO_Port_Init(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHB_SLAVEA, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);  
  #endif

  #ifdef SLAVEB
  HAL_GPIO_WritePin(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHA_SLAVEB, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHB_SLAVEB, GPIO_PIN_RESET);

  GPIO_Port_Init(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHA_SLAVEB, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
  GPIO_Port_Init(EQU_GPIO_PORT, EQU_BALANCE_PIN_CHB_SLAVEB, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);  
  #endif
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{  
  //EnableLedBlink();
}

bool parse_integer(char * str, int * value){
  char * endptr;
  *value = strtol(str, &endptr, 10);
  return (*endptr == '\0');
}

void TimerUpdate_Callback(void){
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
