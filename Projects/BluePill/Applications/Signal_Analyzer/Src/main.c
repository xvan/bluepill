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
#include "embedded_cli.h"

/* Private function prototypes -----------------------------------------------*/
typedef enum {
  STATE_IDLE,
  STATE_CAPTURING,
  STATE_TRANSMITTING,
  STATE_ERROR,
} STATE;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void ShowState(STATE);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Initial autoreload value */
static uint32_t InitialAutoreload = 0;


/* TIM2 Clock */
static uint32_t TimOutClock = 1;
static struct embedded_cli cli;

/* Private function prototypes -----------------------------------------------*/
void     Configure_TIMTimeBase(int);


/* CLI */
typedef struct {
  uint8_t * data;
  size_t buffer_len;
  size_t cursor;
} linear_buffer_object_t;

static void init_linear_buffer(linear_buffer_object_t *buffer_object, uint8_t *data, size_t len){
  buffer_object->data = data;
  buffer_object->buffer_len = len;
  buffer_object->cursor = 0;
}

static bool linear_buffer_putchar(linear_buffer_object_t * buffer_object, char ch){
  if (buffer_object->cursor >= buffer_object->buffer_len){
    return false;
  }
  buffer_object->data[buffer_object->cursor++] = ch;
  return true;
}

static char getch(void);
static void buffer_putch(void *, char, bool);
static void print_buffer(linear_buffer_object_t *);


// static void ugly_prompt(void)
// {
//     uint8_t rxData[8] = {0};
//     char * prompt = "hello world";
//     while (CDC_Transmit_FS(prompt, strlen(prompt)) == USBD_BUSY);
//     // Echo data
//     while(1){
//       uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
//       if (bytesAvailable > 0) {
//         uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
//         if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
//               while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
//               return;
//         }
//       }
//     }
// }
  
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
  MX_USB_DEVICE_Init();

  // Read buffer
  uint8_t rxData[8];
  memset(rxData, 0, 8);
  

  
  HAL_Delay(500);


  
  
  linear_buffer_object_t buffer_object;
  uint8_t buffer[1<<5] = {0};
  init_linear_buffer(&buffer_object, buffer, sizeof(buffer));

  ShowState(STATE_IDLE);
  
  

  embedded_cli_init(&cli, "POSIX> ", buffer_putch, &buffer_object);
  embedded_cli_prompt(&cli);


  bool done = false;
  while (!done) {
      char ch = getch();

      /**
       * If we have entered a command, try and process it
       */
      if (embedded_cli_insert_char(&cli, ch)) {
          int cli_argc;
          char **cli_argv;
          cli_argc = embedded_cli_argc(&cli, &cli_argv);
          // printf("Got %d args\n", cli_argc);
          // for (int i = 0; i < cli_argc; i++) {
          //     printf("Arg %d/%d: '%s'\n", i, cli_argc, cli_argv[i]);
          // }
          done = cli_argc >= 1 && (strcmp(cli_argv[0], "quit") == 0);

          if (!done)
              embedded_cli_prompt(&cli);
      }
    }

    ShowState(STATE_TRANSMITTING);
    while(1);

  // while (1)
  // {    
  //   // Echo data
  //   uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
  //   if (bytesAvailable > 0) {
  //   	uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
  //   	if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
  //           while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
  //   	}
  //   }
  // }
}

void ShowState(STATE state) {
  switch (state) {
    case STATE_IDLE:
      Configure_TIMTimeBase(1);
      break;
    case STATE_CAPTURING:
      LL_TIM_DisableCounter(TIM2);      
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      break;
    case STATE_TRANSMITTING:
      Configure_TIMTimeBase(3);      
      break;
    case STATE_ERROR:
      Configure_TIMTimeBase(10);      
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
void  Configure_TIMTimeBase(int frequency)
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


/**************************/


static char getch(void)
{

  uint8_t buf;
  while (CDC_GetRxBufferBytesAvailable_FS() == 0);  
  if (CDC_ReadRxBuffer_FS(&buf, 1) != USB_CDC_RX_BUFFER_OK){
    Error_Handler();
  }
  return buf;  
}


static void intHandler(int dummy)
{
    (void)dummy;
    embedded_cli_insert_char(&cli, '\x03');
}


static void print_buffer(linear_buffer_object_t * buffer_object){
  while (CDC_Transmit_FS(buffer_object->data, buffer_object->cursor) == USBD_BUSY);
  buffer_object->cursor = 0;
}

static void buffer_putch(void *data, char ch, bool is_last)
{
    linear_buffer_object_t * buffer_object = (linear_buffer_object_t *)data;
    if(! linear_buffer_putchar(buffer_object, ch)){
      print_buffer(buffer_object);
      linear_buffer_putchar(buffer_object, ch);
    }
    if (is_last){
      print_buffer(buffer_object);
    }        
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
