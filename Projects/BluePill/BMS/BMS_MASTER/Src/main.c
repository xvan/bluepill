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
#include "arm_math.h"
#include "math_functions.h"
#include "estimation.h"

/* Private includes ----------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "circularbuffer_u32.h"


#include "../Tests/data/test_data.h"

/* Adc Stuff -----------------------------------------------------------------*/
#define ADC_CALIBRATION_TIMEOUT_MS ((uint32_t)1)
#define ADC_ENABLE_TIMEOUT_MS ((uint32_t)1)
#define ADC_DISABLE_TIMEOUT_MS ((uint32_t)1)
#define ADC_STOP_CONVERSION_TIMEOUT_MS ((uint32_t)1)
#define ADC_CONVERSION_TIMEOUT_MS ((uint32_t)2)

/* Delay between ADC enable and ADC end of calibration.                     */
/* Delay estimation in CPU cycles: Case of ADC calibration done             */
/* immediately after ADC enable, ADC clock setting slow                     */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_ENABLE_CALIB_CPU_CYCLES (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI ((uint32_t)3300)

/* Definitions of data related to this example */
/* Definition of ADCx conversions data table size */
/* Size of array set to ADC sequencer number of ranks converted,            */
/* to have a rank in each array address.                                    */
#define ANALOG_CHANNELS 3

/* Variables for ADC conversion data */
__IO uint16_t aADCxConvertedData[ANALOG_CHANNELS]; /* ADC group regular conversion data */

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not been started yet (initial state)                  */
__IO uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */

/* Variables for ADC conversion data computation to physical values */
__IO uint16_t voltagePA4 = 0; /* Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
__IO uint16_t voltagePA3 = 0; /* Value of internal voltage reference VrefInt calculated from ADC conversion data (unit: mV) */
__IO int16_t voltagePA2 = 0;  /* Value of temperature calculated from ADC conversion data (unit: degree Celsius) */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void adquisicion(void);
void Configure_TIMTimeBase(void);

void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);

void Switch_Off(void);
void Switch_Charge(void);
void Switch_Discharge(void);


void Configure_DMA(void);
void Configure_ADC(void);
void Activate_ADC(void);

#define BUTTON_MODE_GPIO 0
#define BUTTON_MODE_EXTI 1

/* Number of time base frequencies */
#define TIM_BASE_FREQ_NB 10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Initial autoreload value */
static uint32_t InitialAutoreload = 0;

typedef enum state_enum
{
  STATE_MEDICION = 1,
  STATE_MEDIA,
  STATE_ESTIMACION,
  STATE_TIEMPOREMANENTE,
  STATE_DATALOG,
  STATE_REPOSO,
  STATE_ZONASEGURA,
  STATE_ALARMA
} state_enum;

state_enum estado = STATE_MEDICION;

#define MEDIAN_LENGTH 21
#define MEAN_LENGTH 10
#define SAMPLE_RATE (MEAN_LENGTH * MEDIAN_LENGTH)

u_int32_t median_counter = 0;
u_int32_t mean_counter = 0;

float ChanMedian[ANALOG_CHANNELS][MEDIAN_LENGTH] = {0};
float ChanMean[ANALOG_CHANNELS][MEDIAN_LENGTH] = {0};

//Coeficinetes de escala para cada canal: V = A*Vv + B
float A_Coef[ANALOG_CHANNELS] = {1.0, 1.0, 1.0};
float B_Coef[ANALOG_CHANNELS] = {0.0, 0.0, 0.0};

#define CB_LENGTH2N 5
static CircularBufferObject_t_u32 analogCircularBufferObjects[ANALOG_CHANNELS];
uint32_t analogBuffer[ANALOG_CHANNELS][1 << CB_LENGTH2N] = {0};

void initStructs(void);
inline float voltage_to_measurement(int, float, float);

void main_loop(void);
void test_loop(void);
void blink_loop(void);

/***************************************************/

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

  /* Logic Data Structures */
  initStructs();
  initialize_calculate();
  // Read buffer
  uint8_t rxData[8] = {0};

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  Configure_DMA();
  Configure_ADC();
  Activate_ADC();

  // Flash LED briefly on reset
  LED_On();
  HAL_Delay(500);
  LED_Off();

  Configure_TIMTimeBase();
  // main_loop();
  // test_loop();
blink_loop();
}

void test_loop()
{    
    // init_estimacion_original(V1,V1);
        
    for(size_t i = 0; i < TEST_DATA_ROWS ; i++) {
        estimacion(test_data[i][1], test_data[i][1], test_data[i][0]);
    }

    while (1)
    ;
}

LedBar_StepCounter(){
  static uint8_t counter = 0;
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin0, (counter & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin1, (counter & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin2, (counter & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin3, (counter & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin4, (counter & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  counter++;

}

static void print_buffer(){
  bool read = false;
  uint32_t aux_voltage[3];
  char txData[256];

  while (CircularBuffer_getUnreadSize_u32(&analogCircularBufferObjects[1]) != 0){
    read = true;
    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {      
      CircularBuffer_popFront_u32(&analogCircularBufferObjects[i], &aux_voltage[i]);
    }

    if (read == false) return;

    sprintf(txData, "PA4: %i mV, PA3: %i mV, PA2: %i mV\r\n", aux_voltage[0], aux_voltage[1], aux_voltage[2]);
    int bytesToWrite = strlen(txData);
    while (CDC_Transmit_FS((uint8_t *)txData, bytesToWrite) == USBD_BUSY);
  }
  
}

void blink_loop()
{
  static int counter = 0;
  
  while (1)
  {
    LedBar_StepCounter();    
    HAL_Delay(1000);

    print_buffer();

    counter++;

    if(counter == 5){
      LED_On();
      Switch_Discharge();
    }

    if(counter == 10){
      LED_Off();
      Switch_Off();
      counter = 0;
    }
  }
}

void main_loop()
{
  while (1)
  {
    
    /* DATA AQUISITION *****************************************************/
    if( CircularBuffer_getUnreadSize_u32(&analogCircularBufferObjects[1]) == 0 )
      continue;

    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {      
      uint32_t aux_voltage;
      CircularBuffer_popFront_u32(&analogCircularBufferObjects[i], &aux_voltage);       
      ChanMedian[i][median_counter] = voltage_to_measurement(aux_voltage, A_Coef[i], B_Coef[i]);
    }
    median_counter++;

    /* MEDIAN CALCULATION *******************************************************/

    if (median_counter != MEDIAN_LENGTH) 
      continue;
    
    median_counter = 0;
    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {
      ChanMean[i][mean_counter] = median(ChanMedian[i], MEDIAN_LENGTH);
    }
    mean_counter++;
      
    if (mean_counter != MEAN_LENGTH)
      continue;      
    
    /* MEAN CALCULATION *********************************************************/
    mean_counter = 0;
    float32_t aux_mean[ANALOG_CHANNELS];
    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {      
      arm_mean_f32(ChanMean[i], MEAN_LENGTH, &aux_mean[i]);
    }      

    char txData[256];
    sprintf(txData, "PA4: %f mV, PA3: %f mV, PA2: %f mV\r\n", aux_mean[0], aux_mean[1], aux_mean[2]);
    int bytesToWrite = strlen(txData);

    while (CDC_Transmit_FS((uint8_t *)txData, bytesToWrite) == USBD_BUSY)
      ;

    /* CURRENT VALUES */
    float32_t V1 = aux_mean[0];
    float32_t V2 = aux_mean[1] - aux_mean[0];
    float32_t I = aux_mean[3];
    
    estimacion(V1, V2, I);
  }
}

  // while (1)
  // {
  //   ubDmaTransferStatus = 0;

  //   while (ubDmaTransferStatus != 1)
  //   {
  //   }

  //   /* Computation of ADC conversions raw data to physical values             */
  //   /* using LL ADC driver helper macro.                                      */
  //   voltagePA4 = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
  //   voltagePA3 = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
  //   voltagePA2 = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[2], LL_ADC_RESOLUTION_12B);

  //   char txData[256];

  //   sprintf(txData, "PA4: %d mV, PA3: %d mV, PA2: %d mV\r\n", voltagePA4, voltagePA3, voltagePA2);
  //   int bytesToWrite = strlen(txData);

  //   while (CDC_Transmit_FS((uint8_t *)txData, bytesToWrite) == USBD_BUSY);

  //   // Echo data
  //   // uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
  //   // if (bytesAvailable > 0)
  //   // {
  //   //   uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
  //   //   if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK)
  //   //   {
  //   //     while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY)
  //   //       ;
  //   //   }
  //   // }
  // }

void LED_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(Period);
  }
}


void Configure_TIMTimeBase(void)
{
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  // LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

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

  InitialAutoreload = __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM2), SAMPLE_RATE);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

void AdcDmaTransferComplete_Callback()
{
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;
  adquisicion();
}

void GPIO_Port_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
  
  GPIO_Port_Init(LED_GPIO_Port, LED_Pin);
  
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin0);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin1);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin2);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin3);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin4);

  GPIO_Port_Init(SWITCH_GPIO_Port, SWITCH_PIN_Discharge);
  GPIO_Port_Init(SWITCH_GPIO_Port, SWITCH_PIN_Charge);
}

void Switch_Off()
{
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Discharge, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Charge, GPIO_PIN_RESET);
}

void Switch_Charge(){
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Discharge, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Charge, GPIO_PIN_SET);
}

void Switch_Discharge(){
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Charge, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_Port, SWITCH_PIN_Discharge, GPIO_PIN_SET);
}

void Configure_DMA(void)
{
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable DMA interruptions */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1); /* DMA IRQ lower priority than ADC IRQ */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /*## Configuration of DMA ##################################################*/
  /* Enable the peripheral clock of DMA */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Configure the DMA transfer */
  /*  - DMA transfer in circular mode to match with ADC configuration:        */
  /*    DMA unlimited requests.                                               */
  /*  - DMA transfer from ADC without address increment.                      */
  /*  - DMA transfer to memory with address increment.                        */
  /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
  /*    ADC resolution 12 bits.                                               */
  /*  - DMA transfer to memory by half-word to match with ADC conversion data */
  /*    buffer variable type: half-word.                                      */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                            LL_DMA_MODE_CIRCULAR |
                            LL_DMA_PERIPH_NOINCREMENT |
                            LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_HALFWORD |
                            LL_DMA_MDATAALIGN_HALFWORD |
                            LL_DMA_PRIORITY_HIGH);

  /* Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&aADCxConvertedData,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       ANALOG_CHANNELS);

  /* Enable DMA transfer interruption: transfer complete */
  LL_DMA_EnableIT_TC(DMA1,
                     LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1,
                     LL_DMA_CHANNEL_1);

  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_1);
}

void Configure_ADC(void)
{  
  /*## Configuration of GPIO used by ADC channels ############################*/

  /* Note: On this STM32 device, ADC1 channel 4 is mapped on GPIO pin PA.04 */

  /* Enable GPIO Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);

  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority(ADC1_IRQn, 0); /* ADC IRQ greater priority than DMA IRQ */
  NVIC_EnableIRQ(ADC1_IRQn);

  /*## Configuration of ADC ##################################################*/

  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/

  /* Enable ADC clock (core clock) */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC measurement path to internal channels */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

    
    /*## Configuration of ADC hierarchical scope: multimode ####################*/

    /* Set ADC multimode configuration */
    // LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_INDEPENDENT);

    /* Set ADC multimode DMA transfer */
    // LL_ADC_SetMultiDMATransfer(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_REG_DMA_EACH_ADC);

    /* Set ADC multimode: delay between 2 sampling phases */
    // LL_ADC_SetMultiTwoSamplingDelay(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE);
  }

  /*## Configuration of ADC hierarchical scope: ADC instance #################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC conversion data alignment */
    // LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    /* Set Set ADC sequencers scan mode, for all ADC groups                   */
    /* (group regular, group injected).                                       */
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
  }

  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

    /* Set ADC group regular trigger polarity */
    // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

    /* Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* Set ADC group regular sequencer */
    /* Note: On this STM32 series, ADC group regular sequencer is             */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerLength()".                               */

    /* Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);

    /* Set ADC group regular sequencer discontinuous mode */
    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

    /* Set ADC group regular sequence: channel on the selected sequence rank. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_0);
  }

  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, ADC state is checked anyway with standard requirements    */
  /*       (refer to description of this function).                           */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC group injected trigger source */
    // LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);

    /* Set ADC group injected trigger polarity */
    // LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    /* Set ADC group injected conversion trigger  */
    // LL_ADC_INJ_SetTrigAuto(ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);

    /* Set ADC group injected sequencer */
    /* Note: On this STM32 series, ADC group injected sequencer is             */
    /*       fully configurable: sequencer length and each rank               */
    /*       affectation to a channel are configurable.                       */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_INJ_SetSequencerLength()".                               */

    /* Set ADC group injected sequencer length and scan direction */
    // LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);

    /* Set ADC group injected sequencer discontinuous mode */
    // LL_ADC_INJ_SetSequencerDiscont(ADC1, LL_ADC_INJ_SEQ_DISCONT_DISABLE);

    /* Set ADC group injected sequence: channel on the selected sequence rank. */
    // LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_4);
  }

  /*## Configuration of ADC hierarchical scope: channels #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Set ADC channels sampling time */
    /* Note: Considering interruption occurring after each ADC group          */
    /*       regular sequence conversions                                     */
    /*       (IT from DMA transfer complete),                                 */
    /*       select sampling time and ADC clock with sufficient               */
    /*       duration to not create an overhead situation in IRQHandler.      */
    /* Note: Set long sampling time due to internal channels (VrefInt,        */
    /*       temperature sensor) constraints.                                 */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_SetChannelSamplingTime()".                               */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_239CYCLES_5);    
  }

  /*## Configuration of ADC transversal scope: analog watchdog ###############*/

  /* Note: On this STM32 series, there is only 1 analog watchdog available.    */

  /* Set ADC analog watchdog: channels to be monitored */
  // LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_DISABLE);

  /* Set ADC analog watchdog: thresholds */
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B));
  // LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, 0x000);

  /*## Configuration of ADC transversal scope: oversampling ##################*/

  /* Note: Feature not available on this STM32 series */

  /*## Configuration of ADC interruptions ####################################*/
  /* Enable interruption ADC group regular end of sequence conversions */
  LL_ADC_EnableIT_EOS(ADC1);

  /* Note: in this example, ADC group regular end of conversions              */
  /*       (number of ADC conversions defined by DMA buffer size)             */
  /*       are notified by DMA transfer interruptions).                       */
  /*       ADC interruptions of end of conversion are enabled optionally,     */
  /*       as demonstration purpose in this example.                          */
}

void Activate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
#endif                  /* USE_TIMEOUT */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 series, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Enable ADC */
    LL_ADC_Enable(ADC1);

    /* Delay between ADC enable and ADC start of calibration.                 */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);

/* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (Timeout-- == 0)
        {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
        }
      }
#endif /* USE_TIMEOUT */
    }
  }

  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */

  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: No operation on ADC group injected performed here.                 */
  /*       ADC group injected conversions to be performed after this function */
  /*       using function:                                                    */
  /*       "LL_ADC_INJ_StartConversion();"                                    */
}

void AdcDmaTransferError_Callback()
{
  /* Error detected during DMA transfer */
  LED_Blinking(LED_BLINK_ERROR);
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

void TimerUpdate_Callback(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  LL_ADC_REG_StartConversionSWStart(ADC1);
  // LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
}

#ifdef USE_FULL_ASSERT
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


/***************************************************************/
/* CODIGO PARA BMS *********************************************/



inline float voltage_to_measurement(int voltage,float A, float B)
{
  return A * voltage + B;
}


void adquisicion()
{ 
  for (int i = 0; i < ANALOG_CHANNELS; i++)
  {
    int chanVoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[i], LL_ADC_RESOLUTION_12B);
    CircularBuffer_pushBack_u32(&analogCircularBufferObjects[i], chanVoltage);
  }
}


    // Vv1[cm] = voltage_to_measurement(voltageCH1,CH1_A,CH1_B);
    // Vv2[cm] = voltage_to_measurement(voltageCH2,CH2_A,CH2_B);
    // Ii[cm] = voltage_to_measurement(voltageCH3,CH3_A,CH3_B);

    // cm++;

    // if(cm == MEDIAN_LENGTH){
    //   cm = 0;
    //   float V1 = median(Vv1,MEDIAN_LENGTH);
    //   float V2 = median(Vv2,MEDIAN_LENGTH);
    //   float I = median(Ii,MEDIAN_LENGTH);    
    // }

void initStructs()
{
  for (int i = 0; i < ANALOG_CHANNELS; i++)
  {
    CircularBuffer_init_u32(&analogCircularBufferObjects[i], analogBuffer[i], CB_LENGTH2N);    
  }
}