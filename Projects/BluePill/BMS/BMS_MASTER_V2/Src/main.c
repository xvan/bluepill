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
#include "cdc_console.h"

#include "MST_CURR_calibration.h"
#include "MST_vbat_calibration.h"
#include "MST_vpow_calibration.h"


#include "../Tests/data/test_data.h"
/* I2c stuff -----------------------------------------------------------------*/
#define I2C_ADDRESS        0x30F
#define I2C_ADDRESS_A      0x311
#define I2C_ADDRESS_B      0x313

/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32F1xx*/
#define I2C_SPEEDCLOCK   100000
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****El Mismo Mensaje****\r\n";


/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

I2C_HandleTypeDef I2cHandle;
RTC_HandleTypeDef RtcHandle;

static void Config_I2C(void);

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

/* Identity interpolation*/
const arm_linear_interp_instance_f32 Identity_Interpolation = {2, 0, 1, (float32_t []){ 0, 1 }};

/* Voltage Interpolation Tables */
arm_linear_interp_instance_f32 const * Interpolation_Tables[ANALOG_CHANNELS]={
  &MST_CURR_calibration, 
  &MST_vbat_calibration,
  &MST_vpow_calibration  
  };


/* Variables for ADC conversion data */
__IO uint16_t aADCxConvertedData[ANALOG_CHANNELS]; /* ADC group regular conversion data */

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not been started yet (initial state)                  */
__IO uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static inline void adquisicion(void);
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
void Config_RTC(void);

void RTC_TimeShow(uint8_t* showtime);
/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */

void RTC_SetDateTime(RTC_DateTypeDef  * sdatestructure, RTC_TimeTypeDef  * stimestructure);

int command_parser(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_help(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_read_adc(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_switch(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_read_slave(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_test_calibration(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_set_equ(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_unkown(void (* cli_print)(const char * str));
int handle_command_date(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_led(int argc, char **argv, void (* cli_print)(const char * str));
int handle_command_write(int argc, char **argv, void (* cli_print)(const char * str));

void averaging_loop(void (* cli_print)(const char * str));

bool parse_integer(char * str, int * value);
int led_commands(char * cmd);

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

typedef enum {
  CMD_QUIT,
  CMD_HELP,
  CMD_UNKNOWN,
  CMD_OK,
} CMD;

#define MSG_SIZE 24

typedef struct {
  float32_t V1;
  float32_t V2;
  float32_t I;
  float32_t SOC1;
  float32_t SOC2;
  int32_t STATUS;
} MSG_SLAVE;

state_enum estado = STATE_MEDICION;

uint8_t equ_state = 0;

#define MEDIAN_LENGTH 21
#define MEAN_LENGTH 10
#define SAMPLE_RATE (MEAN_LENGTH * MEDIAN_LENGTH)

u_int32_t median_counter = 0;
u_int32_t mean_counter = 0;

float ChanMedian[ANALOG_CHANNELS][MEDIAN_LENGTH] = {0};
float ChanMean[ANALOG_CHANNELS][MEDIAN_LENGTH] = {0};

//Coeficinetes de escala para cada canal: V = A*Vv + B

#define CB_LENGTH2N 5
static CircularBufferObject_t_u32 analogCircularBufferObjects[ANALOG_CHANNELS];
uint32_t analogBuffer[ANALOG_CHANNELS][1 << CB_LENGTH2N] = {0};

void initStructs(void);
inline float voltage_to_measurement(int, arm_linear_interp_instance_f32 const *);

void Get_Next_ADC(float32_t * mean_samples_out);

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
  

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  Configure_DMA();
  Configure_ADC();
  Activate_ADC();
  Config_I2C();
  
  Config_RTC();
  

  //cdc_console_init();
  

  // Flash LED briefly on reset
  LED_On();
  HAL_Delay(500);
  LED_Off();

  Configure_TIMTimeBase();


  cdc_console_init();
  while(1){
     cdc_console_parse(command_parser);
  }
  main_loop();  
  //blink_loop();
}

void Config_RTC(void){
  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Asynch Prediv  = Automatic calculation of prediv for 1 sec timebase
  */
  RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static void LedBar_StepCounter(){
  static uint8_t counter = 0;
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin0, (counter & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin1, (counter & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin2, (counter & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin3, (counter & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BAR_GPIO_Port, LED_BAR_Pin4, (counter & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  counter++;
}

int command_parser(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc == 0) return CMD_UNKNOWN;
  
  if ((strcmp(argv[0], "help") == 0))
    return handle_command_help(argc, argv, cli_print);  

  if ((strcmp(argv[0], "read_adc") == 0))
    return handle_command_read_adc(argc, argv, cli_print);

  if ((strcmp(argv[0], "switch") == 0))
    return handle_command_switch(argc, argv, cli_print);
  
  if ((strcmp(argv[0], "test_calibration") == 0))
    return handle_command_test_calibration(argc, argv, cli_print);

  if ((strcmp(argv[0], "read_slave") == 0))
    return handle_command_read_slave(argc, argv, cli_print);

  if ((strcmp(argv[0], "set_equ") == 0))
    return handle_command_set_equ(argc, argv, cli_print);

  if ((strcmp(argv[0], "date") == 0))
    return handle_command_date(argc, argv, cli_print);

  if ((strcmp(argv[0], "led") == 0))
    return handle_command_led(argc, argv, cli_print);

  if ((strcmp(argv[0], "write") == 0))
    return handle_command_write(argc, argv, cli_print);
  
  return handle_command_unkown(cli_print);    
}



int handle_command_date(int argc, char **argv, void (* cli_print)(const char * str)){
  

  if (argc == 1){
    uint8_t aShowTime[50];
    RTC_TimeShow(aShowTime);
    sprintf(aShowTime, "%s\r\n", aShowTime);
    cli_print(aShowTime);    
    return CMD_OK;
  }

  if (argc != 7){
    cli_print("Usage: date <year> <month> <day> <hour> <minute> <second>\r\n");
    return CMD_UNKNOWN;
  }
  
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  parse_integer(argv[1], & sdatestructure.Year);
  parse_integer(argv[2], & sdatestructure.Month);
  parse_integer(argv[3], & sdatestructure.Date);
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  parse_integer(argv[4], & stimestructure.Hours);
  parse_integer(argv[5], & stimestructure.Minutes);
  parse_integer(argv[6], & stimestructure.Seconds);  

  RTC_SetDateTime(&sdatestructure, &stimestructure);

  return CMD_OK;
}

bool parse_integer(char * str, int * value){
  char * endptr;
  *value = strtol(str, &endptr, 10);
  return (*endptr == '\0');
}

int handle_command_led(int argc, char **argv, void (* cli_print)(const char * str)){
    if (argc == 1){
    cli_print("led [on|off|toggle]");
    return CMD_UNKNOWN;
    }

  return led_commands(argv[1]);
}

int led_commands(char * cmd){
  if ((strcmp(cmd, "on") == 0)){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    return CMD_UNKNOWN;    
  }
  if ((strcmp(cmd, "off") == 0)){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    return CMD_UNKNOWN;    
  }
  if ((strcmp(cmd, "toggle") == 0)){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    return CMD_UNKNOWN;  
  }
}

int handle_command_write(int argc, char **argv, void (* cli_print)(const char * str)){
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
        cli_print("Error writting to I2C\r\n");
        return CMD_UNKNOWN;
      }
    }
    return CMD_OK;
}


int handle_command_set_equ(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc != 3){
        cli_print("Usage: set_equ <CH1 ON|OFF> <CH2 ON|OFF>\r\n");
        return CMD_UNKNOWN;
  }

  int ch1 = 0;
  int ch2 = 0;
  if (strcmp(argv[1], "ON") == 0)
    ch1 = 1;
  else if (strcmp(argv[1], "OFF") == 0)
    ch1 = 0;
  else{
    cli_print("Invalid value for CH1. Use ON or OFF\r\n");
    return CMD_UNKNOWN;
  }

  if (strcmp(argv[2], "ON") == 0)
    ch2 = 1;
  else if (strcmp(argv[2], "OFF") == 0)
    ch2 = 0;
  else{
    cli_print("Invalid value for CH2. Use ON or OFF\r\n");
    return CMD_UNKNOWN;
  }

  equ_state = (ch2 << 1) | ch1;

  return CMD_OK;
}

#define SLAVE_COUNT 2
int slave_adress[SLAVE_COUNT] = {I2C_ADDRESS_A, I2C_ADDRESS_B};



int handle_command_read_slave(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc != 2){
        cli_print("Usage: read_slave <number_of_samples>\r\n");
        return CMD_UNKNOWN;
  }

  int samples = 0;
  if (!parse_integer(argv[1], &samples) || samples < 0){
    cli_print("Invalid number of samples\r\n");
    return CMD_UNKNOWN;
  }  

  uint8_t rxbuffer[MSG_SIZE] = {0};
  // float *pf = (float *) rxbuffer;
  // uint32_t *pi = (uint32_t *) rxbuffer;
  MSG_SLAVE * msg = (MSG_SLAVE *) rxbuffer;
  
  while(samples--){
    for (int slave = 0; slave < SLAVE_COUNT; slave++){    
      for(int i = 0; i < 20; i++)
        rxbuffer[i] = 0;

      while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)slave_adress[slave], (uint8_t *)rxbuffer, MSG_SIZE, 10000) != HAL_OK){
        /* Error_Handler() function is called when Timeout error occurs.
          When Acknowledge failure occurs (Slave don't acknowledge it's address)
          Master restarts communication */
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
          return;
        }
      }

      
      //print msg
      uint8_t timestamp[50];
      RTC_TimeShow(timestamp);
      char str[256];

      

      //read the inverse of msg->STATUS = equ_state  | (vuelta << 2);
      sprintf(str, "%s, SLAVE: %d, V1: %.2f, V2: %.2f, I: %.2f, SOC1: %.2f, SOC2: %.2f, Equ_State: %d, Vuelta: %d\r\n",timestamp,  slave, msg->V1, msg->V2, msg->I, msg->SOC1, msg->SOC2, ( msg->STATUS & 0x3), (msg->STATUS >> 2));

      cli_print(str);
    }


    for (int slave = 0; slave < SLAVE_COUNT; slave++){
      while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)slave_adress[slave], &equ_state, 1, 10000) != HAL_OK){
      /* Error_Handler() function is called when Timeout error occurs.
        When Acknowledge failure occurs (Slave don't acknowledge it's address)
        Master restarts communication */
        if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {
          return;
        }
      }
    }
}
}

int handle_command_test_calibration(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc != 3){
        cli_print("Usage: test_calibration hi_voltage low_voltage\r\n");
        return CMD_UNKNOWN;
  }

  while(true){        
    /* DATA AQUISITION *****************************************************/
    float32_t aux_mean[ANALOG_CHANNELS];
    Get_Next_ADC(aux_mean);


    char txData[256];
    sprintf(txData, "%s %s v_low=%f v_hi=%f\r\n", argv[1], argv[2], aux_mean[2],aux_mean[3]-aux_mean[2]);
    cli_print(txData);
    
    return CMD_OK;
  }

  return CMD_UNKNOWN;
}

int handle_command_switch(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc != 2){
        cli_print("Usage: switch [charge|discharge|off]\r\n");
        return CMD_UNKNOWN;
  }

  if ((strcmp(argv[1], "charge") == 0)){
    Switch_Charge();
    return CMD_OK;
  }

  if ((strcmp(argv[1], "discharge") == 0)){
    Switch_Discharge();
    return CMD_OK;
  }

  if ((strcmp(argv[1], "off") == 0)){
    Switch_Off();
    return CMD_OK;
  }

  cli_print("Usage: switch [charge|discharge|off]\r\n");
  return CMD_UNKNOWN;
}

int handle_command_unkown(void (* cli_print)(const char * str)){
  cli_print("Unknown command. Type 'help' for a list of commands.\r\n");
  return CMD_UNKNOWN;
}

int handle_command_help(int argc, char **argv, void (* cli_print)(const char * str)){
  cli_print("Available commands:\r\n");
  cli_print("help: Display this help message\r\n");
  cli_print("read_adc: Read the ADC values\r\n");
  cli_print("switch [charge|discharge|off]: Switch the charge and discharge relays\r\n");
  cli_print("test_calibration hi_voltage low_voltage: Test the calibration of the voltage sensors\r\n");
  cli_print("read_slave <number_of_samples>: Read the slave device\r\n");  
  cli_print("led [on|off|toggle]\r\n");
  cli_print("write <slave> <command>\r\n");  

  return CMD_HELP;
}

int handle_command_read_adc(int argc, char **argv, void (* cli_print)(const char * str)){
  if (argc != 1){
        cli_print("Usage: read_adc\r\n");
        return CMD_UNKNOWN;
  }

  averaging_loop(cli_print);
  return CMD_OK;
}



void test_loop(){    
    //init_estimacion_original(V1,V1);
        
    for(size_t i = 0; i < TEST_DATA_ROWS ; i++) {
        estimacion(test_data[i][1], test_data[i][1], test_data[i][0]);
    }

    while(1);
}

void Get_Next_ADC(float32_t * mean_samples_out){
  while(1){
    if( CircularBuffer_getUnreadSize_u32(&analogCircularBufferObjects[1]) == 0 )
      continue;

    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {      
      uint32_t aux_voltage;
      CircularBuffer_popFront_u32(&analogCircularBufferObjects[i], &aux_voltage);       
      ChanMedian[i][median_counter] = aux_voltage;
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
    
    for (int i = 0; i < ANALOG_CHANNELS; i++)
    {      
      arm_mean_f32(ChanMean[i], MEAN_LENGTH, &mean_samples_out[i]);
    }
    break;
  }
}


void main_loop(){
  
  int vuelta = 0;
  float32_t el_SOC = 0.5;
  const float32_t h = 1/3600.0;
  const float32_t Q = 7.5;
  
  while(1){
    float32_t aux_mean[ANALOG_CHANNELS];
    Get_Next_ADC(aux_mean);

    float32_t Vbat = aux_mean[1];
    float32_t Vpow = aux_mean[2];
    float32_t I = aux_mean[0];    

    el_SOC -= I * h / Q;
  }
}

void averaging_loop(void (* cli_print)(const char * str)){
  int vuelta = 0;
  float32_t aux_mean[ANALOG_CHANNELS];
  float32_t aux_interp[ANALOG_CHANNELS];
  
  Get_Next_ADC(aux_mean);

  for(int i =0; i < ANALOG_CHANNELS; i++){
    aux_interp[i]=voltage_to_measurement(aux_mean[i], Interpolation_Tables[i]);
  }

  char txData[256];  
  sprintf(txData, "Vbat=%f Vpow=%f I=%f V1=%f V2=%f V3=%f\r\n", aux_mean[1], aux_mean[2], aux_mean[0], aux_interp[1], aux_interp[2], aux_interp[0]);
  cli_print(txData);  
}

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
  
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin0, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin1, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin2, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin3, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  GPIO_Port_Init(LED_BAR_GPIO_Port, LED_BAR_Pin4, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  

  GPIO_Port_Init(SWITCH_GPIO_PORT, SWITH_CHARGE_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
  GPIO_Port_Init(SWITCH_GPIO_PORT, SWITH_DISCHARGE_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);

  GPIO_Port_Init(ALARM_GPIO_Port, ALARM_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

}

void Switch_Off()
{
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_DISCHARGE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_CHARGE_PIN, GPIO_PIN_RESET);
}

void Switch_Charge(){
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_DISCHARGE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_CHARGE_PIN, GPIO_PIN_SET);
}

void Switch_Discharge(){
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_CHARGE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SWITCH_GPIO_PORT, SWITH_DISCHARGE_PIN, GPIO_PIN_SET);
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

void Configure_ADC(void)
{  
  /*## Configuration of GPIO used by ADC channels ############################*/

  /* Note: On this STM32 device, ADC1 channel 4 is mapped on GPIO pin PA.04 */

  /* Enable GPIO Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(CURRENT_SENSOR_GPIO_PORT, CURRENT_SENSOR_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(BAT_SENSE_GPIO_PORT, BAT_SENSE_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(POW_SENSE_GPIO_PORT, POW_SENSE_PIN, LL_GPIO_MODE_ANALOG);  


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
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);    
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
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
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_239CYCLES_5);        
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
//  __disable_irq();
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



inline float voltage_to_measurement(int voltage, arm_linear_interp_instance_f32 const * pInterpolationTable)
{
  return arm_linear_interp_f32(pInterpolationTable, voltage);
}


static inline void adquisicion()
{ 
  for (int i = 0; i < ANALOG_CHANNELS; i++)
  {
    int chanVoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, aADCxConvertedData[i], LL_ADC_RESOLUTION_12B);
    CircularBuffer_pushBack_u32(&analogCircularBufferObjects[i], chanVoltage);
  }
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void initStructs(){
  for (int i = 0; i < ANALOG_CHANNELS; i++)
  {
    CircularBuffer_init_u32(&analogCircularBufferObjects[i], analogBuffer[i], CB_LENGTH2N);    
  }
}

/**
  * @brief  Display the current time.
  * @param  showtime : pointer to buffer
  * @retval None
  */
void RTC_TimeShow(uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
} 

void RTC_SetDateTime(RTC_DateTypeDef  * sdatestructure, RTC_TimeTypeDef  * stimestructure)
{
  
  if(HAL_RTC_SetDate(&RtcHandle,sdatestructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  if(HAL_RTC_SetTime(&RtcHandle,stimestructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}


