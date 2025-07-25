/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#define ARM_MATH_CM4	//Definimos esto porque estamos usando el núcleo ARM-Cortex M4
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
State_t Current_State = STATE_REFRESH; 	// Se fija el estado por defecto

//Variables para llevar el tracking del valor del PWM de cada RGB
uint8_t PWM_ROJO = 99;
uint8_t PWM_VERDE = 99;
uint8_t PWM_AZUL = 99;

//Variable que lleva el contador completo del taxímetro
uint16_t contador_Taximetro = 0;
//Variable que lleva el dato del dígito que se enciende en el taxímetro
uint8_t digito = 0;
//Se inicializa una variable que lleva el tiempo en ms que ha pasado desde el inicio
uint32_t contador_Tiempo = 0;

//Variables auxiliares para llevar los digitos que se deben mostrar en el taxímetro
uint8_t miles_contador_Taximetro 	= 0;
uint8_t centenas_contador_Taximetro = 0;
uint8_t decenas_contador_Taximetro 	= 0;
uint8_t unidades_contador_Taximetro = 0;

//Variables auxiliares para la comunciación USART
uint16_t rx_buffer_length = 64;
uint8_t rx_Buffer_A[RX_BUFFER_MAX_LENGTH]; // Buffer de recepción A
uint8_t rx_Buffer_B[RX_BUFFER_MAX_LENGTH]; // Buffer de recepción B

volatile Paquete_Datos paquete_listo = {.buffer = NULL, .size = 0};
volatile BufferActivo buffer_activo_DMA = BUFFER_A;
uint8_t*  rx_buffer;
uint16_t  rx_size;

uint16_t ADC_Buffer [ADC_BUFFER_MAX_LENGTH]; // Array para almacenar los valores del ADC, provenientes del DMA
ADC_Sampling_Freq_t ARR_timer_4 = LOW;
float32_t ADC_Float_Buffer [ADC_BUFFER_MAX_LENGTH]; 	// Array para almacenar los valores del ADC hechos float
float32_t FFT_Buffer		[ADC_BUFFER_MAX_LENGTH]; 	//Buffer para el output de la FFT
float32_t FFT_Magnitudes 	[ADC_BUFFER_MAX_LENGTH/2];	//Buffer para las magnitudes de la FFT
arm_rfft_fast_instance_f32 rfft_instance; 						//Se crea una instancia que es requerida por la función RFFT
uint16_t fft_Length = ADC_BUFFER_MAX_LENGTH;					//Tamaño de la FFT que se va a realizar, 2048 por defecto
uint8_t esta_activo_ADC = 0;									//Variable auxiliar para saber si el ADC está prendio' o no
uint16_t longitud_ADC = 2048;									//Variable para llevar rastro del tamaño del ADC

//Variables para las conversiones ADC del joystick
uint16_t JOYSTICK_Buffer [2]; // X, Y
float JOYSTICK_X = 0;
float JOYSTICK_Y = 0;

//Le asignamos a una variable el mensaje de help, tipo static porque pues, no lo vamos a modificar nunca
static const char help_msg[] =
    "Opciones:\r\n"
    "1.  PWM [0-99] LED RGB:                         RGB <ColorPWM>, Ej: RGB R08\r\n"
    "2.  Configurar periodo del Blinky:              Config_Blinky_Period <Period (ms)>\r\n"
    "3.  Configurar tiempo de muestreo:              Config_ADC_Sampling_Freq <Velocidad>\r\n"
	"Velocidad: LOW = 44.1 KHz , MEDIUM = 48 KHz, HIGH = 96 KHz, ULTRA = 128 KHz\r\n"
	"4.  Iniciar ADC:                                Start_ADC\r\n"
	"5.  Detener ADC:                                Stop_ADC\r\n"
	"6.  Imprimir señal ADC (raw):                   Print_ADC\r\n"
    "7.  Configurar tamaño de la FFT:                Config_FFT_Size\r\n"
    "8.  Imprimir espectro FFT:                      Print_FFT\r\n"
    "9.  Imprimir configuración del equipo:          Print_Config\r\n"
    "10. Imprimir valores clave de la FFT:           Print_FFT_Features\r\n"
	"11. Medir desempeno pintar digito:              Performance_7Seg\r\n"
	"12. Medir desempeno envío de mensaje:           Performance_Message\r\n";

// Measurement variables
volatile uint32_t start_cycles, end_cycles, elapsed_cycles;
char tx_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void printhelp(void);
void FSM_update(State_t State);
void start_adc(void);
void setNumeroSieteSegmentos(uint8_t numero);
void measure_performance(void);
uint32_t DWT_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); 	// Inicia el Timer 2 para interrupciones
  HAL_TIM_Base_Start_IT(&htim3); 	// Inicia el Timer 3 para interrupciones
  HAL_TIM_Base_Start(&htim4); 		// Inicia el Timer 4 para la conversión del ADC
  //Iniciamos los PWM que se utilizaran para el RGB
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_SET); // Apagamos todos los digitos para evitar el fantasma
  HAL_GPIO_WritePin(DigitoD1_GPIO_Port, DigitoD1_Pin, GPIO_PIN_SET); // Buuuu
  HAL_GPIO_WritePin(DigitoD2_GPIO_Port, DigitoD2_Pin, GPIO_PIN_SET); // Buuuu
  HAL_GPIO_WritePin(DigitoD3_GPIO_Port, DigitoD3_Pin, GPIO_PIN_SET); // Buuuu

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_A, rx_buffer_length); // Configura la recepción DMA con IDLE

  start_adc();	//ADC encendido por defecto
  printhelp();	//Imprimimos la ayuda
  DWT_Init();	//Iniciamos el DWT

  arm_rfft_fast_init_f32(&rfft_instance, fft_Length);	//Iniciamos la instancia al incio para no llamarla cada printFFT, Lo haremos con 2048 por defecto
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	/*
    uint8_t resultado_2 = (((!!0xC)<<0b11)|(~0x45));
    uint8_t resultado_3 = ((0b00111100 & 0xAC) << 0x3);
    uint8_t resultado_5 = ((0xCAFE & 0x0FF0) >> 7);
    uint8_t resultado_6 = (1<<4) + (0x3<<4) - 10 + (0b11 << 0x6);
    uint8_t resultado_7 = 84 | 53;
    uint8_t resultado_8 = (2921 & 0xD6);
    uint8_t resultado_operacion = 0b01100110 ^ 0b00101011;
    */


  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  FSM_update(Current_State); // Llama a la función de actualización del FSM

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 101-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLed_GPIO_Port, UserLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DigitoD3_Pin|DigitoD2_Pin|DigitoD1_Pin|SieteSegmentosLEDA_Pin
                          |SieteSegmentosLEDE_Pin|SieteSegmentosLEDG_Pin|SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SieteSegmentosLEDF_Pin|SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Switch_Encoder_Pin Clk_Encoder_Pin */
  GPIO_InitStruct.Pin = Switch_Encoder_Pin|Clk_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLed_Pin */
  GPIO_InitStruct.Pin = UserLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BotonTasaRefrescoIncremento_Pin */
  GPIO_InitStruct.Pin = BotonTasaRefrescoIncremento_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BotonTasaRefrescoIncremento_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DigitoD3_Pin DigitoD2_Pin DigitoD1_Pin SieteSegmentosLEDA_Pin
                           SieteSegmentosLEDE_Pin SieteSegmentosLEDG_Pin SieteSegmentosLEDD_Pin */
  GPIO_InitStruct.Pin = DigitoD3_Pin|DigitoD2_Pin|DigitoD1_Pin|SieteSegmentosLEDA_Pin
                          |SieteSegmentosLEDE_Pin|SieteSegmentosLEDG_Pin|SieteSegmentosLEDD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Data_Encoder_Pin */
  GPIO_InitStruct.Pin = Data_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Data_Encoder_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BotonTasaRefrescoDecremento_Pin */
  GPIO_InitStruct.Pin = BotonTasaRefrescoDecremento_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BotonTasaRefrescoDecremento_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SieteSegmentosLEDF_Pin SieteSegmentosLEDB_Pin */
  GPIO_InitStruct.Pin = SieteSegmentosLEDF_Pin|SieteSegmentosLEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SieteSegmentosLEDC_Pin */
  GPIO_InitStruct.Pin = SieteSegmentosLEDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SieteSegmentosLEDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DigitoD0_Pin */
  GPIO_InitStruct.Pin = DigitoD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DigitoD0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Para EXTI0 (PC0)
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  // Para EXTI1 (PC1)
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  // Para EXTI4 (PA4)
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  // Para EXTI15_10 (PB15)
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void setNumeroSieteSegmentos(uint8_t numero){
	switch (numero){
		case 0:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_SET);
			break;
		case 8:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
		case 9:
			HAL_GPIO_WritePin(SieteSegmentosLEDA_GPIO_Port, SieteSegmentosLEDA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDB_GPIO_Port, SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDD_GPIO_Port, SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDE_GPIO_Port, SieteSegmentosLEDE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SieteSegmentosLEDF_GPIO_Port, SieteSegmentosLEDF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SieteSegmentosLEDG_GPIO_Port, SieteSegmentosLEDG_Pin, GPIO_PIN_RESET);
			break;
	}
}

void separarContador(void){
	/*
	 * Esta función asigna a estas variables el número correspondiente para que pueda mostrarse
	 * el número adecuado en el taxímetro
	 */
	miles_contador_Taximetro = contador_Taximetro / 1000;
	centenas_contador_Taximetro = (contador_Taximetro % 1000) / 100;
	decenas_contador_Taximetro = (contador_Taximetro % 100) / 10;
	unidades_contador_Taximetro = contador_Taximetro % 10;
}

uint32_t DWT_Init(void) {
    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Enable clock cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Reset the clock cycle counter */
    DWT->CYCCNT = 0;

    /* Check if DWT is available */
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) {
        return 1; // DWT cycle counter not available on this device.
    }

    return 0; // DWT initialized successfully
}

void performance_7seg(void){

    // --- START MEASUREMENT ---
    start_cycles = DWT->CYCCNT;

    // --- CODE TO BE PROFILED ---
	HAL_GPIO_WritePin(DigitoD3_GPIO_Port, DigitoD3_Pin, GPIO_PIN_SET);   			//Se desactiva el dígito anterior
	setNumeroSieteSegmentos(unidades_contador_Taximetro);							//Se asigna el número correspondiente a las unidades
	HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_RESET);			//Se muestra el número en el dígito 0
    // -------------------------

    // --- END MEASUREMENT ---
    end_cycles = DWT->CYCCNT;

    // Calculate the elapsed time
    elapsed_cycles = end_cycles - start_cycles;

    // --- Report the result via UART ---
    sprintf(tx_buffer, "Tiempo para pintar digito: %lu ciclos\r\n", elapsed_cycles);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

    // You can also calculate the time in microseconds
    // MCU Clock is 16MHz, so 1 cycle = 1/16,000,000 seconds = 0.0625 us
    float time_us = (float)elapsed_cycles * (1.0f / 16.0f);
    sprintf(tx_buffer, "Equivalente a: %.2f us\r\n\n", time_us);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    Current_State = STATE_REFRESH;
}

void performance_message(void){
	char message[] = "Hola mundo! - Examen Taller V\n";

    // --- START MEASUREMENT ---
    start_cycles = DWT->CYCCNT;

    // --- CODE TO BE PROFILED ---
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message),100);
    // -------------------------

    // --- END MEASUREMENT ---
    end_cycles = DWT->CYCCNT;

    // Calculate the elapsed time
    elapsed_cycles = end_cycles - start_cycles;

    // --- Report the result via UART ---
    sprintf(tx_buffer, "Tiempo para Imprimir Mensaje: %lu ciclos\r\n", elapsed_cycles);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

    // You can also calculate the time in microseconds
    // MCU Clock is 16MHz, so 1 cycle = 1/16,000,000 seconds = 0.0625 us
    float time_us = (float)elapsed_cycles * (1.0f / 16.0f);
    sprintf(tx_buffer, "Equivalente a: %.2f us\r\n\n", time_us);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    Current_State = STATE_REFRESH;
}

void displaySieteSegmentos(uint8_t digito){
	switch (digito){
		case 0 :{
			HAL_GPIO_WritePin(DigitoD3_GPIO_Port, DigitoD3_Pin, GPIO_PIN_SET);   			//Se desactiva el dígito anterior
			setNumeroSieteSegmentos(unidades_contador_Taximetro);	//Se asigna el número correspondiente a las unidades
			HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_RESET);			//Se muestra el número en el dígito 0
			break;
		}case 1:{
			HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_SET);				//Se desactiva el digito anterior
			setNumeroSieteSegmentos(decenas_contador_Taximetro);	//Se asigna el número correspondiente a las decenas
			HAL_GPIO_WritePin(DigitoD1_GPIO_Port, DigitoD1_Pin, GPIO_PIN_RESET);			//Se muestra el número en el dígito 1
			break;
		}case 2:{
			HAL_GPIO_WritePin(DigitoD1_GPIO_Port, DigitoD1_Pin, GPIO_PIN_SET);				//Se desactiva el digito anterior
			setNumeroSieteSegmentos(centenas_contador_Taximetro);	//Se asigna el número correspondiente a las centenas
			HAL_GPIO_WritePin(DigitoD2_GPIO_Port, DigitoD2_Pin, GPIO_PIN_RESET);			//Se muestra el número en el dígito 2
			break;
		}case 3:{
			HAL_GPIO_WritePin(DigitoD2_GPIO_Port, DigitoD2_Pin, GPIO_PIN_SET);				//Se desactiva el digito anterior
			setNumeroSieteSegmentos(miles_contador_Taximetro);		//Se asigna el número correspondiente a los miles
			HAL_GPIO_WritePin(DigitoD3_GPIO_Port, DigitoD3_Pin, GPIO_PIN_RESET);			//Se muestra el número en el dígito 3
			break;
		}
	}
}

void actualizar_RGB(void){
	//Esta función aplica el PWM actual del LED RGB
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWM_VERDE); 	//CHANNEL_1 = VERDE
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PWM_ROJO); 	//CHANNEL_2 = ROJO
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,PWM_AZUL); 	//CHANNEL_3 = AZUL
}

void printhelp(void){
	//Esta función imprime el menú de opciones en el terminal
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)help_msg,strlen(help_msg));
	Current_State = STATE_REFRESH; //Se vuelve al estado de REFRESCO
}

void rgb_modify(const char *argumento){

	if (argumento[0] == 'R'){
		PWM_ROJO = atoi(&argumento[1]);
		Current_State = STATE_RGB_FEEDBACK; //Se cambia el estado a STATE_RGB_FEEDBACK
		return;
	}if (argumento[0] == 'G'){
		//Si el argumento es "GREEN", se hace toogle LED VERDE
		PWM_VERDE = atoi(&argumento[1]);
		Current_State = STATE_RGB_FEEDBACK; //Se cambia el estado a STATE_RGB_FEEDBACK
		return;
	}if (argumento[0] == 'B'){
		//Si el argumento es "BLUE", se hace toogle LED AZUL
		PWM_AZUL = atoi(&argumento[1]);
		Current_State = STATE_RGB_FEEDBACK; //Se cambia el estado a STATE_RGB_FEEDBACK
		return;
	}
	unknown();
}

void config_blinky(const char *argumento){
	//Esta función configura el periodo del blinky
	uint32_t nuevo_periodo = atoi(argumento);			//Convierte el argumento a un entero
	if(nuevo_periodo > 0 && nuevo_periodo <= 65536){ 	//Verifica que el periodo esté en el rango permitido
		HAL_TIM_Base_Stop_IT(&htim2); 						//Detiene el timer para poder configurarlo
		htim2.Init.Period = nuevo_periodo - 1; 			//Actualiza el periodo del timer
		HAL_TIM_Base_Init(&htim2); 						//Se carga la configuración del timer con el nuevo periodo
		HAL_TIM_Base_Start_IT(&htim2); 					//Inicia el timer con interrupciones
		const char *msg = "Periodo del blinky configurado correctamente.\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, strlen(msg));
	}else{
		const char *error_msg = "Periodo inválido. Debe estar entre 1 y 65536 ms.\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)error_msg, strlen(error_msg));
	}
	Current_State = STATE_REFRESH; //Se vuelve al estado de REFRESCO
}

ADC_Sampling_Freq_t get_adc_sampling_freq_enum(const char *str) {
	// Se usará esta función para convertir el string a un enum para obtener la frecuencia de muestreo del ADC
    if (strcmp(str, "LOW") == 0) return LOW;
    else if (strcmp(str, "MEDIUM") == 0) return MEDIUM;
    else if (strcmp(str, "HIGH") == 0) return HIGH;
    else if (strcmp(str, "ULTRA") == 0) return ULTRA;
    else return 0; // Valor por defecto si no coincide con ninguno
}

void cfg_adc_sampling_freq(const char *argumento){
	ARR_timer_4 = get_adc_sampling_freq_enum(argumento); // Convierte el argumento a un enum adecuado
	if(ARR_timer_4 == 0){
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)"Frecuencia de muestreo inválida. Debe ser LOW, MEDIUM, HIGH o ULTRA.\n", strlen("Frecuencia de muestreo inválida. Debe ser LOW, MEDIUM, HIGH o ULTRA.\n"));
		Current_State = STATE_REFRESH; // Se vuelve al estado de REFRESCO
		return; // Sale de la función si el argumento es inválido
	}
	HAL_TIM_Base_Stop(&htim4); 				// Detiene el timer para poder configurarlo
	HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_4); 	// Detiene el canal Output Compare
	htim4.Init.Period = ADC_Sampling_Freq[ARR_timer_4-1] - 1; // Actualiza el periodo del timer
	HAL_TIM_Base_Init(&htim4); 					// Se carga la configuración del timer con el nuevo periodo
	HAL_TIM_Base_Start(&htim4); 				// Inicia el timer
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4); 	// Inicia el canal Output Compare
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)"Frecuencia de muestreo configurada correctamente\n",strlen("Frecuencia de muestreo configurada correctamente\n"));
	Current_State = STATE_REFRESH; 				// Se vuelve al estado de REFRESCO
}

void print_adc(void){
	Current_State = STATE_REFRESH; //Se vuelve al estado de REFRESCO Marranada
	char aux_buffer [128];	//Buffer temporal
	for(uint16_t i = 0; i < longitud_ADC ; i++){
		float ADC_Voltaje =(ADC_Buffer[i]*3.3f/4095);
		int aux = sprintf(&aux_buffer[0], "%.3f\r\n", ADC_Voltaje);
		HAL_UART_Transmit(&huart2, (uint8_t *)&aux_buffer, aux,100);
		//Marranada para que funcione mejor
		FSM_update(Current_State);
	}
}

void start_adc(void){
	if (esta_activo_ADC == 0){
		HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4); 	// Inicia el canal Output Compare
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) JOYSTICK_Buffer, 2);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) JOYSTICK_Buffer, 2);
	}
	esta_activo_ADC = 1;						//El ADC está activo y empieza a funcionar
    Current_State = STATE_REFRESH; 				//Se vuelve al estado de REFRESCO
}

void stop_adc(void){
	if(esta_activo_ADC == 1){
		HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_4); 	// Detiene el canal Output Compare
		HAL_ADC_Stop_DMA(&hadc1); 					// Detiene el ADC en modo DMA
	}
	esta_activo_ADC = 0;
	Current_State = STATE_REFRESH; 				//Se vuelve al estado de REFRESCO
}

void convertir_uint16_to_float32(uint16_t* pOrigen, float32_t* pDestino, uint32_t buffer_length){
	const float32_t conversion_factor = 1.0f / (float32_t)ADC_MAX_VALUE;

    for (uint32_t i = 0; i < buffer_length; i++) {
        // Normalizamos a los valores [0.0, 1.0] y convertimos a float cada uno de los valores
    	pDestino[i] = ((float32_t)pOrigen[i]) * conversion_factor;
    }
}

void print_config(void){
	/*
	 * Esta función imprime la configuración del equipo
	 */
	float tasa_muestreo = 44.1;
	if(ARR_timer_4 == MEDIUM){
		tasa_muestreo = 48;
	}else if(ARR_timer_4 == HIGH){
		tasa_muestreo = 96;
	}else if(ARR_timer_4 == ULTRA){
		tasa_muestreo = 128;
	}

	char config[512];
	sprintf(config,
	    "Tasa de Muestreo:      %.1f   kHz\n"
	    "Tamano FFT:            %u   index\n"
	    "Tamano Buffer ADC:     %u   index\n"
	    "TIM2 Prescaler:        %u   \n"
	    "TIM2 Period:           %u   \n"
	    "TIM3 Prescaler:        %u   \n"
	    "TIM3 Period:           %u   \n"
	    "TIM4 Prescaler:        %u   \n"
	    "TIM4 Period:           %u   \n",
	    tasa_muestreo,              // uint16_t
	    fft_Length,                 // uint16_t
	    longitud_ADC,      		// uint16_t
	    (uint16_t )TIM2->PSC,   // uint32_t
	    (uint16_t)TIM2->ARR,   // uint32_t
	    (uint16_t)TIM3->PSC,   // uint32_t
	    (uint16_t)TIM3->ARR,   // uint32_t
	    (uint16_t)TIM4->PSC,   // uint32_t
	    (uint16_t)TIM4->ARR    // uint32_t
	);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)config,strlen(config));
	Current_State = STATE_REFRESH;
}

void perform_fft(void) {
	if (esta_activo_ADC == 0){	//El buffer de ADC no está activo, necesitamos que recolecte datos para hacer la FFT
		const char *errorMsg = "El ADC no está activo, inicia con Start_ADC y vuelve a intentarlo\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)errorMsg, strlen(errorMsg));
		Current_State = STATE_REFRESH;		//Volvemos al Refresh
		return;								//Se sale de la función
	}
	convertir_uint16_to_float32(ADC_Buffer, ADC_Float_Buffer, ADC_BUFFER_MAX_LENGTH);
    // --- Paso 1: Realizar la FFT ---
    // Se asume que el buffer del ADC está listo
    arm_rfft_fast_f32(
        &rfft_instance,       	// Puntero a la instancia que se inicializó
		ADC_Float_Buffer,     	// Puntero a los datos de valor real del ADC
		FFT_Buffer,    			// Puntero al buffer donde se guardan los datos de la FFT
        0                     	// '0' para FFT, '1' para FFT Inversa (IFFT)
    );

    // --- Paso 2: Calcular la magnitud ---
    // The RFFT output is in a packed complex format. This function "unpacks" it
    // and calculates the magnitude of each complex frequency bin.
    // Magnitude = sqrt(real^2 + imag^2)
    arm_cmplx_mag_f32(
    	FFT_Buffer,    			// Puntero con la información de la FFT
		FFT_Magnitudes,       	// PUntero destino a donde se guardan las magnitudes de la FFT
        fft_Length / 2          	// Solo se computan las primeras N/2 magnitudes, los datos relevantes
    );

    // --- Done! ---
    // The 'fft_magnitudes' array now holds the spectrum of your signal.
    // fft_magnitudes[0] = Magnitude of the DC component (0 Hz)
    // fft_magnitudes[1] = Magnitude of the first frequency bin
    // ...
    // fft_magnitudes[511] = Magnitude of the last useful frequency bin
}

void print_fft_features(void){
	/*
	 * Esta función imprime los valores importantes de la FFT
	 */
	perform_fft();	//Realizamos una transformada de Fourier
	float32_t magnitud_predominante;
	uint32_t  magnitud_predominante_index;
	HAL_StatusTypeDef status;
	float tasa_muestreo = 44100;
	if(ARR_timer_4 == MEDIUM){
		tasa_muestreo = 48000;
	}else if(ARR_timer_4 == HIGH){
		tasa_muestreo = 96000;
	}else if(ARR_timer_4 == ULTRA){
		tasa_muestreo = 128000;
	}
	// --- Paso 1: Hallar el componente con el valor más alto ---
	// Se busca desde el índice 1 porque el 0 es la componente DC).
	arm_max_f32(
			&FFT_Magnitudes[1],     		// Iniciamos desde el elemento 1
			(fft_Length / 2) - 1,     		// Número de los elementos que se van a buscar, se asume que son la mitad
			&magnitud_predominante,         // Esta variable contiene la magnitud más grande encontrada
			&magnitud_predominante_index    // Y esta el índice respectivo
	);

	// Ajuste del índice, pues este debe ser absoluto
	magnitud_predominante_index = magnitud_predominante_index + 1;

	// Paso 2: Convertir el índice a un valor de frecuencia real
	float32_t frequency_resolution = tasa_muestreo / (fft_Length);
	float32_t dominant_frequency = magnitud_predominante_index * frequency_resolution;

	// --- Step 3: Formatear aduacadamente la salida ---
	// Formatear la frecuencia en String
	char tx_buffer[80];
	sprintf(tx_buffer, "Frecuencia Dominante:     %.2f Hz\r\n", dominant_frequency);

	// --- Paso 4: Transmitir la frecuencia ya formateada  ---
	status = HAL_UART_Transmit(
			&huart2,                     // Pointer to the UART handle
			(uint8_t*)tx_buffer,         // Pointer to the data buffer (must be cast to uint8_t*)
			strlen(tx_buffer),           // Number of bytes to send
			100
	);

	// Format the magnitude string
	sprintf(tx_buffer, "Magnitud Freq Dominante:  %.2f\r\n", magnitud_predominante);
	sprintf(tx_buffer, "Bin Freq Dominante:       %lu\r\n", magnitud_predominante_index);
	// Transmit the second string
	status = HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer),100);

	// --- Paso 5: Hallemos el componente DC de la señal, para esto, se debe aplicar el factor de
	// Escala adecuado, pues F(x) = 1/N SUM(n=0, n=N-1, x[n]), por lo tanto, este factor
	// Es simplemente el número de muestras 1/N
	float componente_DC = 3.3f * FFT_Magnitudes[0] / fft_Length;
	// Lo formateamos:
	sprintf(tx_buffer, "Componente DC:            %.2f V\r\n", componente_DC);
	//Transmitimos
	status = HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer),100);

	float valor_RMS = 0;
	// Hallamos el valor RMS de la señal
	convertir_uint16_to_float32(ADC_Buffer, ADC_Float_Buffer, ADC_BUFFER_MAX_LENGTH);
	arm_rms_f32(ADC_Float_Buffer, ADC_BUFFER_MAX_LENGTH, &valor_RMS);	//El tercer argumento debe ser un puntero que apunta a una variable tipo float, se obtiene el valor RMS de la señal
	valor_RMS = valor_RMS * 3.3;	//Originilamente el ADC_Float_Buffer está normalizado entre 0 y 1, lo multiplicamos por 3.3V para hallar la potencia adecuadamente
	float potencia_promedio = valor_RMS * valor_RMS;	//La potencia de la señal es su valor RMS al cuadrado
	// Lo formateamos:
	sprintf(tx_buffer, "Potencia Promedio:        %.2f W\r\n", potencia_promedio);
	//Transmitimos
	status = HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer),100);

	if (status != HAL_OK) {
		// Transmission error
	}


	Current_State = STATE_REFRESH;
}

uint8_t uint16_es_potencia_de_2(uint16_t numero){
	//Si es potencia de 2, entonces solo debe haber un bit encendido en el número
	if((numero & (numero-1)) == 0){
		//Se trata de una potencia de 2
		return 0;
	}
	return 1;
}

void cfg_size_fft(const char *argumento){
	/*
	 * Esta función configura el tamaño de la FFT
	 */
	uint16_t aux_verificacion = 0;
	uint16_t fft_new_Length = atoi(argumento);
	aux_verificacion = uint16_es_potencia_de_2(fft_new_Length);
	if(aux_verificacion || (fft_new_Length < 16) || (fft_new_Length > ADC_BUFFER_MAX_LENGTH)){
		//No pasa la verificación, no se modifica nada
		const char *errorMsg = "Valor no adecuado para la FFT, deben ser potencias de 2 desde 16 hasta 2048\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) errorMsg, strlen(errorMsg));
		Current_State = STATE_REFRESH;
		return;
	}
	//En caso contrario, procedemos a modificar el valor de la FFT
	fft_Length = fft_new_Length;	//Asignamos el nuevor valor a la FFT
	longitud_ADC = fft_Length;
	arm_rfft_fast_init_f32(&rfft_instance, fft_Length);
	const char *doneMsg = "Tamaño de FFT actualizado correctamente\n";
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *) doneMsg, strlen(doneMsg));
	Current_State = STATE_REFRESH;
}

void print_fft(void){
	/*
	 * Esta función imprime el espectro de la FFT
	 */
	//Primero transformamos los datos a float del buffer ADC
	perform_fft();

	char aux_buffer [8192];	//Buffer temporal
	uint16_t offset = 0;	//Variable para saber en qué parte del buffer poner los datos

	for(uint16_t i = 0; i < (fft_Length / 2) ; i++){
		int aux = sprintf(&aux_buffer[offset], "%.3f\r\n", FFT_Magnitudes[i]);
		offset+=aux;	//Aumentamos lo que escribimos
		//Marranada para que funcione mejor
		Current_State = STATE_REFRESH;
		FSM_update(Current_State);
	}
	FSM_update(Current_State);	//Otra marranada
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)aux_buffer, offset );	//A enviar la data de la FFT
}

void unknown(void){
	const char *error_msg = "Comando no reconocido. Escriba 'help' para ver las opciones disponibles.\n";
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)error_msg, strlen(error_msg));
	Current_State = STATE_REFRESH;
}

ID_Comando encontrar_id_comando(const char* str_comando){
    for (int i = 0; i < NUM_COMANDOS; i++) {
        if (strcmp(str_comando, comandos[i].command_str) == 0) {
            return comandos[i].id_comando;
        }
    }
    return CMD_ID_UNKNOWN;
}

void despachar_comando(char *line){
	/*
	 * Esta función despacha el comando recibido por el USART
	 * Se van a separar los comandos y argumentos
	 * de la línea recibida por el USART
	 */

	//Empezamos por eliminar los caracteres de nueva línea y retorno de carro \r
	for (int i = 0; i < rx_size; i++) {
	    if (line[i] == '\r' || line[i] == '\n') {
	    	line[i] = '\0';
	        break;
	    }
	}

	char *argumento = strchr(line, ' '); //Se separa el parámetro del resto de la línea
	 if (argumento) {
	        *argumento = '\0';  // separamos con null terminator
	        argumento++;        // ahora argumento apunta al primer argumento
	    } else {
	        argumento = "";     // sin argumentos
	    }

	 ID_Comando id = encontrar_id_comando(line);
	 switch (id){
	 	 case CMD_ID_printhelp: 			printhelp(); 						break;
	 	 case CMD_ID_rgb_modify:			rgb_modify(argumento); 				break;
	 	 case CMD_ID_config_blinky:			config_blinky(argumento);			break;
	 	 case CMD_ID_cfg_adc_sampling_freq: cfg_adc_sampling_freq(argumento);	break;
	 	 case CMD_ID_cfg_size_fft:			cfg_size_fft(argumento);			break;
	 	 case CMD_ID_print_adc:				print_adc();						break;
	 	 case CMD_ID_print_fft:				print_fft();						break;
	 	 case CMD_ID_print_config:			print_config();						break;
	 	 case CMD_ID_print_fft_features:	print_fft_features();				break;
	 	 case CMD_ID_start_adc:				start_adc();						break;
	 	 case CMD_ID_stop_adc:				stop_adc();							break;
	 	 case CMD_ID_performance_7_segments:performance_7seg();					break;
	 	 case CMD_ID_performance_message:	performance_message();				break;
	 	 case CMD_ID_UNKNOWN:				unknown();							break;
	 }
}

void FSM_update(State_t State){
	switch (State){
		case STATE_REFRESH:{
			if(digito > 3){	//Si se quiere prender el digito 4 (No existe) se devuelve al digito 0
				digito = 0;
			}
			displaySieteSegmentos(digito);	//Se muestran los digitos con el efecto visual deseado
			break;
		}case STATE_RGB_FEEDBACK :{
			//Este condicional es para poder repetir el ciclo dentro del ENUM indefinidamente
			if((PWM_ROJO > 99) & (PWM_ROJO < 101)){
				PWM_ROJO = 99;
			}else if(PWM_ROJO >100){
				PWM_ROJO =0;
			}if((PWM_AZUL > 99) & (PWM_AZUL < 101)){
				PWM_AZUL = 99;
			}else if(PWM_AZUL > 100){
				PWM_AZUL = 0;
			}if((PWM_VERDE > 99) & (PWM_VERDE < 101)){
				PWM_VERDE = 99;
			}else if(PWM_VERDE > 100){
				PWM_VERDE = 0;
			}

			actualizar_RGB(); //Se actualiza el color del LED RGB
			Current_State = STATE_REFRESH;
			break;
		}case STATE_TAXIMETER_FEEDBACK :{
			uint16_t x = (uint16_t)(JOYSTICK_X * 99.0f / 4095.0f);
			uint16_t y = (uint16_t)(JOYSTICK_Y * 99.0f / 4095.0f);

			contador_Taximetro = (y*100)+(x);
			separarContador();	//Se separan los miles, centenas, decenas, unidades del nuevo número en el contador para
			Current_State = STATE_REFRESH;//Se cambia al estado IDLE para que se muestre los números del contador
			break;
		}case STATE_CHANGE_REFRESH :{
			HAL_TIM_Base_Stop_IT(&htim3); 					//Se apaga el Timer 3 para configurarlo
			if((htim3.Init.Period >= 4) & (htim3.Init.Period <= 1000)){  					//Se verifica si el valor que entra es válido
				//En el caso de que sí, entonces:
				HAL_TIM_Base_Init(&htim3);					//La variable ya fue modificada y se carga a la configuración del timer
				HAL_TIM_Base_Start_IT(&htim3);
				Current_State = STATE_REFRESH;				//Volvemos al estado de Refresh ahora con una velocidad más rápida
			}else{											//El decremento es inválido
				htim3.Init.Period = 4;						// El periodo del timer vuelve a su valor de 5ms
				HAL_TIM_Base_Start_IT(&htim3);				//Se vuelve a encender el Timer
			    Current_State = STATE_REFRESH;				//Se vuelve al estado de REFRESCO
			}
			break;
		}case STATE_TERMINAL_FEEDBACK :{
			  if (paquete_listo.size > 0){
				  // --- DATA IS READY FOR PROCESSING ---
				  // The DMA is now safely filling the *other* buffer.
				  // We can take as long as we need to process the data in
				  // data_ready_packet.buffer without risk of corruption.

				  // First, get a local copy of the pointer and size
				  rx_buffer = paquete_listo.buffer;
				  rx_size = paquete_listo.size;

				  // --- IMPORTANT: Clear the volatile struct to "consume" the data ---
				  // This signals we are done and can accept the next packet.
				  paquete_listo.buffer = NULL;
				  paquete_listo.size = 0;
			  }
			despachar_comando((char *) rx_buffer); //Se despacha el comando recibido por el terminal
			break;
		}case STATE_ENCODER_FEEDBACK :{
			//Primero se identifica el sentido de giro del encoder,
			//Sumando o restando en cada caso
			if (HAL_GPIO_ReadPin(Data_Encoder_GPIO_Port, Data_Encoder_Pin) == GPIO_PIN_RESET){	//Se lee el pin Data del Encoder
				PWM_ROJO++;
				PWM_VERDE++;
				PWM_AZUL++;
			}else{
				PWM_ROJO--;
				PWM_VERDE--;
				PWM_AZUL--;
			}
			Current_State = STATE_RGB_FEEDBACK;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(UserLed_GPIO_Port, UserLed_Pin); // UserLed parpadea cada 250ms
	}else if(htim->Instance == TIM3){
		digito++; // Se incrementa el dígito cada 5ms
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Switch_Encoder_Pin){ // Botón del encoder
		if((PWM_ROJO == 0) && (PWM_VERDE == 0) && (PWM_AZUL == 0)){
			PWM_ROJO = 99;
			PWM_VERDE = 99;
			PWM_AZUL = 99;
		}else{
			PWM_ROJO = 0;
			PWM_VERDE = 0;
			PWM_AZUL = 0;
		}
		Current_State = STATE_RGB_FEEDBACK;				//Se cambia al estado RGB
	}else if(GPIO_Pin == Clk_Encoder_Pin){
		Current_State = STATE_ENCODER_FEEDBACK; //Se identifica un flanco de subida en el clock y se pasa rápidamente al estado que cambia el número del taxímetro
	}else if(GPIO_Pin == BotonTasaRefrescoDecremento_Pin){
		htim3.Init.Period	+= 10; 	//Modificamos el valor del periodo del timer, aumentando la velocidad
		Current_State = STATE_CHANGE_REFRESH;
	}else if(GPIO_Pin == BotonTasaRefrescoIncremento_Pin){
		htim3.Init.Period	-= 10; 	//Modificamos el valor del periodo del timer, disminuyendo la velocidad
		Current_State = STATE_CHANGE_REFRESH;

	}
}

//Callback del USART Idle para no tener que saber el tamaño del mensaje
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2) {
		if (buffer_activo_DMA == BUFFER_A){
			// Señal de que el Buffer A está listo
			paquete_listo.buffer  = rx_Buffer_A;
			paquete_listo.size = Size;
			// Inmediatamente cambiamos el DMA al buffer B
			buffer_activo_DMA = BUFFER_B;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_B, rx_buffer_length); // Configura la recepción DMA con IDLE
		}else {
			// Señal de que el Buffer B está listo
			paquete_listo.buffer = rx_Buffer_B;
			paquete_listo.size = Size;
			// Inmediatamente cambiamos el DMA al buffer
			buffer_activo_DMA = BUFFER_A;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_A, rx_buffer_length); // Configura la recepción DMA con IDLE
		}
		Current_State = STATE_TERMINAL_FEEDBACK;
		}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        JOYSTICK_X = JOYSTICK_Buffer[0];
        JOYSTICK_Y = JOYSTICK_Buffer[1];
        Current_State = STATE_TAXIMETER_FEEDBACK;
    }
}

/* USER CODE END 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
