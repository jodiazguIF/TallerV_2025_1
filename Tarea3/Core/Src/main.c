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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
State_t Current_State = STATE_REFRESH; // Se fija el estado por defecto

RGB_Color_t Current_Color = APAGADO;	//Se fija el estado inicial del LED RGB

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void FSM_update(State_t State);
void configMagic(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // Inicia el Timer 2 para interrupciones
  HAL_TIM_Base_Start_IT(&htim3); // Inicia el Timer 3 para interrupciones
  HAL_GPIO_WritePin(DigitoD0_GPIO_Port, DigitoD0_Pin, GPIO_PIN_SET); // Apagamos todos los digitos para evitar el fantasma
  HAL_GPIO_WritePin(DigitoD1_GPIO_Port, DigitoD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DigitoD2_GPIO_Port, DigitoD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DigitoD3_GPIO_Port, DigitoD3_Pin, GPIO_PIN_SET);
  configMagic();
  printf("Configuración Terminada :D \n");
  printf("+-- Intro EXTI --+");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
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
  HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin|RGB_AZUL_Pin|SieteSegmentosLEDF_Pin|SieteSegmentosLEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DigitoD3_Pin|DigitoD2_Pin|DigitoD1_Pin|SieteSegmentosLEDA_Pin
                          |SieteSegmentosLEDE_Pin|SieteSegmentosLEDG_Pin|SieteSegmentosLEDD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SieteSegmentosLEDC_GPIO_Port, SieteSegmentosLEDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DigitoD0_Pin|RGB_ROJO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLed_Pin */
  GPIO_InitStruct.Pin = UserLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch_Encoder_Pin Clk_Encoder_Pin */
  GPIO_InitStruct.Pin = Switch_Encoder_Pin|Clk_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BotonTasaRefrescoIncremento_Pin */
  GPIO_InitStruct.Pin = BotonTasaRefrescoIncremento_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BotonTasaRefrescoIncremento_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_VERDE_Pin SieteSegmentosLEDF_Pin SieteSegmentosLEDB_Pin */
  GPIO_InitStruct.Pin = RGB_VERDE_Pin|SieteSegmentosLEDF_Pin|SieteSegmentosLEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_AZUL_Pin */
  GPIO_InitStruct.Pin = RGB_AZUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RGB_AZUL_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : SieteSegmentosLEDC_Pin */
  GPIO_InitStruct.Pin = SieteSegmentosLEDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SieteSegmentosLEDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DigitoD0_Pin RGB_ROJO_Pin */
  GPIO_InitStruct.Pin = DigitoD0_Pin|RGB_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

void cambioEstadoLEDRGB(RGB_Color_t Color){
	switch (Color){
		case APAGADO: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_RESET);
			break;
		}
		case ROJO: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_RESET);
			break;
		}
		case AZUL: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_RESET);
			break;
		}
		case VERDE: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_SET);
			break;
		}
		case AZUL_VERDE: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_SET);
			break;
		}
		case AZUL_ROJO: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_RESET);
			break;
		}
		case ROJO_VERDE: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_SET);
			break;
		}
		case ROJO_VERDE_AZUL: {
			HAL_GPIO_WritePin(GPIOA, RGB_AZUL_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RGB_ROJO_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, RGB_VERDE_Pin, GPIO_PIN_SET);
			break;
		}
	}
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
			if (Current_Color == ROJO_VERDE_AZUL ){	//Si se hace una interrupción y el estado es el final
				Current_Color = APAGADO;			//Se vuelve al estado apagado
				cambioEstadoLEDRGB(Current_Color);	//Se muestra el estado apagado
				Current_State = STATE_REFRESH;			//Se vuelve al estado IDLE
			}else{
				Current_Color++;					//De otra forma, se cambia al siguiente estado
				cambioEstadoLEDRGB(Current_Color);	//Se muestra el color del estado
				Current_State = STATE_REFRESH;			//Se vuelve al estado IDLE
			}
			break;
		}case STATE_TAXIMETER_FEEDBACK :{
			//Primero se identifica el sentido de giro del encoder,
			//Sumando o restando en cada caso
			if (HAL_GPIO_ReadPin(Data_Encoder_GPIO_Port, Data_Encoder_Pin) == GPIO_PIN_RESET){	//Se lee el pin Data del Encoder
				contador_Taximetro++;
			}else{
				contador_Taximetro--;
			}
			//Luego se evita que el contador pase de 0 a 0xFFFF, pasándolo a 0xFFF, pues queremos simular una variable de 12 bits
			if(contador_Taximetro > 4096){
				contador_Taximetro = 4095;
			}else if(contador_Taximetro > 4095){ //También se debe desbordar de 0xFFF a 0 en lugar de seguir subiendo
				contador_Taximetro = 0;
			}
			separarContador();	//Se separan los miles, centenas, decenas, unidades del nuevo número en el contador para
								//Mostrarlos posteriormente
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
		printf("Tiempo en ms: %ld", contador_Tiempo);
		Current_State = STATE_RGB_FEEDBACK;	//Al identificar un flanco de subida en el switch se cambia al estado RGB
	}else if(GPIO_Pin == Clk_Encoder_Pin){
		Current_State = STATE_TAXIMETER_FEEDBACK; //Se identifica un flanco de subida en el clock y se pasa rápidamente al estado que cambia el número del taxímetro
	}else if(GPIO_Pin == BotonTasaRefrescoDecremento_Pin){
		htim3.Init.Period	+= 10; 	//Modificamos el valor del periodo del timer, aumentando la velocidad
		Current_State = STATE_CHANGE_REFRESH;
	}else if(GPIO_Pin == BotonTasaRefrescoIncremento_Pin){
		htim3.Init.Period	-= 10; 	//Modificamos el valor del periodo del timer, disminuyendo la velocidad
		Current_State = STATE_CHANGE_REFRESH;

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
