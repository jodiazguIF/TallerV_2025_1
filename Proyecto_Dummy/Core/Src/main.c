/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Se viene proyecto Siuuu
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
State_t Current_State = STATE_IDLE; 	// Se fija el estado por defecto

//Variables auxiliares para la comunciación USART
uint16_t rx_buffer_length = 64;
uint8_t rx_Buffer_A[RX_BUFFER_MAX_LENGTH]; // Buffer de recepción A
uint8_t rx_Buffer_B[RX_BUFFER_MAX_LENGTH]; // Buffer de recepción B
char 	tx_buffer[100];
volatile Paquete_Datos paquete_listo = {.buffer = NULL, .size = 0};
volatile BufferActivo buffer_activo_DMA = BUFFER_A;
uint8_t*  rx_buffer;
uint16_t  rx_size;
char *argumento_comandos;

//Variables para el acelerómetro
float ACCEL_X, ACCEL_Y, ACCEL_Z; // Variables para almacenar los valores del acelerómetro
float GYRO_X, GYRO_Y, GYRO_Z; // Variables para almacenar los valores del giroscopio
int16_t ACCEL_X_RAW, ACCEL_Y_RAW, ACCEL_Z_RAW; // Variables para almacenar los valores del acelerómetro _RAW
int16_t GYRO_X_RAW, GYRO_Y_RAW, GYRO_Z_RAW; // Variables para almacenar los valores del giroscopio _RAW


//Variable para el tracking del estado del balance
uint8_t balance = 0; // 0 = Balance desactivado, 1 = Balance activado

//Variables para el PID
float dt = 1.0f / 100.0f;  // Tiempo entre lecturas, el MPU6050 tiene una frecuencia de 700Hz
float alpha = 0.98;        // Puedes ajustar entre 0.95 y 0.99 si quieres mayor o menor peso del giroscopio
float angulo = 0;

float target_angle = 0.0f;      // Ángulo objetivo (vertical = 0°)
float error = 0.0f;
float prev_error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;

float kp = 3.73f;           // Ganancia proporcional
float ki = 0.12f;            // Ganancia integral
float kd = 0.31f;            // Ganancia derivativa

float pid_output = 0.0f;    // Salida del PID

//Le asignamos a una variable el mensaje de help, tipo static porque pues, no lo vamos a modificar nunca
static const char help_msg[] =
    "Opciones:\r\n"
    "1. Ayuda:              help\r\n"
	"2. Balance:            balance\r\n"
;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void printhelp(void);
void FSM_update(State_t State);
void I2C_verify(void);
void MPU6050_Init(void);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2); // Inicia el timer 2 con interrupciones
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_A, rx_buffer_length); // Configura la recepción DMA con IDLE

  //Todos los motores empiezan con el freno activado
  HAL_GPIO_WritePin(Motor_1_BRAKE_GPIO_Port, Motor_1_BRAKE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Motor_2_BRAKE_GPIO_Port, Motor_2_BRAKE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Motor_3_BRAKE_GPIO_Port, Motor_3_BRAKE_Pin, GPIO_PIN_RESET);

  MPU6050_Init(); //Inicializamos el MPU6050
  printhelp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  FSM_update(Current_State); // Llama a la máquina de estados con el estado actual
    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Period = 100;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim2.Init.Period = 250;
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

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLed_GPIO_Port, UserLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Balance_OK_Pin|I2C_Error_Pin|Motor_1_BRAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_3_DIR_Pin|Motor_1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C_OK_Pin|Motor_2_BRAKE_Pin|Motor_2_DIR_Pin|Motor_3_BRAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserLed_Pin */
  GPIO_InitStruct.Pin = UserLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Balance_OK_Pin I2C_Error_Pin Motor_1_BRAKE_Pin */
  GPIO_InitStruct.Pin = Balance_OK_Pin|I2C_Error_Pin|Motor_1_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_3_DIR_Pin Motor_1_DIR_Pin */
  GPIO_InitStruct.Pin = Motor_3_DIR_Pin|Motor_1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_Data_Ready_Pin */
  GPIO_InitStruct.Pin = MCU_Data_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCU_Data_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_OK_Pin Motor_2_BRAKE_Pin Motor_2_DIR_Pin Motor_3_BRAKE_Pin */
  GPIO_InitStruct.Pin = I2C_OK_Pin|Motor_2_BRAKE_Pin|Motor_2_DIR_Pin|Motor_3_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void printhelp(void){
	//Esta función imprime el menú de opciones en el terminal
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)help_msg,strlen(help_msg));
	Current_State = STATE_IDLE; //Se vuelve al estado de REFRESCO
}

void  I2C_verify(void){
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MCU_WHO_AM_I_REG, 1, &check, 1, 1000);
	if (check == 0x68) {
	    //La pantalla y el MPU están respondiendo
		HAL_GPIO_WritePin(I2C_OK_GPIO_Port, I2C_OK_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(I2C_OK_GPIO_Port, I2C_OK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(I2C_Error_GPIO_Port, I2C_Error_Pin, GPIO_PIN_SET);

	}
}

void MPU6050_Init(void){
	uint8_t check, data; //Variables auxiliares para la comunicación I2C
	//Primero verificamos que el MPU6050 esté conectado, leyendo el registro WHO_AM_I (0x75), el sensor debería retornar 0x68, su ADDRESS con AD0 = 0
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MCU_WHO_AM_I_REG, 1, &check, 1, 1000);

	if(check != 0x68){
		//El sensor no está conectado correctamente
		const char *msg = "MPU6050 desconectado. Conéctelo correctamente.\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, strlen(msg));
		HAL_GPIO_WritePin(I2C_Error_GPIO_Port, I2C_Error_Pin, GPIO_PIN_SET);
		return; //No se puede continuar
	}
	HAL_GPIO_WritePin(I2C_OK_GPIO_Port, I2C_OK_Pin, GPIO_PIN_SET);
	//Si está conectado, procedemos a configurarlo, escribiendno en el registro PWR_MGMT_1 (0x6B) un 0x00, para que el sensor esté activo
	/*	Bit 7: Device_Reset 	= 0, No queremos reiniciar el dispositivo
	 *  Bit 6: Sleep 			= 0, No queremos que el dispositivo esté en modo de sueño
	 *  Bit 5: Cycle 			= 0, No queremos que el dispositivo esté en modo de ciclo
	 *  Bit 4: Reserved 		= 0, Reservado
	 *  Bit 3: Temp_Disable 	= 1, No necesitamos que el sensor mida la temperatura
	 *  Bit 2-0: Clock_Select 	= 000, Seleccionamos el reloj interno del sensor
	 *  Escribimos entonces  0b00001000 = 0x08
	 */
	data = 0x08; //Valor a escribir en el registro PWR_MGMT_1
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
	/*
	 * Ahora debemos configurar el sample rate del sensor, fijándonos en el registro SMPLRT_DIV 0x19 y en el Configuration 0x1A
	 * Por defecto, configuration = 0x00, lo que significa que el sensor tiene un gyroscope output rate de 8kHz
	 * Luego, en en SMPLRT_Div, escribimos un valor de 0x07, para configurar el sample rate a 1kHz
	 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	 */
	data = 0x50; //Valor para escribir en SMPLRT_DIV
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
	/*
	 * Procedemos a configurar la escala del giroscopio y acelerómetro a un rango de 2000°/s 0x18
	 * Y un rango en la aceleración de +- 2g: 0x18
	 * Omitimos los self tests, pues no los vamos a usar
	 */
	data = 0x00; //Valor para escribir en el registro GYRO_CONFIG (0x1B) y ACCEL_CONFIG_REG (0x1C)
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

	data = 0x01; //Valor a escribir en el INT_ENABLE para activar una interrupción en el MPU cada que hayan datos listos para leer
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MCU_INT_ENABLE, 1, &data, 1, 1000);

}

void MPU6050_Read_Accel(void){
	/*
	 * La configuración que tenemos es de +- 16g
	 * Por lo tanto, LSB sensitivity es de 2048 LSB/g
	 */
	uint8_t data_Recolectada[6];

	// Leemos los 6 bytes de data empezando desde el registro ACCEL_XOUT_H (0x3B)

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data_Recolectada, 6, 1000);

	ACCEL_X_RAW = (int16_t)(data_Recolectada[0] << 8 | data_Recolectada[1]); // Combinamos los dos bytes de datos para el eje X
	ACCEL_Y_RAW = (int16_t)(data_Recolectada[2] << 8 | data_Recolectada[3]); // Combinamos los dos bytes de datos para el eje Y
	ACCEL_Z_RAW = (int16_t)(data_Recolectada[4] << 8 | data_Recolectada[5]); // Combinamos los dos bytes de datos para el eje Z

	ACCEL_X = (float )ACCEL_X_RAW / 16384; // Convertimos a g
	ACCEL_Y = (float )ACCEL_Y_RAW / 16384; // Convertimos a g
	ACCEL_Z = (float )ACCEL_Z_RAW / 16384; // Convertimos a g
}

void MPU6050_Read_Gyro(void){
	/*
	 * La configuración que tenemos es de 2000°/s
	 * Por lo tanto, LSB sensitivity es de 16.4 LSB/s
	 */
	uint8_t data_Recolectada[6];

	// Leemos los 6 bytes de data empezando desde el registro GYRO_XOUT_H (0x43)

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data_Recolectada, 6, 1000);

	GYRO_X_RAW = (int16_t)(data_Recolectada[0] << 8 | data_Recolectada[1]); // Combinamos los dos bytes de datos para el eje X
	GYRO_Y_RAW = (int16_t)(data_Recolectada[2] << 8 | data_Recolectada[3]); // Combinamos los dos bytes de datos para el eje Y
	GYRO_Z_RAW = (int16_t)(data_Recolectada[4] << 8 | data_Recolectada[5]); // Combinamos los dos bytes de datos para el eje Z

	GYRO_X = (float )GYRO_X_RAW / 131; // Convertimos a °/s
	GYRO_Y = (float )GYRO_Y_RAW / 131; // Convertimos a °/s
	GYRO_Z = (float )GYRO_Z_RAW / 131; // Convertimos a °/s
}

void update_angle_with_complementary_filter() {

    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();

    // Calcula ángulo a partir del acelerómetro (ejemplo: eje Y vs Z, inclinación sobre eje X)
    float accel_angle = atan2f(ACCEL_Y, ACCEL_Z) * 180.0f / M_PI;

    // Integración del giroscopio (ejemplo: GYRO_X controla rotación sobre X)
    float gyro_rate = GYRO_X;  // en grados por segundo

    angulo = alpha * (angulo + gyro_rate * dt) + (1 - alpha) * accel_angle;
}

void actualizar_PID(float angle, float dt) {
    error = target_angle - angle;

    integral += error * dt;
    derivative = (error - prev_error) / dt;

    pid_output = kp * error + ki * integral + kd * derivative;

    prev_error = error;

    // Límite de salida (anti-windup y saturación PWM)
    if (pid_output > 100) pid_output = 100;
    else if (pid_output < -100) pid_output = -100;
}

void set_motor_pwm(MotorID_t motor, uint8_t duty) {
	if (duty > 100) duty = 100;

	uint32_t pwm_value = (__HAL_TIM_GET_AUTORELOAD(&htim1) * duty) / 100;

	switch (motor) {
		case MOTOR_1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value); // Motor 1 -> PA8
			break;
		case MOTOR_2:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_value); // Motor 2 -> PA10
			break;
		case MOTOR_3:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_value); // Motor 3 -> PA11
			break;
	}
}

void aplicar_salida_motor(MotorID_t motor, float pid_output) {
	GPIO_TypeDef* DIR_Port = NULL;
	uint16_t DIR_Pin = 0;
	GPIO_TypeDef* BRAKE_Port = NULL;
	uint16_t BRAKE_Pin = 0;

	if (pid_output > 100.0f) pid_output = 100.0f;
	if (pid_output < -100.0f) pid_output = -100.0f;

	// Seleccionar pines de acuerdo al motor
	switch (motor) {
		case MOTOR_1:
			DIR_Port = Motor_1_DIR_GPIO_Port;
			DIR_Pin  = Motor_1_DIR_Pin;
			BRAKE_Port = Motor_1_BRAKE_GPIO_Port;
			BRAKE_Pin  = Motor_1_BRAKE_Pin;
			break;
		case MOTOR_2:
			DIR_Port = Motor_2_DIR_GPIO_Port;
			DIR_Pin  = Motor_2_DIR_Pin;
			BRAKE_Port = Motor_2_BRAKE_GPIO_Port;
			BRAKE_Pin  = Motor_2_BRAKE_Pin;
			break;
		case MOTOR_3:
			DIR_Port = Motor_3_DIR_GPIO_Port;
			DIR_Pin  = Motor_3_DIR_Pin;
			BRAKE_Port = Motor_3_BRAKE_GPIO_Port;
			BRAKE_Pin  = Motor_3_BRAKE_Pin;
			break;
		default:
			return; // Motor inválido
	}

	if (pid_output > 0) {
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_SET);   // Sentido A
		HAL_GPIO_WritePin(BRAKE_Port, BRAKE_Pin, GPIO_PIN_SET);
		set_motor_pwm(motor, (uint8_t)pid_output);
	} else if (pid_output < 0) {
		HAL_GPIO_WritePin(DIR_Port, DIR_Pin, GPIO_PIN_RESET); // Sentido B
		HAL_GPIO_WritePin(BRAKE_Port, BRAKE_Pin, GPIO_PIN_SET);
		set_motor_pwm(motor, (uint8_t)(-pid_output));
	} else {
		// Freno activo
		HAL_GPIO_WritePin(BRAKE_Port, BRAKE_Pin, GPIO_PIN_RESET);
		set_motor_pwm(motor, 0);
	}
}

void set_balance_angle(void) {
    // Asegúrate de haber leído sensores antes
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    update_angle_with_complementary_filter();

    target_angle = angulo;  // Establece el ángulo actual como el nuevo punto de equilibrio
    Current_State = STATE_BALANCE; // Cambia al estado de balanceo
    balance = 1; // Activa el modo de balanceo
    HAL_GPIO_WritePin(Balance_OK_GPIO_Port, Balance_OK_Pin, GPIO_PIN_SET); // Enciende un LED para indicar que el balance está activo
}

void unknown(void){
	const char *error_msg = "Comando no reconocido. Escriba 'help' para ver las opciones disponibles.\n";
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)error_msg, strlen(error_msg));
	Current_State = STATE_IDLE;
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
	 	 case CMD_ID_Balance:				set_balance_angle();				break;
	 	 case CMD_ID_UNKNOWN:				unknown();							break;
	 }
}

void FSM_update(State_t State){
	switch (State){
		case STATE_IDLE:{
			I2C_verify();	 //Verificamos el estado del I2C
			break;
		}case STATE_READ_MCU_DATA:{
			MPU6050_Read_Accel(); //Leemos el acelerómetro
			MPU6050_Read_Gyro(); //Leemos el giroscopio
			I2C_verify();	 //Verificamos el estado del I2C
			Current_State = STATE_IDLE; //Si el balance no está activo, pasamos al estado IDLE
			break; //No hacemos nada más
		}case STATE_BALANCE:{
			I2C_verify();	 //Verificamos el estado del I2C
		    // 1. Leer sensores (acelerómetro y giroscopio)
		    MPU6050_Read_Accel();
		    MPU6050_Read_Gyro();

		    // 2. Estimar ángulo con filtro complementario
		    update_angle_with_complementary_filter();

		    // 3. Calcular salida PID
		    actualizar_PID(angulo, dt);

		    // 4. Aplicar a motores
		    aplicar_salida_motor(MOTOR_1, pid_output);
		    aplicar_salida_motor(MOTOR_2, pid_output);
		    aplicar_salida_motor(MOTOR_3, pid_output);

		    break;
		}case STATE_TERMINAL:{
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
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(UserLed_GPIO_Port, UserLed_Pin); // UserLed parpadea cada 250ms
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == MCU_Data_Ready_Pin){
		if (balance){
			Current_State = STATE_BALANCE; // Si el balance está activo, pasamos al estado de balanceo
		}else{
			Current_State = STATE_READ_MCU_DATA;
		}
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
		Current_State = STATE_TERMINAL;
		FSM_update(Current_State); // Llama a la máquina de estados con el estado actual
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
