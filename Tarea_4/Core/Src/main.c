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
#include "i2c-lcd.h"
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

uint16_t Buffer_Datos 			[ADC_BUFFER_MAX_LENGTH]; // Array para almacenar los valores del MCU
float 	Buffer_Datos_Float 		[ADC_BUFFER_MAX_LENGTH]; // Array para almacenar los valores del ADC hechos float
float32_t FFT_Buffer			[ADC_BUFFER_MAX_LENGTH]; 	//Buffer para el output de la FFT
float32_t FFT_Magnitudes 		[ADC_BUFFER_MAX_LENGTH/2];	//Buffer para las magnitudes de la FFT
arm_rfft_fast_instance_f32 rfft_instance; 						//Se crea una instancia que es requerida por la función RFFT
uint16_t fft_Length = ADC_BUFFER_MAX_LENGTH;					//Tamaño de la FFT que se va a realizar, 2048 por defecto
uint16_t longitud_Buffer_Datos = 2048;
char *argumento_comandos;


//Le asignamos a una variable el mensaje de help, tipo static porque pues, no lo vamos a modificar nunca
static const char help_msg[] =
    "Opciones:\r\n"
    "1.  Configurar periodo del Blinky:              Config_Blinky_Period <Period (ms)>\r\n"
    "2.  Configurar tamaño de la FFT:                Config_FFT_Size\r\n"
    "3.  Imprimir espectro FFT:                      Print_FFT\r\n"
    "4.  Imprimir configuración del equipo:          Print_Config\r\n"
    "5. Imprimir valores clave de la FFT:           Print_FFT_Features\r\n";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void printhelp(void);
void FSM_update(State_t State);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); 	// Inicia el Timer 2 para interrupciones

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_Buffer_A, rx_buffer_length); // Configura la recepción DMA con IDLE

  printhelp();	//Imprimimos la ayuda


  arm_rfft_fast_init_f32(&rfft_instance, fft_Length);	//Iniciamos la instancia al incio para no llamarla cada printFFT, Lo haremos con 2048 por defecto
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLed_GPIO_Port, UserLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserLed_Pin */
  GPIO_InitStruct.Pin = UserLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLed_GPIO_Port, &GPIO_InitStruct);

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

void MPU6050_Init(void){
	uint8_t check, data; //Variables auxiliares para la comunicación I2C
	//Primero verificamos que el MPU6050 esté conectado, leyendo el registro WHO_AM_I (0x75), el sensor debería retornar 0x68, su ADDRESS con AD0 = 0
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	if(check != 0x68){
		//El sensor no está conectado correctamente
		const char *msg = "MPU6050 desconectado. Conéctelo correctamente y escriba: Start_DMU.\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, strlen(msg));
		return; //No se puede continuar
	}

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
	data = 0x07; //Valor para escribir en SMPLRT_DIV
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
	/*
	 * Procedemos a configurar la escala del giroscopio y acelerómetro a un rango de 250°/s 0x00
	 * Y un rango en la aceleración de +- 16g: 0x18
	 * Omitimos los self tests, pues no los vamos a usar
	 */
	data = 0x00; //Valor para escribir en el registro GYRO_CONFIG (0x1B)
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	data = 0x18; //Valor para escribir en el registro ACCEL_CONFIG (0x1C)
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
}

void printhelp(void){
	//Esta función imprime el menú de opciones en el terminal
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)help_msg,strlen(help_msg));
	Current_State = STATE_IDLE; //Se vuelve al estado de REFRESCO
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
	Current_State = STATE_IDLE; //Se vuelve al estado de REFRESCO
}

void print_config(void){
	/*
	 * Esta función imprime la configuración del equipo
	 */

	char config[512];
	sprintf(config,
	    "Tamano FFT:            %u   index\n"
	    "TIM2 Prescaler:        %u   \n"
	    "TIM2 Period:           %u   \n",
		(uint16_t) longitud_Buffer_Datos, // uint16_t
	    (uint16_t)TIM2->PSC,   // uint32_t
	    (uint16_t)TIM2->ARR   // uint32_t
	);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)config,strlen(config));
	Current_State = STATE_IDLE;
}

void convertir_uint16_to_float32(uint16_t* pOrigen, float32_t* pDestino, uint32_t buffer_length){
	const float32_t conversion_factor = 1.0f / (float32_t)ADC_MAX_VALUE;

    for (uint32_t i = 0; i < buffer_length; i++) {
        // Normalizamos a los valores [0.0, 1.0] y convertimos a float cada uno de los valores
    	pDestino[i] = ((float32_t)pOrigen[i]) * conversion_factor;
    }
}

uint8_t uint16_es_potencia_de_2(uint16_t numero){
	//Si es potencia de 2, entonces solo debe haber un bit encendido en el número
	if((numero & (numero-1)) == 0){
		//Se trata de una potencia de 2
		return 0;
	}
	return 1;
}

void perform_fft(void) {
	convertir_uint16_to_float32(Buffer_Datos, Buffer_Datos_Float, ADC_BUFFER_MAX_LENGTH);
    // --- Paso 1: Realizar la FFT ---
    // Se asume que el buffer del ADC está listo
    arm_rfft_fast_f32(
        &rfft_instance,       	// Puntero a la instancia que se inicializó
		Buffer_Datos_Float,     	// Puntero a los datos de valor real del ADC
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
	convertir_uint16_to_float32(Buffer_Datos, Buffer_Datos_Float, ADC_BUFFER_MAX_LENGTH);
	arm_rms_f32(Buffer_Datos_Float, ADC_BUFFER_MAX_LENGTH, &valor_RMS);	//El tercer argumento debe ser un puntero que apunta a una variable tipo float, se obtiene el valor RMS de la señal
	valor_RMS = valor_RMS * 3.3;	//Originilamente el ADC_Float_Buffer está normalizado entre 0 y 1, lo multiplicamos por 3.3V para hallar la potencia adecuadamente
	float potencia_promedio = valor_RMS * valor_RMS;	//La potencia de la señal es su valor RMS al cuadrado
	// Lo formateamos:
	sprintf(tx_buffer, "Potencia Promedio:        %.2f W\r\n", potencia_promedio);
	//Transmitimos
	status = HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer),100);

	if (status != HAL_OK) {
		// Transmission error
	}


	Current_State = STATE_IDLE;
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
		Current_State = STATE_IDLE;
		return;
	}
	//En caso contrario, procedemos a modificar el valor de la FFT
	fft_Length = fft_new_Length;	//Asignamos el nuevor valor a la FFT
	longitud_Buffer_Datos = fft_Length;
	arm_rfft_fast_init_f32(&rfft_instance, fft_Length);
	const char *doneMsg = "Tamaño de FFT actualizado correctamente\n";
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *) doneMsg, strlen(doneMsg));
	Current_State = STATE_IDLE;
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
		Current_State = STATE_IDLE;
		FSM_update(Current_State);
	}
	FSM_update(Current_State);	//Otra marranada
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)aux_buffer, offset );	//A enviar la data de la FFT
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

	 argumento_comandos = argumento;
	 ID_Comando id = encontrar_id_comando(line);
	 switch (id){
	 	 case CMD_ID_printhelp: 			Current_State = STATE_HELP; 						break;
	 	 case CMD_ID_config_blinky:			Current_State = STATE_CONFIG_BLINKY;				break;
	 	 case CMD_ID_cfg_size_fft:			Current_State = STATE_CONFIG_FFT_SIZE;				break;
	 	 case CMD_ID_print_fft:				Current_State = STATE_PRINT_FFT;					break;
	 	 case CMD_ID_print_config:			Current_State = STATE_PRINT_CONFIG;					break;
	 	 case CMD_ID_print_fft_features:	Current_State = STATE_PRINT_FFT_FEATURES;			break;
	 	 case CMD_ID_UNKNOWN:				unknown();											break;
	 }
}

void FSM_update(State_t State){
	switch (State){
		case STATE_IDLE :{
			//Aquí irá la toma de datos del MCU y el display en la pantalla
			break;
		}
		case STATE_TERMINAL :{
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
		}case STATE_HELP :{
			printhelp(); //Imprime el menú de ayuda
			break;
		}case STATE_CONFIG_BLINKY: {
			config_blinky(argumento_comandos);
			break;
		}case STATE_CONFIG_FFT_SIZE: {
			cfg_size_fft(argumento_comandos);
			break;
		}case STATE_PRINT_FFT:{
			print_fft();
			break;
		}case STATE_PRINT_CONFIG: {
			print_config();
			break;
		}case STATE_PRINT_FFT_FEATURES:{
			print_fft_features();
			break;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(UserLed_GPIO_Port, UserLed_Pin); // UserLed parpadea cada 250ms
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

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
