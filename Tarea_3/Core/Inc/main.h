/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Se definen los estados finitos que usaremos para la máquina de estados
typedef enum{
	STATE_REFRESH,
	STATE_RGB_FEEDBACK,
	STATE_TAXIMETER_FEEDBACK,
	STATE_CHANGE_REFRESH,
	STATE_TERMINAL_FEEDBACK,
} State_t;

//Se definen los estados finitos que puede tener el led RGB a través de una variable de 8 bits sin signo
typedef enum {
    RGB_OFF 		= 0b000,
    RGB_ROJO 		= 0b001,
    RGB_VERDE 		= 0b010,
    RGB_AZUL 		= 0b100,
    RGB_AZUL_VERDE 	= 0b110,
    RGB_AZUL_ROJO	= 0b101,
    RGB_ROJO_VERDE 	= 0b011,
    RGB_ON 			= 0b111
} RGB_Color_t;

// Se definen las máscaras que se usarán para modificar el estado del LED RGB
#define RGB_RED_MASK   (1 << 0) 	//Mascara para modificar el LED Rojo del RGB
#define RGB_GREEN_MASK (1 << 1)		//Mascara para modificar el LED Verde del RGB
#define RGB_BLUE_MASK  (1 << 2)		//Mascara para modificar el LED Azul del RGB

//Identifica para las frecuencias de muestreo del ADC
typedef enum {
	LOW,
	MEDIUM,
	HIGH,
	ULTRA,
} ADC_Sampling_Freq_t;

// Se definen los valores de periodo del timer para las diferentes frecuencias de muestreo del ADC
static const uint16_t ADC_Sampling_Freq[] = {
		363,   // LOW 	 = 44.1 kHz
		333,   // MEDIUM = 48 kHz
		166,   // HIGH	 = 96 kHz
		125	,  // ULTRA  = 128 kHz
};

//Declaramos los prototipos de las funciones que se usarán en el programa
void printhelp(void);
void rgb_modify(const char *argumento);
void config_blinky(const char *argumento);
void cfg_adc_sampling_freq(const char *argumento);
void cfg_size_fft(const char *argumento);
void print_adc(void);
void print_fft(void);
void print_config(void);
void print_fft_features(void);
void start_adc(void);
void stop_adc(void);
void unknown(void);


//Estructura que contiene los ID de los comandos que vamos a utilizar
typedef enum {
	CMD_ID_printhelp,
	CMD_ID_rgb_modify,
	CMD_ID_config_blinky,
	CMD_ID_cfg_adc_sampling_freq,
	CMD_ID_cfg_size_fft,
	CMD_ID_print_adc,
	CMD_ID_print_fft,
	CMD_ID_print_config,
	CMD_ID_print_fft_features,
	CMD_ID_start_adc,
	CMD_ID_stop_adc,
	CMD_ID_UNKNOWN,
}ID_Comando;

typedef struct {
	const char *command_str; 					// Nombre del comando
	ID_Comando id_comando;
} Comando_t;

//también se crea una tabla de comandos que contiene los comandos disponibles
static const Comando_t comandos[] = {
		{"help"							, CMD_ID_printhelp},
		{"RGB"							, CMD_ID_rgb_modify},
		{"Config_Blinky_Period"			, CMD_ID_config_blinky},
		{"Config_ADC_Sampling_Freq"		, CMD_ID_cfg_adc_sampling_freq},
		{"Config_FFT_Size"				, CMD_ID_cfg_size_fft    },
		{"Print_ADC"					, CMD_ID_print_adc  },
		{"Print_FFT"					, CMD_ID_print_fft  },
		{"Print_Config"					, CMD_ID_print_config},
		{"Print_FFT_Features"			, CMD_ID_print_fft_features},
		{"Start_ADC"					, CMD_ID_start_adc},
		{"Stop_ADC"						, CMD_ID_stop_adc},
};

#define NUM_COMANDOS (sizeof(comandos) / sizeof(Comando_t)) //Cantidad de comandos disponibles

//Ahora vamos a implementar la arquitectura ping-pong para los receive del UART
typedef enum { BUFFER_A, BUFFER_B } BufferActivo;

typedef struct {
    uint8_t*  buffer;
    uint16_t  size;
} Paquete_Datos;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Switch_Encoder_Pin GPIO_PIN_13
#define Switch_Encoder_GPIO_Port GPIOC
#define Switch_Encoder_EXTI_IRQn EXTI15_10_IRQn
#define UserLed_Pin GPIO_PIN_1
#define UserLed_GPIO_Port GPIOH
#define Clk_Encoder_Pin GPIO_PIN_1
#define Clk_Encoder_GPIO_Port GPIOC
#define Clk_Encoder_EXTI_IRQn EXTI1_IRQn
#define ADC_SENAL_Pin GPIO_PIN_1
#define ADC_SENAL_GPIO_Port GPIOA
#define BotonTasaRefrescoIncremento_Pin GPIO_PIN_4
#define BotonTasaRefrescoIncremento_GPIO_Port GPIOA
#define BotonTasaRefrescoIncremento_EXTI_IRQn EXTI4_IRQn
#define RGB_VERDE_Pin GPIO_PIN_5
#define RGB_VERDE_GPIO_Port GPIOA
#define RGB_AZUL_Pin GPIO_PIN_6
#define RGB_AZUL_GPIO_Port GPIOA
#define DigitoD3_Pin GPIO_PIN_5
#define DigitoD3_GPIO_Port GPIOC
#define Data_Encoder_Pin GPIO_PIN_2
#define Data_Encoder_GPIO_Port GPIOB
#define BotonTasaRefrescoDecremento_Pin GPIO_PIN_15
#define BotonTasaRefrescoDecremento_GPIO_Port GPIOB
#define BotonTasaRefrescoDecremento_EXTI_IRQn EXTI15_10_IRQn
#define DigitoD2_Pin GPIO_PIN_6
#define DigitoD2_GPIO_Port GPIOC
#define DigitoD1_Pin GPIO_PIN_8
#define DigitoD1_GPIO_Port GPIOC
#define SieteSegmentosLEDA_Pin GPIO_PIN_9
#define SieteSegmentosLEDA_GPIO_Port GPIOC
#define SieteSegmentosLEDF_Pin GPIO_PIN_11
#define SieteSegmentosLEDF_GPIO_Port GPIOA
#define SieteSegmentosLEDB_Pin GPIO_PIN_12
#define SieteSegmentosLEDB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SieteSegmentosLEDE_Pin GPIO_PIN_10
#define SieteSegmentosLEDE_GPIO_Port GPIOC
#define SieteSegmentosLEDG_Pin GPIO_PIN_11
#define SieteSegmentosLEDG_GPIO_Port GPIOC
#define SieteSegmentosLEDD_Pin GPIO_PIN_12
#define SieteSegmentosLEDD_GPIO_Port GPIOC
#define SieteSegmentosLEDC_Pin GPIO_PIN_2
#define SieteSegmentosLEDC_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DigitoD0_Pin GPIO_PIN_7
#define DigitoD0_GPIO_Port GPIOB
#define RGB_ROJO_Pin GPIO_PIN_8
#define RGB_ROJO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_MAX_LENGTH 64
#define ADC_BUFFER_MAX_LENGTH 2048
#define ADC_MAX_VALUE 4095
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
