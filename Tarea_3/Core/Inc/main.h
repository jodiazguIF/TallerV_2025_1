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


typedef void (*function_t)(const char *args); // Definición de un puntero a función que recibe un string como argumento
//Definimos una estructura que contiene el nombre del comando y la función asociada

typedef struct {
	const char *command_name; 		// Nombre del comando
	const char *help; 				// Descripción del comando
	function_t handler;				// Puntero a la función asociada al comando
} Entrada_Comando_t;

//Declaramos los prototipos de las funciones que se usarán en el programa
static void printhelp(void);
static void rgb_on(void);
static void rgb_off(void);
static void rgb_toogle_red(void);
static void rgb_toogle_blue(void);
static void rgb_toogle_green(void);
static void config_blinky(const char *argumento);
static void cfg_sample_rate(const char *argumento);
static void cfg_size_fft(const char *argumento);
static void print_adc(void);
static void print_fft(void);
static void print_config(void);
static void print_fft_features(void);

//también se crea una tabla de comandos que contiene los comandos disponibles

static const Entrada_Comando_t comandos[] = {
		{"help"						,"Muestra esta ayuda"					, printhelp},
		{"RGB_ON"					,"Enciende el LED RGB"					, rgb_on},
		{"RGB_OFF"					,"Apaga el LED RGB"						, rgb_off},
		{"RGB_RED"					,"Toggle Led Rojo del RGB"				, rgb_toogle_red},
		{"RGB_BLUE"					,"Toggle Led Azul del RGB"				, rgb_toogle_blue},
		{"RGB_GREEN"				,"Toggle Led Verde del RGB"				, rgb_toogle_green},
		{"Config_Blinky_Period"		,"Configura periodo blinky [1‑15999]"	, config_blinky},
		{"Config_Sampling_Time"		,"Configura tiempo muestreo"			, cfg_sample_rate},
		{"Config_FFT_Size"			,"Configura tamaño FFT"					, cfg_size_fft    },
		{"Print_ADC"				,"Envía muestras ADC"					, print_adc  },
		{"Print_FFT"				,"Envía espectro FFT"					, print_fft  },
		{"Print_Config"				,"Imprime la configuración del equipo"	, print_config},
		{"Print_FFT_Features"		,"Imprime valores importantes de la FFT", print_fft_features},
};
#define NUM_COMANDOS (sizeof(comandos) / sizeof(Entrada_Comando_t)) //Cantidad de comandos disponibles

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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
