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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Se definen los estados finitos que usaremos para la máquina de estados
typedef enum{
	STATE_REFRESH,
	STATE_RGB_FEEDBACK,
	STATE_TAXIMETER_FEEDBACK,
	STATE_CHANGE_REFRESH,
} State_t;

//Se definen los estados finitos que puede tener el led RGB
typedef enum{
	APAGADO,
	ROJO,
	VERDE,
	AZUL,
	AZUL_VERDE,
	AZUL_ROJO,
	ROJO_VERDE,
	ROJO_VERDE_AZUL,
}RGB_Color_t ;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define UserLed_Pin GPIO_PIN_1
#define UserLed_GPIO_Port GPIOH
#define Switch_Encoder_Pin GPIO_PIN_0
#define Switch_Encoder_GPIO_Port GPIOC
#define Clk_Encoder_Pin GPIO_PIN_1
#define Clk_Encoder_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BotonTasaRefrescoIncremento_Pin GPIO_PIN_4
#define BotonTasaRefrescoIncremento_GPIO_Port GPIOA
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
