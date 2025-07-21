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


//Declaramos los prototipos de las funciones que se usarán en el programa
void printhelp(void);
void unknown(void);
void set_balance_angle(void);


//Estructura que contiene los ID de los comandos que vamos a utilizar
typedef enum {
	CMD_ID_printhelp,
	CMD_ID_Balance,
	CMD_ID_UNKNOWN,
}ID_Comando;

typedef struct {
	const char *command_str; 					// Nombre del comando
	ID_Comando id_comando;
} Comando_t;

// Se definen los estados finitos que usaremos para la máquina de estados
typedef enum{
	STATE_IDLE,
	STATE_HELP,
	STATE_TERMINAL,
	STATE_READ_MCU_DATA,
	STATE_BALANCE,
} State_t;

//también se crea una tabla de comandos que contiene los comandos disponibles
static const Comando_t comandos[] = {
		{"help"							, CMD_ID_printhelp},
		{"balance"						, CMD_ID_Balance},
};

#define NUM_COMANDOS (sizeof(comandos) / sizeof(Comando_t)) //Cantidad de comandos disponibles

//Ahora vamos a implementar la arquitectura ping-pong para los receive del UART
typedef enum { BUFFER_A, BUFFER_B } BufferActivo;

typedef struct {
    uint8_t*  buffer;
    uint16_t  size;
} Paquete_Datos;

typedef enum {
	MOTOR_1 = 0,
	MOTOR_2,
	MOTOR_3
} MotorID_t;


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
#define UserLed_Pin GPIO_PIN_1
#define UserLed_GPIO_Port GPIOH
#define Balance_OK_Pin GPIO_PIN_0
#define Balance_OK_GPIO_Port GPIOC
#define I2C_Error_Pin GPIO_PIN_2
#define I2C_Error_GPIO_Port GPIOC
#define Motor_3_DIR_Pin GPIO_PIN_5
#define Motor_3_DIR_GPIO_Port GPIOA
#define MCU_Data_Ready_Pin GPIO_PIN_4
#define MCU_Data_Ready_GPIO_Port GPIOC
#define I2C_OK_Pin GPIO_PIN_0
#define I2C_OK_GPIO_Port GPIOB
#define Motor_2_BRAKE_Pin GPIO_PIN_10
#define Motor_2_BRAKE_GPIO_Port GPIOB
#define Motor_1_BRAKE_Pin GPIO_PIN_7
#define Motor_1_BRAKE_GPIO_Port GPIOC
#define Motor_1_PWM_Pin GPIO_PIN_8
#define Motor_1_PWM_GPIO_Port GPIOA
#define Motor_1_DIR_Pin GPIO_PIN_9
#define Motor_1_DIR_GPIO_Port GPIOA
#define Motor_2_PWM_Pin GPIO_PIN_10
#define Motor_2_PWM_GPIO_Port GPIOA
#define Motor_3_PWM_Pin GPIO_PIN_11
#define Motor_3_PWM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Motor_2_DIR_Pin GPIO_PIN_5
#define Motor_2_DIR_GPIO_Port GPIOB
#define Motor_3_BRAKE_Pin GPIO_PIN_9
#define Motor_3_BRAKE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_MAX_LENGTH 64
#define ADC_BUFFER_MAX_LENGTH 512
#define ADC_MAX_VALUE 4095

//Definimos los registros del MPU6050
#define MPU6050_ADDR 	(0x68 << 1) // Dirección del MPU6050 (7 bits) << 1 para I2C
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define MCU_WHO_AM_I_REG 0x75
#define MCU_INT_ENABLE 0x38

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
