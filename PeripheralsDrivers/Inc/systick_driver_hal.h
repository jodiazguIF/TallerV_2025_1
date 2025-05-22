/*
 * systick_driver_hal.h
 *
 *Permite configurar la fuente de reloj, valor de recarga y habilitación
 *de interrupciones del SysTick. También permite iniciar/parar el SysTick
 *
 *  Created on: May 20, 2025
 *      Author: Jose A. Diaz
 */

#ifndef SYSTICK_DRIVER_HAL_H_
#define SYSTICK_DRIVER_HAL_H_

#include "stm32f4xx.h"
#include "stm32_assert.h"

/*
 * SysTick ClockSource
 */
typedef enum {
	SYSTICK_CLOCK_SOURCE_EXTERNAL= 0,
	SYSTICK_CLOCK_SOURCE_PROCCESOR,
} SysTick_ClockSource_t;

/*
 * SysTick Exception Request Mode
 */
typedef enum {
	SYSTICK_TICKINT_DISABLED = 0,
	SYSTICK_TICKINT_ENABLED,
}SysTick_InterruptEnable_t;
enum {
	SYSTICK_OFF = 0,
	SYSTICK_ON
};
//Estructura que contiene la configuración mínima para el manejo del SysTick
typedef struct{
	uint32_t 						SYSTICK_ReloadValue;			//Valor de recarga para generar el periodo deseado, máximo 0xFFFFFF
	SysTick_ClockSource_t 			SYSTICK_ClockSource;			//De dónde proviene la señal de reloj para el SysTick
	SysTick_InterruptEnable_t		SYSTICK_InterruptEnable;		//Activa o desactiva el modo interrupción
} SysTick_Config_t;

/*Handler para el SysTick*/
typedef struct{
	SysTick_Type		*pSysTickx;
	SysTick_Config_t	SysTickConfig;
} SysTick_Handler_t;

/* For testing assert parameters - checking basic configurations */
#define IS_SYSTICK_RELOAD(VALUE)					(((uint32_t)VALUE) > 0 && ((uint32_t)VALUE) < 0xFFFFFF) //Debe ser mayor estricto que 0 y menor estricto que 24 bits
#define IS_SYSTICK_CLOCKSOURCE(VALUE)				(((VALUE) == SYSTICK_CLOCK_SOURCE_EXTERNAL) || ((VALUE) == SYSTICK_CLOCK_SOURCE_PROCCESOR))
#define IS_SYSTICK_INTERRUPT(VALUE)					(((VALUE) == SYSTICK_TICKINT_ENABLED) || ((VALUE) == SYSTICK_TICKINT_DISABLED))
#define IS_SYSTICK_STATE(VALUE)						(((VALUE) == SYSTICK_OFF) || ((VALUE) == SYSTICK_ON))

//Funciones Públicas
void systick_Config(SysTick_Handler_t *pSysTickHandler);
void systick_SetState(SysTick_Handler_t *pSysTickHandler, uint8_t newState);

//El callback del SysTick
void systick_Callback(void);

#endif /* SYSTICK_DRIVER_HAL_H_ */
