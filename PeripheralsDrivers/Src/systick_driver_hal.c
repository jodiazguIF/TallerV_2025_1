/*
 * systick_driver_hal.c
 *
 *  Created on: May 20, 2025
 *      Author: Jose A. Diaz
 */

#include "stm32f4xx.h"
#include "stm32_assert.h"
#include "systick_driver_hal.h"

//Variable que guarda la referencia de lo que se está usando
SysTick_Type *pSystickUsed;

/* Headers para funciones privadas */
static void systick_enable(SysTick_Handler_t *pSysTickHandler);
static void systick_set_reload_value(SysTick_Handler_t *pSysTickHandler);
static void systick_clock_source(SysTick_Handler_t *pSysTickHandler);
static void systick_config_interrupt(SysTick_Handler_t *pSysTickHandler);

/*
 * Función en la que cargamos la configuración del SysTick
 * Siempre se debe comenzar activando el SysTick
 * Se debe ser cuidadoso al momento de utilizar las interrupciones
 * Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 * el sistema global de interrupciones, activar la IRQ específica y luego volver a encender
 * el sistema.
 */

void systick_Config(SysTick_Handler_t *pSysTickHandler){
	// Se guarda una referencia al systick en usp
	pSystickUsed = pSysTickHandler;

	//0. Desativamos las interrupciones globales mientras configuramos el sistema
	__disable_irq();
	//1. Activar la señal de reloj del SysTick
	systick_enable(pSysTickHandler);

	//2. Se configura el valor de Recarga
	systick_set_reload_value(pSysTickHandler);

	//3. Se configura el origen de la señal de reloj
	systick_clock_source(pSysTickHandler);

	//4. Se configura la interrupción
	systick_config_interrupt(pSysTickHandler);

	//Se vuelve a activar las interrupciones del sistema
	__enable_irq();

	//El SysTick inicia apagado
	systick_SetState(pSysTickHandler, SYSTICK_OFF);
}

void systick_enable(SysTick_Handler_t *pSysTickHandler){
	//Se limpia el registro que contiene el enable
	SysTick->CTRL &= ~(1 << SysTick_CTRL_ENABLE_Pos);
	//Se activa el SysTick
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk);
}


















