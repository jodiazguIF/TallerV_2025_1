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
SysTick_Type *ptrSystickUsed;
uint32_t COUNTERFLAG = 0;

/* Headers para funciones privadas */
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
	ptrSystickUsed = pSysTickHandler->pSysTickx;

	//0. Desativamos las interrupciones globales mientras configuramos el sistema
	__disable_irq();

	//1. Se configura el valor de Recarga
	systick_set_reload_value(pSysTickHandler);

	//2. Se configura el origen de la señal de reloj
	systick_clock_source(pSysTickHandler);

	//3. Se configura la interrupción
	systick_config_interrupt(pSysTickHandler);

	//Se vuelve a activar las interrupciones del sistema
	__enable_irq();

	//El SysTick inicia apagado
	systick_SetState(pSysTickHandler, SYSTICK_OFF);
}

void systick_set_reload_value(SysTick_Handler_t *pSysTickHandler){
	//Verificamos que el valor del reload es válido
	assert_param(IS_SYSTICK_RELOAD(pSysTickHandler->SysTickConfig.SYSTICK_ReloadValue));

	//Se condigura el valor del autoreload
	pSysTickHandler->pSysTickx->LOAD = pSysTickHandler->SysTickConfig.SYSTICK_ReloadValue;
}

void systick_clock_source(SysTick_Handler_t *pSysTickHandler){
	//Verificamos que el origen de la señal de reloj es válida
	assert_param(IS_SYSTICK_CLOCKSOURCE(pSysTickHandler->SysTickConfig.SYSTICK_ClockSource));
	//Luego, limpiamos el registro
	pSysTickHandler->pSysTickx->CTRL &= ~(1 << SysTick_CTRL_CLKSOURCE_Pos);

	//Verificamos cuál es el modo que se desea configurar
	if (pSysTickHandler->SysTickConfig.SYSTICK_ClockSource == SYSTICK_CLOCK_SOURCE_EXTERNAL){
		//Se configura el origen del clock como externo Ext_Clk = 0
		pSysTickHandler->pSysTickx->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk);
	}else{
		//Se configura el origen como del procesador, Proc_Clk = 1
		pSysTickHandler->pSysTickx->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	}
}

void systick_config_interrupt(SysTick_Handler_t *pSysTickHandler){
	//Verificamos que se está ingresando una opción válida
	assert_param(IS_SYSTICK_INTERRUPT(pSysTickHandler->SysTickConfig.SYSTICK_InterruptEnable));
	//Se limpia lo que haya en el bit
	pSysTickHandler->pSysTickx->CTRL &= ~(1 << SysTick_CTRL_TICKINT_Pos);
	//Luego se activa o desactiva la interrupción según corresponda
	if(pSysTickHandler->SysTickConfig.SYSTICK_InterruptEnable == SYSTICK_TICKINT_DISABLED){
		//Se desactivan, 0 en el registro de tickint
		pSysTickHandler->pSysTickx->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	}else{
		//Se activan, 1 en el registro
		pSysTickHandler->pSysTickx->CTRL |= SysTick_CTRL_TICKINT_Msk;
		NVIC_EnableIRQ(SysTick_IRQn);
	}

}
void systick_SetState(SysTick_Handler_t *pSysTickHandler, uint8_t newState){
	//Verificamos que el estado ingresado es adecuado
	assert_param(IS_SYSTICK_STATE(newState));
	//Reiniciamos el registro del count
	pSysTickHandler->pSysTickx-> VAL = 0;

	if(newState == SYSTICK_ON){
		//Activamos el SysTick
		pSysTickHandler->pSysTickx->CTRL |= SysTick_CTRL_ENABLE_Msk;
	}else{
		//Desactivamos el SysTick
		pSysTickHandler->pSysTickx->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	}

}

__attribute__((weak)) void SysTick_Callback(void){
	__NOP();
}

void SysTick_Handler(void){
	//Se limpia la bandera que indica que la interrupción se ha generado
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);

	//Se llama a la función que se debe encargar de hacer algo con esa interrupción
	SysTick_Callback();
}
