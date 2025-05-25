/*
 * timer_driver_hal.c
 *
 *  Created on: May 11, 2025
 *      Author: Jose A. Diaz
 */

#include "stm32f4xx.h"
#include "stm32_assert.h"
#include "timer_driver_hal.h"

/*Variable que guarda la referencia del periférico que se está usando*/
TIM_TypeDef *ptrTimerUsed;

/*Headers para funciones privadas*/
static void timer_enable_clock_peripheral(Timer_Handler_t *pTimerHandler);
static void timer_set_prescaler(Timer_Handler_t *pTimerHandler);
static void timer_set_period(Timer_Handler_t *pTimerHandler);
static void timer_set_mode(Timer_Handler_t *pTimerHandler);
static void timer_config_interrupt(Timer_Handler_t *pTimerHandler);

/* Función en la que cargamos la configuración del Timer
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periférico que se está utilizando.
 * Ademáś, en este caso, debemos ser cuidados al momento de utilizar las interrupciones.
 * Los timer están conectados directamente al elemento NVIC del Cortex-Mx
 * Debemos configurar y/o utilizar:
 * - TIMx_CR1	(control Register 1)
 * - TIMx_SMCR 	(slave mode control register) -> mantener en 0 para modo Timer Básico
 * - TIMx_DIER	(DMA and Interrupt enable register)
 * - TIMx_SR	(Status Register)
 * - TIMx_CNT 	(Counter)
 * - TIMx_PSC 	(Pre-scaler)
 * - TIMx_ARR	(Auto-reload register)
 *
 * Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 * el sistema global de interrupciones, activar la IRQ específica y luego volver a encender
 * el sistema.
 */
void timer_Config(Timer_Handler_t *pTimerHandler){
	//Guardamos una referencia al periférico que estemos usando
	ptrTimerUsed = pTimerHandler->pTIMx;

	//0. Desativamos las interrupciones globales mientras configuramos el sistema
	__disable_irq();

	//1. Activar la señal de reloj del periférico requerido
	timer_enable_clock_peripheral(pTimerHandler);

	//2. Configuramos el Pres-Scaler
	timer_set_prescaler(pTimerHandler);

	//3. Configuramos si UP_COUNT o DOWN_COUNT (mode)
	timer_set_mode(pTimerHandler);

	//4. Configuramos el Auto-reload
	timer_set_period(pTimerHandler);

	//5.Configuramos la interrupción
	timer_config_interrupt(pTimerHandler);

	//6. Volvemos a activar las interrupciones del sistema
	__enable_irq();

	//EL timer inicia apagado
	timer_SetState(pTimerHandler, TIMER_OFF);
}

void timer_enable_clock_peripheral(Timer_Handler_t *pTimerHandler){
	//Verificamos que es un timer permitido
	assert_param(IS_TIM_INSTANCE(pTimerHandler->pTIMx));

	if(pTimerHandler->pTIMx == TIM2){
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}else if(pTimerHandler->pTIMx == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}else if(pTimerHandler->pTIMx == TIM4){
		RCC ->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}else if(pTimerHandler->pTIMx == TIM5){
		RCC ->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}else if(pTimerHandler->pTIMx == TIM9){
		RCC ->APB2ENR |= RCC_APB2ENR_TIM9EN;
	}else if(pTimerHandler->pTIMx == TIM10){
		RCC ->APB2ENR |= RCC_APB2ENR_TIM10EN;
	}else if(pTimerHandler->pTIMx == TIM11){
		RCC ->APB2ENR |= RCC_APB2ENR_TIM11EN;
	}else{
		__NOP();
	}
}

/*
 * El pre-scaler nos configura la velocidad a la que se incrementa el registro
 * CNT del Timer
 */

void timer_set_prescaler(Timer_Handler_t *pTimerHandler){
	// Verificamos que el valor de pre-scaler es válido
	assert_param(IS_TIMER_PRESC(pTimerHandler->TIMx_Config.TIMx_Prescaler));

	//Configuramos el valor del pre-scaler
	pTimerHandler->pTIMx->PSC = pTimerHandler->TIMx_Config.TIMx_Prescaler - 1;
}

/*
 * Esta función configura el límite hasta donde cuenta el Timer para generar un
 * evento "update" (cuando está contando de forma ascendente), o configura
 * el valor desde donde se comienza a contar, cuando el sistema funciona de forma
 * descendente
 */

void timer_set_period(Timer_Handler_t *pTimerHandler){
	//Verificamos que el valor que genera el periodo es válido
	assert_param(IS_TIMER_PERIOD(pTimerHandler->TIMx_Config.TIMx_Period));

	//Configuramos el valor del autoreload
	pTimerHandler->pTIMx->ARR = pTimerHandler->TIMx_Config.TIMx_Period- 1;
}

/*
 * UpCounter or DownCounter
 */
void timer_set_mode(Timer_Handler_t *pTimerHandler){
	//Verificamos que el modo de funcionamiento es correcto.
	assert_param(IS_TIMER_MODE(pTimerHandler->TIMx_Config.TIMx_MODE));

	//Verificamos cuál es el modo que se desea configurar
	if(pTimerHandler->TIMx_Config.TIMx_Mode == TIMER_UP_COUNTER){
		//Configuramos en modo UpCounter DIR = 0
		pTimerHandler->pTIMx->CR1 &= ~TIM_CR1_DIR;
	}else{
		//Configuramos el modo DownCounter DIR = 1
		pTimerHandler->pTIMx->CR1 |= TIM_CR1_DIR;
	}
}

/*
 *
 */
void timer_config_interrupt(Timer_Handler_t *pTimerHandler){
	//Verificamo el posible valor configurado
	assert_param(IS_TIMER_INTERRUPT(pTimerHandler->TIMx_Config.InterruptEnable));

	if(pTimerHandler->TIMx_Config.TIMx_InterruptEnable == TIMER_INT_ENABLE){
		//Activamos la interrupcion debida al Timerx Utilizado
		pTimerHandler->pTIMx->DIER |= TIM_DIER_UIE;

		//Activamos el canal del sistema NVIC para que lea la interrupción
		if(pTimerHandler->pTIMx == TIM2){
			NVIC_EnableIRQ(TIM2_IRQn);
		}else if(pTimerHandler->pTIMx == TIM3){
			NVIC_EnableIRQ(TIM3_IRQn);
		}else if(pTimerHandler->pTIMx == TIM4){
			NVIC_EnableIRQ(TIM4_IRQn);
		}else if(pTimerHandler->pTIMx == TIM5){
			NVIC_EnableIRQ(TIM5_IRQn);
		}else{
			__NOP();
		}
	}
}

/*
 *
 */
void timer_SetState(Timer_Handler_t *pTimerHandler, uint8_t newState){
	//Verificamos que el estado ingresado es adecuado
	assert_param(IS_TIMER_STATE(newState));

	//4. Reiniciamos el registro counter
	pTimerHandler->pTIMx->CNT = 0;

	if(newState == TIMER_ON){
		// 5a. Activamos el Timer (El CNT debe comenzar a contar)
		pTimerHandler->pTIMx->CR1 |= TIM_CR1_CEN;
	}else{
		// 5b. Desactivamos el Timer (El CNT debe detenerse)
		pTimerHandler->pTIMx->CR1 &= ~TIM_CR1_CEN;
	}
}
 /*
  *
  */

__attribute__((weak)) void timer2_Callback(void){
	__NOP();
}
__attribute__((weak)) void timer3_Callback(void){
	__NOP();
}
__attribute__((weak)) void timer4_Callback(void){
	__NOP();
}
__attribute__((weak)) void timer5_Callback(void){
	__NOP();
}

/*
 * Esta es la función que apunta al sistema en el vector de interrupciones.
 * Se debe utilizar usando exactamente el mismo nombre definido en el vector de interrupciones
 * AL hacerlo correcta, el sistema apunta a esta función y cuando la interrupción se lanza
 * el sistema inmediatamente salta a este ligar de la memoria
 */

void TIM2_IRQHandler(void){
	//Limpiamos la bandera que indica que la interrupción se ha generado
	TIM2->SR &= ~TIM_SR_UIF;

	//Llamamos a la función que se debe encargar de hacer algo con esta interrupción
	timer2_Callback();
}
void TIM3_IRQHandler(void){
	//Limpiamos la bandera que indica que la interrupción se ha generado
	TIM3->SR &= ~TIM_SR_UIF;

	//Llamamos a la función que se debe encargar de hacer algo con esta interrupción
	timer3_Callback();
}
void TIM4_IRQHandler(void){
	//Limpiamos la bandera que indica que la interrupción se ha generado
	TIM4->SR &= ~TIM_SR_UIF;

	//Llamamos a la función que se debe encargar de hacer algo con esta interrupción
	timer4_Callback();
}
void TIM5_IRQHandler(void){
	//Limpiamos la bandera que indica que la interrupción se ha generado
	TIM5->SR &= ~TIM_SR_UIF;

	//Llamamos a la función que se debe encargar de hacer algo con esta interrupción
	timer5_Callback();
}
