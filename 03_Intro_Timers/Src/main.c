/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Jose A. Diaz
 * @brief          : Main program body
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "timer_driver_hal.h"

//Definimos un Pin de prueba
GPIO_Handler_t userLed = {0}; //Pin A5

Timer_Handler_t blinkTimer = {0};

//Main function, where everything happens

int main(void){
    //Se configura el Pin

	userLed.pGPIOx							= GPIOA;
	userLed.pinConfig.GPIO_PinNumber		= PIN_5;
	userLed.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed);

	gpio_WritePin(&userLed, SET);

	blinkTimer.pTIMx							= TIM2;
	blinkTimer.TIMx_Config.TIMx_Prescaler		= 16000;	//Genera Incrementos de 1ms
	blinkTimer.TIMx_Config.TIMx_Period			= 250;	//De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_Mode			= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable	= TIMER_INT_ENABLE;

	//Configuramos el Timer
	timer_Config(&blinkTimer);

	//Encedemos el Timer
	timer_SetState(&blinkTimer, TIMER_ON);

	while(1){
		//Loop pa siempre
	}
}

/*
 * OverWrite function
 */
void Timer2_Callback(void){
	gpio_TooglePin(&userLed);

}

/* Esta función sirve para detectar problemas de parámetros
 * Incorrectos al momento de ejecutar el programa
 */

void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		//problemas....
	}
}
