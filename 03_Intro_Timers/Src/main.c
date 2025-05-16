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
GPIO_Handler_t userLed1 = {0};
GPIO_Handler_t userLed2 = {0};
GPIO_Handler_t userLed3 = {0};

Timer_Handler_t blinkTimer = {0};

//Main function, where everything happens
void initSystem(void); 	//Se llama la función para que se pueda ejecutar en el main
void initTimers(void);

int main(void){
	initSystem();
	initTimers();

	gpio_WritePin(&userLed, SET);

	while(1){
		//Loop pa siempre
	}
}

void initSystem(void){
    //Se configura el Pin
	userLed.pGPIOx							= GPIOH;//Pin de estado para la board Táctica GPIOA PIN_5 sin boar Táctica
	userLed.pinConfig.GPIO_PinNumber		= PIN_1;
	userLed.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed);


	userLed1.pGPIOx							= GPIOC;
	userLed1.pinConfig.GPIO_PinNumber		= PIN_9;
	userLed1.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed1.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed1.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed1.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed1);

	userLed2.pGPIOx							= GPIOC;
	userLed2.pinConfig.GPIO_PinNumber		= PIN_8;
	userLed2.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed2.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed2.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed2.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed2);

	userLed3.pGPIOx							= GPIOC;
	userLed3.pinConfig.GPIO_PinNumber		= PIN_6;
	userLed3.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed3.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed3.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed3.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed3);
}

void initTimers(void){
	blinkTimer.pTIMx							= TIM2;
	blinkTimer.TIMx_Config.TIMx_Prescaler		= 16000;	//Genera Incrementos de 1ms
	blinkTimer.TIMx_Config.TIMx_Period			= 250;	//De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_Mode			= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable	= TIMER_INT_ENABLE;
	//Configuramos el Timer
	timer_Config(&blinkTimer);
	//Encedemos el Timer
	timer_SetState(&blinkTimer, TIMER_ON);
}

void timer2_Callback(void){
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
