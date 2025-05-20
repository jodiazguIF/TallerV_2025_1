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


//Definimos lo que vamos a utilizar en el código
GPIO_Handler_t userLed = {0};
//Se definen los pines de los 7 segmentos
GPIO_Handler_t SieteSegmentosLEDA = {0};
GPIO_Handler_t SieteSegmentosLEDB = {0};
GPIO_Handler_t SieteSegmentosLEDC = {0};
GPIO_Handler_t SieteSegmentosLEDD = {0};
GPIO_Handler_t SieteSegmentosLEDE = {0};
GPIO_Handler_t SieteSegmentosLEDF = {0};
GPIO_Handler_t SieteSegmentosLEDG = {0};



Timer_Handler_t blinkTimer = {0};

//Main function, where everything happens
void initSystem(void); 	//Se llama la función para que se pueda ejecutar en el main
void initTimers(void);

int main(void){
	initSystem();
	initTimers();

	while(1){
		//Loop pa siempre
	}
}

void initSystem(void){
    //Se configura el Pin
	userLed.pGPIOx							= GPIOH;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	userLed.pinConfig.GPIO_PinNumber		= PIN_1;
	userLed.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	userLed.pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_PUSHPULL;
	userLed.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	userLed.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&userLed);
	gpio_WritePin(&userLed, SET);

	//Se configuran los pines del 7 segmentos, LED A
	SieteSegmentosLEDA.pGPIOx							= GPIOA;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDA.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDA.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDA.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDA.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDA.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDA);
	gpio_WritePin(&SieteSegmentosLEDA, RESET);

	//LED B
	SieteSegmentosLEDB.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDB.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDB.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDB.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDB.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDB.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDB);
	gpio_WritePin(&SieteSegmentosLEDB, RESET);

	//LED C
	SieteSegmentosLEDC.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDC.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDC.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDC.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDC.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDC.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDC);
	gpio_WritePin(&SieteSegmentosLEDC, RESET);

	//LED D
	SieteSegmentosLEDD.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDD.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDD.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDD.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDD.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDD.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDD);
	gpio_WritePin(&SieteSegmentosLEDD, RESET);

	//LED E
	SieteSegmentosLEDE.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDE.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDE.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDE.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDE.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDE.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDE);
	gpio_WritePin(&SieteSegmentosLEDE, SET);

	//LED F
	SieteSegmentosLEDF.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDF.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDF.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDF.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDF.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDF.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDF);
	gpio_WritePin(&SieteSegmentosLEDF, SET);

	//LED G
	SieteSegmentosLEDG.pGPIOx							= GPIOB;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDG.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDG.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDG.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDG.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDG.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDG);
	gpio_WritePin(&SieteSegmentosLEDG, SET);


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
