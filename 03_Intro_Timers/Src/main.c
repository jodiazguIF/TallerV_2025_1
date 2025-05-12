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

//Definimos un Pin de prueba
GPIO_Handler_t userLed = {0}; //Pin A5

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

	while(1){
		//Loop pa siempre
	}
}

/* Esta función sirve para detectar problemas de parámetros
 * Incorrectos al momento de ejecutar el programa
 */

void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		//problemas....
	}
}
