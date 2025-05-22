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
#include "exti_driver_hal.h"

// Se definen los estados finitos que usaremos para la máquina de estados
typedef enum{
	STATE_IDLE,
	STATE_RGB_FEEDBACK,
	STATE_TAXIMETER_FEEDBACK
} State_t;
State_t Current_State = STATE_IDLE; // Se fija el estado por defecto

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

RGB_Color_t Current_Color = APAGADO;	//Se fija el estado inicial

//Variable que lleva el contador completo del taxímetro
uint16_t contador_Taximetro = 0000;
//Variable que lleva el dato del dígito que se enciende en el taxímetro
short digito = 0;

//Variables auxiliares para llevar los digitos que se deben mostrar en el taxímetro
uint8_t miles_contador_Taximetro 	= 0;
uint8_t centenas_contador_Taximetro = 0;
uint8_t decenas_contador_Taximetro 	= 0;
uint8_t unidades_contador_Taximetro = 0;

//Se define el LED de estado
GPIO_Handler_t userLed = {0};

//Pin de Interrupción para el Switch
GPIO_Handler_t gpio_SwitchPin = {0};
EXTI_Handler_t exti_SwitchPin = {0};
//Pines de Interrupción para el Encoder
GPIO_Handler_t gpio_CLKPin = {0};
EXTI_Handler_t exti_CLKPin = {0};
GPIO_Handler_t gpio_DATAPin = {0};


//Se definen los pines de los 7 segmentos
GPIO_Handler_t SieteSegmentosLEDA = {0};
GPIO_Handler_t SieteSegmentosLEDB = {0};
GPIO_Handler_t SieteSegmentosLEDC = {0};
GPIO_Handler_t SieteSegmentosLEDD = {0};
GPIO_Handler_t SieteSegmentosLEDE = {0};
GPIO_Handler_t SieteSegmentosLEDF = {0};
GPIO_Handler_t SieteSegmentosLEDG = {0};

//Se definen los pines que encienden o apagan cada Digito
GPIO_Handler_t DigitoD3 = {0};	//Miles
GPIO_Handler_t DigitoD2 = {0};	//Centenas
GPIO_Handler_t DigitoD1 = {0};	//Decenas
GPIO_Handler_t DigitoD0 = {0};	//Unidades

//Se definen los pines que manejan el RGB
GPIO_Handler_t RGB_AZUL = {0};
GPIO_Handler_t RGB_ROJO = {0};
GPIO_Handler_t RGB_VERDE = {0};

//Se define el timer del led de estado
Timer_Handler_t blinkTimer = {0};
//Se define el timer para el taxímetro
Timer_Handler_t taxiTimer = {0};

//Funciones que se usan en el main
void initPortPin(void);
void initTimers(void);
void initEXTI(void);
void FSM_update(State_t State);

//Main function, where everything happens
int main(void){
	initPortPin();
	initTimers();
	initEXTI();

	while(1){
		FSM_update(Current_State);
	}
}

void initPortPin(void){
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
	SieteSegmentosLEDA.pGPIOx							= GPIOC;
	SieteSegmentosLEDA.pinConfig.GPIO_PinNumber			= PIN_9;
	SieteSegmentosLEDA.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDA.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDA.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDA.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDA);
	gpio_WritePin(&SieteSegmentosLEDA, RESET);

	//LED B
	SieteSegmentosLEDB.pGPIOx							= GPIOA;//Pin de estado para la board Táctica GPIOA PIN_5 sin board Táctica
	SieteSegmentosLEDB.pinConfig.GPIO_PinNumber			= PIN_12;
	SieteSegmentosLEDB.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDB.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDB.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDB.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDB);
	gpio_WritePin(&SieteSegmentosLEDB, RESET);

	//LED C
	SieteSegmentosLEDC.pGPIOx							= GPIOD;
	SieteSegmentosLEDC.pinConfig.GPIO_PinNumber			= PIN_2;
	SieteSegmentosLEDC.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDC.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDC.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDC.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDC);
	gpio_WritePin(&SieteSegmentosLEDC, RESET);

	//LED D
	SieteSegmentosLEDD.pGPIOx							= GPIOC;
	SieteSegmentosLEDD.pinConfig.GPIO_PinNumber			= PIN_12;
	SieteSegmentosLEDD.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDD.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDD.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDD.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDD);
	gpio_WritePin(&SieteSegmentosLEDD, RESET);

	//LED E
	SieteSegmentosLEDE.pGPIOx							= GPIOC;
	SieteSegmentosLEDE.pinConfig.GPIO_PinNumber			= PIN_10;
	SieteSegmentosLEDE.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDE.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDE.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDE.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDE);
	gpio_WritePin(&SieteSegmentosLEDE, RESET);

	//LED F
	SieteSegmentosLEDF.pGPIOx							= GPIOA;
	SieteSegmentosLEDF.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDF.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDF.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDF.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDF.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDF);
	gpio_WritePin(&SieteSegmentosLEDF, RESET);

	//LED G
	SieteSegmentosLEDG.pGPIOx							= GPIOC;
	SieteSegmentosLEDG.pinConfig.GPIO_PinNumber			= PIN_11;
	SieteSegmentosLEDG.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	SieteSegmentosLEDG.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	SieteSegmentosLEDG.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	SieteSegmentosLEDG.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&SieteSegmentosLEDG);
	gpio_WritePin(&SieteSegmentosLEDG, SET);

	//Digito D0
	DigitoD0.pGPIOx								= GPIOB;
	DigitoD0.pinConfig.GPIO_PinNumber			= PIN_7;
	DigitoD0.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	DigitoD0.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	DigitoD0.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	DigitoD0.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&DigitoD0);
	gpio_WritePin(&DigitoD0, RESET);

	//Digito D1
	DigitoD1.pGPIOx								= GPIOC;
	DigitoD1.pinConfig.GPIO_PinNumber			= PIN_8;
	DigitoD1.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	DigitoD1.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	DigitoD1.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	DigitoD1.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&DigitoD1);
	gpio_WritePin(&DigitoD1, RESET);

	//Digito D2
	DigitoD2.pGPIOx								= GPIOC;
	DigitoD2.pinConfig.GPIO_PinNumber			= PIN_6;
	DigitoD2.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	DigitoD2.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	DigitoD2.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	DigitoD2.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&DigitoD2);
	gpio_WritePin(&DigitoD2, RESET);

	//Digito D3
	DigitoD3.pGPIOx								= GPIOC;
	DigitoD3.pinConfig.GPIO_PinNumber			= PIN_5;
	DigitoD3.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	DigitoD3.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	DigitoD3.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	DigitoD3.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&DigitoD3);
	gpio_WritePin(&DigitoD3, RESET);

	//RGB ROJO
	RGB_ROJO.pGPIOx								= GPIOB;
	RGB_ROJO.pinConfig.GPIO_PinNumber			= PIN_8;
	RGB_ROJO.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	RGB_ROJO.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	RGB_ROJO.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	RGB_ROJO.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&RGB_ROJO);
	gpio_WritePin(&RGB_ROJO, RESET);

	//RGB VERDE
	RGB_VERDE.pGPIOx							= GPIOA;
	RGB_VERDE.pinConfig.GPIO_PinNumber			= PIN_5;
	RGB_VERDE.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	RGB_VERDE.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	RGB_VERDE.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	RGB_VERDE.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&RGB_VERDE);
	gpio_WritePin(&RGB_VERDE, RESET);

	//RGB AZUL
	RGB_AZUL.pGPIOx								= GPIOA;
	RGB_AZUL.pinConfig.GPIO_PinNumber			= PIN_6;
	RGB_AZUL.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	RGB_AZUL.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	RGB_AZUL.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	RGB_AZUL.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Se carga la configuración en los registro que gobiernan el puerto
	gpio_Config(&RGB_AZUL);
	gpio_WritePin(&RGB_AZUL, RESET);

	//Se configura el pin del Data
	gpio_DATAPin.pGPIOx									= GPIOB;
	gpio_DATAPin.pinConfig.GPIO_PinNumber				= PIN_2;
	gpio_DATAPin.pinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	gpio_DATAPin.pinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	gpio_Config(&gpio_DATAPin);

}

void initTimers(void){
	blinkTimer.pTIMx							= TIM2;		//Timer básico
	blinkTimer.TIMx_Config.TIMx_Prescaler		= 16000;	//Genera Incrementos de 1ms
	blinkTimer.TIMx_Config.TIMx_Period			= 250;		//Interrupción cada 250ms
	blinkTimer.TIMx_Config.TIMx_Mode			= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable	= TIMER_INT_ENABLE;
	//Configuramos el Timer
	timer_Config(&blinkTimer);
	//Encedemos el Timer
	timer_SetState(&blinkTimer, TIMER_ON);

	taxiTimer.pTIMx								= TIM3;		//Se usa un Timer diferente
	taxiTimer.TIMx_Config.TIMx_Prescaler		= 16000; 	//Incrementos de 1ms
	taxiTimer.TIMx_Config.TIMx_Period			= 5;		//Interrupción cada 7ms
	taxiTimer.TIMx_Config.TIMx_Mode				= TIMER_UP_COUNTER;
	taxiTimer.TIMx_Config.TIMx_InterruptEnable	= TIMER_INT_ENABLE;
	timer_Config(&taxiTimer);				//Se configura el Timer
	timer_SetState(&taxiTimer, TIMER_ON);	//Se enciende el Timer
}

void initEXTI(void){
	//Se configura el pin del Switch
	gpio_SwitchPin.pGPIOx								= GPIOC;
	gpio_SwitchPin.pinConfig.GPIO_PinNumber				= PIN_0;
	gpio_SwitchPin.pinConfig.GPIO_PinMode				= GPIO_MODE_IN;
	gpio_SwitchPin.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	//Ahora se debe configurar el EXTI para este pin
	exti_SwitchPin.pGPIOHandler 						= &gpio_SwitchPin;
	exti_SwitchPin.edgeType								= EXTERNAL_INTERRUPT_RISING_EDGE;
	//Luego se carga la configuración de ambos pines
	gpio_Config(&gpio_SwitchPin);
	exti_Config(&exti_SwitchPin);

	//Se configura el pin del Clock
	gpio_CLKPin.pGPIOx									= GPIOC;
	gpio_CLKPin.pinConfig.GPIO_PinNumber				= PIN_1;
	gpio_CLKPin.pinConfig.GPIO_PinMode					= GPIO_MODE_IN;
	gpio_CLKPin.pinConfig.GPIO_PinPuPdControl			= GPIO_PUPDR_NOTHING;
	//Se configura su EXTI
	exti_CLKPin.pGPIOHandler							= &gpio_CLKPin;
	exti_CLKPin.edgeType								= EXTERNAL_INTERRUPT_RISING_EDGE;
	//Se inicializa
	gpio_Config(&gpio_CLKPin);
	exti_Config(&exti_CLKPin);

}

void cambioEstadoLEDRGB(RGB_Color_t Color){
	switch (Color){
	/*
	 * Esta función lleva el switch del ciclo deseado para el LED RGB
	 * Encendiendo o apagando el LED según corresponda en el orden impuesto
	 */
		case APAGADO: {
			gpio_WritePin(&RGB_AZUL, RESET);
			gpio_WritePin(&RGB_ROJO, RESET);
			gpio_WritePin(&RGB_VERDE, RESET);
			break;
		}case ROJO: {
			gpio_WritePin(&RGB_AZUL, RESET);
			gpio_WritePin(&RGB_ROJO, SET);
			gpio_WritePin(&RGB_VERDE, RESET);
			break;
		}case AZUL: {
			gpio_WritePin(&RGB_AZUL, SET);
			gpio_WritePin(&RGB_ROJO, RESET);
			gpio_WritePin(&RGB_VERDE, RESET);
			break;
		}case VERDE: {
			gpio_WritePin(&RGB_AZUL, RESET);
			gpio_WritePin(&RGB_ROJO, RESET);
			gpio_WritePin(&RGB_VERDE, SET);
			break;
		}case AZUL_VERDE: {
			gpio_WritePin(&RGB_AZUL, SET);
			gpio_WritePin(&RGB_ROJO, RESET);
			gpio_WritePin(&RGB_VERDE, SET);
			break;
		}case AZUL_ROJO: {
			gpio_WritePin(&RGB_AZUL, SET);
			gpio_WritePin(&RGB_ROJO, SET);
			gpio_WritePin(&RGB_VERDE, RESET);
			break;
		}case ROJO_VERDE: {
			gpio_WritePin(&RGB_AZUL, RESET);
			gpio_WritePin(&RGB_ROJO, SET);
			gpio_WritePin(&RGB_VERDE, SET);
			break;
		}case ROJO_VERDE_AZUL: {
			gpio_WritePin(&RGB_AZUL, SET);
			gpio_WritePin(&RGB_ROJO, SET);
			gpio_WritePin(&RGB_VERDE, SET);
			break;
		}
	}
}

void callback_ExtInt0(void){	//función callback para el EXTI0
	Current_State = STATE_RGB_FEEDBACK;	//Al identificar un flanco de subida en el switch se cambia al estado RGB
}

void callback_ExtInt1(void){	//función callback para el EXTI1
	Current_State = STATE_TAXIMETER_FEEDBACK; //Se identifica un flanco de subida en el clock y se pasa rápidamente al estado que cambia el número del taxímetro
}

void timer2_Callback(void){		//función callback para el Timer2
	gpio_TooglePin(&userLed);	//La interrupción solamente hace un blinky para el led de estado
}

void timer3_Callback(void){		//función callback para el TImer3, la cual lleva el refresco indepenediente del 7 segmentos de 4 digitos
	digito++;	//Cambia al siguiente dígito del 7 segmentos
}

void separarContador(void){
	/*
	 * Esta función asigna a estas variables el número correspondiente para que pueda mostrarse
	 * el número adecuado en el taxímetro
	 */
	miles_contador_Taximetro = contador_Taximetro / 1000;
	centenas_contador_Taximetro = (contador_Taximetro % 1000) / 100;
	decenas_contador_Taximetro = (contador_Taximetro % 100) / 10;
	unidades_contador_Taximetro = contador_Taximetro % 10;
}

void setNumeroSieteSegmentos(uint8_t numero){
	/*
	 * Con esta función que implementa un switch-case, es posible encender cualquier número del 0-9 en el 7 segmentos
	 * Su retorno es entonces en el cambio de dígito en el 7 segmentos
	 */
	switch (numero){
		case 0:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, RESET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, SET);
			break;
		}case 1:{
			gpio_WritePin(&SieteSegmentosLEDA, SET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, SET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, SET);
			gpio_WritePin(&SieteSegmentosLEDG, SET);
			break;
		}case 2:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, SET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, RESET);
			gpio_WritePin(&SieteSegmentosLEDF, SET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 3:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, SET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 4:{
			gpio_WritePin(&SieteSegmentosLEDA, SET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, SET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 5:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, SET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 6:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, SET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, RESET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 7:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, SET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, SET);
			gpio_WritePin(&SieteSegmentosLEDG, SET);
			break;
		}case 8:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, RESET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}case 9:{
			gpio_WritePin(&SieteSegmentosLEDA, RESET);
			gpio_WritePin(&SieteSegmentosLEDB, RESET);
			gpio_WritePin(&SieteSegmentosLEDC, RESET);
			gpio_WritePin(&SieteSegmentosLEDD, RESET);
			gpio_WritePin(&SieteSegmentosLEDE, SET);
			gpio_WritePin(&SieteSegmentosLEDF, RESET);
			gpio_WritePin(&SieteSegmentosLEDG, RESET);
			break;
		}
	}
}

void displaySieteSegmentos(short digito){
	switch (digito){
		case 0 :{
			gpio_WritePin(&DigitoD3, SET);							//Se desactiva el digito anterior
			setNumeroSieteSegmentos(unidades_contador_Taximetro);	//Se asigna el número correspondiente a las unidades
			gpio_WritePin(&DigitoD0, RESET);						//Se muestra el número en el dígito 0
			break;
		}case 1:{
			gpio_WritePin(&DigitoD0, SET);							//Se desactiva el digito anterior
			setNumeroSieteSegmentos(decenas_contador_Taximetro);	//Se asigna el número correspondiente a las decenas
			gpio_WritePin(&DigitoD1, RESET);						//Se muestra el número en el dígito 1
			break;
		}case 2:{
			gpio_WritePin(&DigitoD1, SET);							//Se desactiva el digito anterior
			setNumeroSieteSegmentos(centenas_contador_Taximetro);	//Se asigna el número correspondiente a las centenas
			gpio_WritePin(&DigitoD2, RESET);						//Se muestra el número en el dígito 2
			break;
		}case 3:{
			gpio_WritePin(&DigitoD2, SET);							//Se desactiva el digito anterior
			setNumeroSieteSegmentos(miles_contador_Taximetro);	//Se asigna el número correspondiente a los miles
			gpio_WritePin(&DigitoD3, RESET);						//Se muestra el número en el dígito 3
			break;
		}
	}
}

void FSM_update(State_t State){
	switch (State){
		case STATE_IDLE:{
			if(digito > 3){	//Si se quiere prender el digito 4 (No existe) se devuelve al digito 0
				digito = 0;
			}
			displaySieteSegmentos(digito);	//Se muestran los digitos con el efecto visual deseado
			break;
		}case STATE_RGB_FEEDBACK :{
			//Este condicional es para poder repetir el ciclo dentro del ENUM indefinidamente
			if (Current_Color == ROJO_VERDE_AZUL ){	//Si se hace una interrupción y el estado es el final
				Current_Color = APAGADO;			//Se vuelve al estado apagado
				cambioEstadoLEDRGB(Current_Color);	//Se muestra el estado apagado
				Current_State = STATE_IDLE;			//Se vuelve al estado IDLE
			}else{
				Current_Color++;					//De otra forma, se cambia al siguiente estado
				cambioEstadoLEDRGB(Current_Color);	//Se muestra el color del estado
				Current_State = STATE_IDLE;			//Se vuelve al estado IDLE
			}
			break;
		}case STATE_TAXIMETER_FEEDBACK :{
			//Primero se identifica el sentido de giro del encoder,
			//Sumando o restando en cada caso
			if (gpio_ReadPin(&gpio_DATAPin) == 0){
				contador_Taximetro++;
			}else{
				contador_Taximetro--;
			}
			//Luego se evita que el contador pase de 0 a 0xFFFF, pasándolo a 0xFFF, pues queremos simular una variable de 12 bits
			if(contador_Taximetro > 4096){
				contador_Taximetro = 4095;
			}else if(contador_Taximetro > 4095){ //También se debe desbordar de 0xFFF a 0 en lugar de seguir subiendo
				contador_Taximetro = 0;
			}
			separarContador();	//Se separan los miles, centenas, decenas, unidades del nuevo número en el contador para
								//Mostrarlos posteriormente
			Current_State = STATE_IDLE;//Se cambia al estado IDLE para que se muestre los números del contador
			break;
		}
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
