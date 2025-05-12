/*
 * gpio_driver_hal.c
 *
 *  Created on: May 8, 2025
 *      Author: Jose A. Diaz
 */

#include "gpio_driver_hal.h"
#include "stm32f4xx.h"
#include "stm32_assert.h"

/* === Headers for private functions === */
static void gpio_enable_clock_peripherical(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_mode(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_output_type(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_output_speed(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_pullup_pulldown(GPIO_Handler_t *pGPIOHandler);
static void gpio_config_alternate_function(GPIO_Handler_t *pGPIOHandler);

/*
 * Para cualquier periférico hay varios pasos que siempre se deben seguir en un
 * orden estricto para poder que el sistema permite configurar el periférico X.
 * Lo primero y más importante es activar la señal del reloj principal hacia ese
 * elemento específico (relacionado con el periférico RCC), a esto le llamaremos
 * simplemente "activar el periférico o activar la señal de reloj del periférico"
 */
void gpio_Config (GPIO_Handler_t *pGPIOHandler){
	/*Verificamos que el pin seleccionado es correcto */
	assert_param(IS_GPIO_PIN(pGPIOHandler -> pinConfig.GPIO_PinNumber));

	// 1. Activar el periférico
	gpio_enable_clock_peripherical(pGPIOHandler);

	//Después de activado, se puede empezar a configurar.
	//2. Configurando el registro GPIOx_MODER
	gpio_config_mode(pGPIOHandler);

	//3. Configurando el registro GPIOx_OTYPER
	gpio_config_output_type(pGPIOHandler);

	//4. Configurando la velocidad
	gpio_config_output_speed(pGPIOHandler);

	//5. Configurando si se desea pull-up, pull-down o flotante
	gpio_config_pullup_pulldown(pGPIOHandler);

	//6. Configurando las funciones alternativas... se verá luego en el curso
	gpio_config_alternate_function(pGPIOHandler);
} //Fin del GPIO_config

/* Enable Clock Signal for Specific GPIOx PORT */
void gpio_enable_clock_peripherical(GPIO_Handler_t *pGPIOHandler){

	//Verificamos para GPIOA
	if(pGPIOHandler->pGPIOx == GPIOA){
		// Escribimos 1 (SET)  en la posición correspondiente al GPIOA
		RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
	}
	else if(pGPIOHandler->pGPIOx == GPIOB){
		// Escribimos 1 (SET)  en la posición correspondiente al GPIOB
		RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
	}
	else if(pGPIOHandler->pGPIOx == GPIOC){
			// Escribimos 1 (SET)  en la posición correspondiente al GPIOC
			RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);
	}
	else if(pGPIOHandler->pGPIOx == GPIOD){
			// Escribimos 1 (SET)  en la posición correspondiente al GPIOD
			RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIODEN);
	}
	else if(pGPIOHandler->pGPIOx == GPIOE){
			// Escribimos 1 (SET)  en la posición correspondiente al GPIOE
			RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIOEEN);
	}
	else if(pGPIOHandler->pGPIOx == GPIOH){
			// Escribimos 1 (SET)  en la posición correspondiente al GPIOH
			RCC -> AHB1ENR |= (RCC_AHB1ENR_GPIOHEN);
	}

}

/*
 * Configures the mode in which the pin will work_
 * - Input
 * - Output
 * - Analog
 * - Alternate Function
 */
void gpio_config_mode(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxConfig = 0;

	/* Verificamos si el modo que se ha seleccionado es permitido */
	assert_param(IS_GPIO_MODE(pGPIOHandler ->pinConfig.GPIO_PinMode));

	/* Acá estamos leyendo la config, moviendo "PinNumber" veces hacia la izquierda ese valor (shift left)
	 * y tod0 eso lo cargamos en la variable auxConig
	 */
	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinMode << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);

	/* Antes de cargar el nuevo valor, limpiamos los bits específicos de ese registro (escribir 0b00)
	 * para lo cual se aplica una máscara y una operación bitwise AND
	 */
	pGPIOHandler->pGPIOx->MODER &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);

	// Cargamos a auxConfig en el registro MODER
	pGPIOHandler->pGPIOx->MODER |= auxConfig;
}

/*
 * Configura que tipo de output el PinX usará;
 * - Push-Pull
 * - openDrain
 */
void gpio_config_output_type(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxConfig = 0;

	/* Verificamos que el tipo de salida corresponda a los que se van a utilizar */
	assert_param(IS_GPIO_OUTPUT_TYPE(pGPIOHandler->pinConfig.GPIO_PinOutputType));

	//De nuevo, se lee y mueve el valor un número "PinNumber" de veces
	auxConfig = (pGPIOHandler ->pinConfig.GPIO_PinOutputType << pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Limpiamos antes de cargar
	pGPIOHandler->pGPIOx->OTYPER &= ~(SET << pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Se carga el resultado sobre el registro adecuado
	pGPIOHandler->pGPIOx->OTYPER |= auxConfig;
}

/*
 * Seleccionamos entre 4 posibles velocidades para la salida del PinX
 * - Low
 * - Medium
 * - Fast
 * - HighSpeed
 */

void gpio_config_output_speed(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxConfig = 0;

	assert_param(IS_GPIO_OSPEED(pGPIOHandler->pinConfig.GPIO_PinOutpuSpeed));

	auxConfig = (pGPIOHandler ->pinConfig.GPIO_PinOutputSpeed << 2*pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Limpiando la posición antes de cargar la nueva configuración
	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Cargamos el resultado sobre el registro adecuado
	pGPIOHandler->pGPIOx->OSPEEDR |= auxConfig;
}

/*
 * Turns ON/OFF the pull-up and pull-down resistor for each PinX in selected GPIO port
 */
void gpio_config_pullup_pulldown(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxConfig = 0;

	// Verificamos si la configuración cargada para las resistencias es correcta
	assert_param(IS_GPIO_PUPDR(pGPIOHandler->pinConfig.GPIO_PinPuPdControl));

	auxConfig = (pGPIOHandler->pinConfig.GPIO_PinPuPdControl << 2 *pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Limpiando la posición antes de cargar la nueva configuración
	pGPIOHandler->pGPIOx->PUPDR &= ~(0b11 << 2 * pGPIOHandler->pinConfig.GPIO_PinNumber);

	//Cargamos el resultado sobre el registro adecuado
	pGPIOHandler->pGPIOx->PUPDR |= auxConfig;
}
/*
 * Allows to configure other functions (more specialized) on the selected PinX
 */
void gpio_config_alternate_function(GPIO_Handler_t *pGPIOHandler){

	uint32_t auxPosition = 0;

	if(pGPIOHandler->pinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//Seleccionamos primero si se debe utilizar el registro bajo (AFRL) o el alto (AFRH)
		if(pGPIOHandler->pinConfig.GPIO_PinNumber<8){
			//Estamos en el registro AFRL, que controla los pines del PIN_0 al PIN_7
			auxPosition = 4 * pGPIOHandler->pinConfig.GPIO_PinNumber;

			//Limpiamos primero la posición del registro que deseamos escribir a continuación
			pGPIOHandler->pGPIOx->AFR[0] &= ~(0b1111 << auxPosition);

			//Y escribimos el valor conifgurando en la posición seleccionado
			pGPIOHandler->pGPIOx->AFR[0] |= (pGPIOHandler->pinConfig.GPIO_PinAltFunMode << auxPosition);
		}else{
			//Estamos en el registro AFRH, que controla los pines PIN 8 al PIN15
			auxPosition = 4 * (pGPIOHandler->pinConfig.GPIO_PinNumber -8 );

			//Limpiamos primero la posición del registro que deseamos escribir a continuación
			pGPIOHandler->pGPIOx->AFR[1] &= ~(0b1111 << auxPosition);

			//Y Escribimos el valor configurado en la posición seleccionada
			pGPIOHandler->pGPIOx->AFR[1] |= (pGPIOHandler->pinConfig.GPIO_PinAltFunMode << auxPosition);
		}
	}
}

/*
 * Función para leer el estado de un pin específico
 */

void gpio_WritePin(GPIO_Handler_t *pPinHandler, uint8_t newState){
	//Se verifica si la acción es permitida
	assert_param(IS_GPIO_PIN_ACTION(newState));

	//Se limpia la posición que se desea
	pPinHandler->pGPIOx->ODR &= ~(SET << pPinHandler->pinConfig.GPIO_PinNumber);
	if (newState == SET){
		//Trabajamos con la parte baja del registro
		pPinHandler->pGPIOx->BSRR |= (SET << pPinHandler->pinConfig.GPIO_PinNumber);
	}else{
		//Trabajando con la parte alta del registro
		pPinHandler->pGPIOx->BSRR |= (SET << (pPinHandler->pinConfig.GPIO_PinNumber+16));
	}
}

uint32_t gpio_ReadPin(GPIO_Handler_t *pPinHandler){
	//Creamos una variable auxiliar la cual retornaremos
	uint32_t pinValue = 0;

	//Cargamos el valor del registro IDR, desplazado a derecha tantas veces como la ubicación del pin específo
	pinValue = (pPinHandler->pGPIOx->IDR << pPinHandler->pinConfig.GPIO_PinNumber);

	return pinValue;
}

void gpio_TooglePin(GPIO_Handler_t *pPinHandler){
	uint16_t pinNumber = pPinHandler->pinConfig.GPIO_PinNumber;

	//Se lee el estado actual del pin
	if(pPinHandler->pGPIOx->ODR & (1 << pinNumber)){
		//Está en 1, se debe apagar (Escribir en la parte alta del BSRR)
		pPinHandler->pGPIOx->BSRR = (1 << (pinNumber+16));
	}else{
		//Está en 0, se debe encender (Escribir en la parte baja del BSRR)
		pPinHandler->pGPIOx->BSRR = (1 << pinNumber);
	}
}
