/*
 * exti_driver_hal.c
 *
 *  Created on: May 20, 2025
 *      Author: Jose A. Diaz
 */

#include "exti_driver_hal.h"
#include "gpio_driver_hal.h"

/* === Headers for private functions === */
static void exti_enable_clock_peripheral(void);
static void exti_assign_channel(EXTI_Handler_t *extiConfig);
static void exti_select_edge(EXTI_Handler_t *extiConfig);
static void exti_config_interrupt(EXTI_Handler_t *extiConfig);

/*
 * Funcion de configuracion del sistema EXTI.
 * Requiere que un pinX ya se encuentre configurado como
 * entrada digital
 * */
void exti_Config(EXTI_Handler_t *extiConfig){

	/* 1.0 Se carga la configuración, que debe ser el PINx como entrada "simple" */
	gpio_Config(extiConfig->pGPIOHandler);

	/* 2.0 Activamos el acceso al SYSCFG */
	exti_enable_clock_peripheral();

	/* 3.0 Seleccion de canal */
	exti_assign_channel(extiConfig);

	/* 4.0 Seleccionamos el tipo de flanco */
	exti_select_edge(extiConfig);

	/* 5.0 Desactivo primero las interrupciones globales */
	/*Agregue su código acá*/

	/* 6. 0 Manejo de Interrupciones */
	exti_config_interrupt(extiConfig);

	/* 7.0 Volvemos a activar las interrupciones globales */
	/*Agregue su código acá*/
}

/*
 * No requiere el periferico, ya que solo es necesario activar
 * al SYSCFG
 * */
static void exti_enable_clock_peripheral(void){
	/* 2.0 Activamos el acceso al SYSCFG */
	RCC-> APB2ENR |= (RCC_APB2ENR_SYSCFGEN);//Se pone un 1 en el registo que controla el clock para el SYSCFG
}

/*
 * Funcion que configura los MUX para asignar el pinX del puerto Y
 * a la entrada EXTI correspondiente.
 * */
static void exti_assign_channel(EXTI_Handler_t *extiConfig){
	/*Asignamos el canal EXTI que corresponde al PIN_y del puerto GPIO_X
		 * Debemos activar la línea PIN_Xy (Y = A, B, C... y x = 0, 1, 2, 3...)
		 * en el módulo EXTI */
		switch (extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber) {
		/* Configurando para el todos los pines GPIOX_0*/
		case 0: {
			/* SYSCFG_EXTICR1 */
			// Limpiamos primero la posición que deseamos configurar
			SYSCFG->EXTICR[0] &= ~(0xF << SYSCFG_EXTICR1_EXTI0_Pos);

			// Ahora seleccionamos el valor a cargar en la posición, segun sea la selección
			// del puerto que vamos a utilizar: GPIOA_0, ó GPIOB_0, ó GPIOC_0, etc
			if (extiConfig->pGPIOHandler->pGPIOx == GPIOA) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PA);

			} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PB);

			} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PC);

			} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PD);

			} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PE);

			} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
				SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PH);

			} else {
				__NOP();
			}

			break;
		}

		/* Configurando para el todos los pines GPIOX_1*/
	case 1: {
		/* SYSCFG_EXTICR1 */
        // Se limpia la posición que se desea configurar
		SYSCFG->EXTICR[0] &= ~(0xF << SYSCFG_EXTICR1_EXTI1_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR1_EXTI1_Pos = desfase necesario para acceder al registro de los pines Y1
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PA);	//Carga la configuración del puerto A al registro del pin 1, ahora se debe repetir con cada puerto Y
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PH);

		} else {
			__NOP();
		}

		break;
	}



	/* Configurando para el todos los pines GPIOX_2*/
	case 2: {
		/* SYSCFG_EXTICR1 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[0] &= ~(0xF << SYSCFG_EXTICR1_EXTI2_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR1_EXTI2_Pos = desfase necesario para acceder al registro de los pines Y2
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI2_PH);

		} else {
			__NOP();
		}

		break;
	}

	case 3:{
		/* SYSCFG_EXTICR1 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[0] &= ~(0xF << SYSCFG_EXTICR1_EXTI3_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR1_EXTI3_Pos = desfase necesario para acceder al registro de los pines Y3
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PH);

		} else {
			__NOP();
		}

		break;
	}

	case 4:{
		//Desde el puerto 4 al 7 pertenecen al EXTICR[1], también expresado como EXTICR2
		/* SYSCFG_EXTICR2 */
		SYSCFG->EXTICR[1] &= ~(0xF << SYSCFG_EXTICR2_EXTI4_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR2_EXTI4_Pos = desfase necesario para acceder al registro de los pines Y4
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PA);	//Carga la configuración del puerto A al registro del pin 1, ahora se debe repetir con cada puerto Y
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 5:{
		/* SYSCFG_EXTICR2 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[1] &= ~(0xF << SYSCFG_EXTICR2_EXTI5_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR2_EXTI5_Pos = desfase necesario para acceder al registro de los pines Y5
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 6:{
		/* SYSCFG_EXTICR2 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[1] &= ~(0xF << SYSCFG_EXTICR2_EXTI6_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR2_EXTI6_Pos = desfase necesario para acceder al registro de los pines Y6
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PH);

		} else {
			__NOP();
		}

		break;
	}

	case 7:{
		/* SYSCFG_EXTICR2 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[1] &= ~(0xF << SYSCFG_EXTICR2_EXTI7_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR2_EXTI7_Pos = desfase necesario para acceder al registro de los pines Y7
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI7_PH);

		} else {
			__NOP();
		}

		break;
	}

	case 8:{
		/* SYSCFG_EXTICR3, se cambia de registro nuevamente para los pines 8 - 11, también se cambia EXTICR[1] por EXTICR[2]*/
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[2] &= ~(0xF << SYSCFG_EXTICR3_EXTI8_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR3_EXTI8_Pos = desfase necesario para acceder al registro de los pines Y8
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI8_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 9:{
		//Se continua con la secuencia :D
		/* SYSCFG_EXTICR3 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[2] &= ~(0xF << SYSCFG_EXTICR3_EXTI9_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR3_EXTI9_Pos = desfase necesario para acceder al registro de los pines Y9
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI9_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 10:{
		/* SYSCFG_EXTICR3 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[2] &= ~(0xF << SYSCFG_EXTICR3_EXTI10_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR3_EXTI10_Pos = desfase necesario para acceder al registro de los pines Y10
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI10_PH);

		} else {
			__NOP();
		}

		break;
	}

	case 11:{
		/* SYSCFG_EXTICR3 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[2] &= ~(0xF << SYSCFG_EXTICR3_EXTI11_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR3_EXTI11_Pos = desfase necesario para acceder al registro de los pines Y11
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[2] |= (SYSCFG_EXTICR3_EXTI11_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 12:{
		/* SYSCFG_EXTICR4, se cambia de registro nuevamente para los pines 12 - 15, también se cambia EXTICR[2] por EXTICR[3]*/
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI12_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR4_EXTI12_Pos = desfase necesario para acceder al registro de los pines Y12
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI12_PH);

		} else {
			__NOP();
		}

		break;
	}
	case 13:{
		//Continuamos como veníamos :D
		/* SYSCFG_EXTICR4 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI13_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR4_EXTI13_Pos = desfase necesario para acceder al registro de los pines Y13
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI13_PH);

		} else {
			__NOP();
		}

		break;

	}
	case 14:{
		//Continuamos como veníamos :D
		/* SYSCFG_EXTICR4 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI14_Pos); //0xF = 0b1111, máscara de 4 bits
		//SYSCFG_EXTICR4_EXTI14_Pos = desfase necesario para acceder al registro de los pines Y14
		if(extiConfig->pGPIOHandler->pGPIOx == GPIOA){
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PA);
		}else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI14_PH);

		} else {
			__NOP();
		}

		break;

	}

	/* Configurando para el todos los pines GPIOX_15 */
	case 15: {
		/* SYSCFG_EXTICR4 */
		// Limpiamos primero la posición que deseamos configurar
		SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI15_Pos);

		// Ahora seleccionamos el valor a cargar en la posición, segun sea la selección
		// del puerto que vamos a utilizar: GPIOA_0, ó GPIOB_0, ó GPIOC_0, etc
		if (extiConfig->pGPIOHandler->pGPIOx == GPIOA) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PA);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOB) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PB);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOC) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PC);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOD) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PD);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOE) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PE);

		} else if (extiConfig->pGPIOHandler->pGPIOx == GPIOH) {
			SYSCFG->EXTICR[3] |= (SYSCFG_EXTICR4_EXTI15_PH);

		} else {
			__NOP();
		}
		break;
	}

	default: {
		__NOP();
		break;
	}

	}// Fin del switch-case
}


/*
 * Funcion para seleccionar adecuadamente el flanco que lanza la interrupcion
 * en el canal EXTI especifico.
 * */
static void exti_select_edge(EXTI_Handler_t *extiConfig){

	if(extiConfig->edgeType == EXTERNAL_INTERRUPT_FALLING_EDGE){
		/* Falling Trigger selection register*/
		//Se deben configurar cada uno de los puertos x (0-15) según la configuración del pin seleccionado para el EXTI}
		//El EXTI x solo se puede conectar a un GPIOY1 a la vez, por ejemplo, puede estar conectado al PA1, PA2 a la vez, pero no al PA1, PB1 a la vez.
		if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_0){
			//Se limpia la configuración que haya en el pin
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR0_Pos);
			//Se activa la interrupción por flanco de bajada asignando un 1 a la posición de memoria anterior
			EXTI->FTSR |= (EXTI_FTSR_TR0);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_1){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR1_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR1);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_2){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR2_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR2);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_3){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR3_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR3);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_4){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR4_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR4);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_5){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR5_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR5);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_6){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR6_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR6);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_7){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR7_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR7);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_8){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR8_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR8);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_9){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR9_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR9);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_10){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR10_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR10);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_11){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR11_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR11);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_12){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR12_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR12);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_13){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR13_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR13);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_14){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR14_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR14);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_15){
			EXTI->FTSR &= ~(1 << EXTI_FTSR_TR15_Pos);
			EXTI->FTSR |= (EXTI_FTSR_TR15);
		}
		//Se han configurado los posibles casos de los pines 1-15 de falling edge
	}
	else if(extiConfig->edgeType == EXTERNAL_INTERRUPT_RISING_EDGE){
		if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_0){
			//Se limpia la configuración que haya en el pin
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR0_Pos);
			//Se activa la interrupción por flanco de bajada asignando un 1 a la posición de memoria anterior
			EXTI->RTSR |= (EXTI_RTSR_TR0);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_1){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR1_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR1);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_2){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR2_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR2);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_3){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR3_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR3);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_4){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR4_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR4);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_5){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR5_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR5);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_6){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR6_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR6);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_7){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR7_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR7);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_8){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR8_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR8);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_9){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR9_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR9);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_10){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR10_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR10);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_11){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR11_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR11);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_12){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR12_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR12);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_13){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR13_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR13);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_14){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR14_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR14);
		}else if(extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber == PIN_15){
			EXTI->RTSR &= ~(1 << EXTI_RTSR_TR15_Pos);
			EXTI->RTSR |= (EXTI_RTSR_TR15);
		}
		//Se han configurado los pines para el trigger con rising edge

	}
	/*
	else if(extiConfig->edgeType == EXTERNAL_INTERRUPT_BOTH){	//Esta cosa la intentaré plantear luego

	}*/
}

/*
 * Funcion que configura las mascaras de interrupciones (registro de mascaras) y
 * ademas matricula cada una de las posibles interrupciones en el NVIC
 * */
static void exti_config_interrupt(EXTI_Handler_t *extiConfig){
	/* 6.0 Activamos la interrupción del canal que estamos configurando */
	// Interrupt Mask register
	//Creamos una variable que sea el valor del pin
	uint32_t pinNumber = extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber;
	// Se limpia el bit correspondiente al EXTIx
	EXTI->IMR &= ~(1 << pinNumber);
	// Se activa la IMR correspondiente al EXTIx
	EXTI->IMR |= (1 << pinNumber);

	/* 6.1 Matriculamos la interrupción en el NVIC para el canal correspondiente,
	 * donde el canal 0 corresponde al EXTI_0, canal 1 al EXTI_1, etc.
	 *
	 * NOTA: Observar que algunos canales EXTI comparten un mismo vector de interrupción
	 * */
	switch (extiConfig->pGPIOHandler->pinConfig.GPIO_PinNumber) {
	case 0: {
		__NVIC_EnableIRQ(EXTI0_IRQn);
		break;
	}case 1: {
		__NVIC_EnableIRQ(EXTI1_IRQn);
		break;
	}case 2: {
		__NVIC_EnableIRQ(EXTI2_IRQn);
		break;
	}case 3: {
		__NVIC_EnableIRQ(EXTI3_IRQn);
		break;
	}case 4: {
		__NVIC_EnableIRQ(EXTI4_IRQn);
		break;
	}case 5: {
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	}case 6: {
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	}case 7: {
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	}case 8: {
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	}case 9: {
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	}case 10: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}case 11: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}case 12: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}case 13: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}case 14: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}
	case 15: {
		__NVIC_EnableIRQ(EXTI15_10_IRQn);
		break;
	}
	default: {
		break;
	}

	}
}

/**/
__attribute__((weak)) void callback_ExtInt0(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt1(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt2(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt3(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt4(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt5(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt6(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt7(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt8(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt9(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt10(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt11(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt12(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt13(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt14(void) {
	__NOP();
}

__attribute__((weak)) void callback_ExtInt15(void) {
	__NOP();
}

/*
 * Todos los callbacks del 0 al 15 han sido agregados para un total de 16 callbacks
 */


/* ISR de la interrupción canal 0*/
void EXTI0_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_0 del GPIO_X
	if(EXTI->PR & EXTI_PR_PR0){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR0;

		// llamamos al callback
		callback_ExtInt0();
	}
}
void EXTI1_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_1 del GPIO_X
	if(EXTI->PR & EXTI_PR_PR1){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR1;

		// llamamos al callback
		callback_ExtInt1();
	}
}
void EXTI2_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_2 del GPIO_X
	if(EXTI->PR & EXTI_PR_PR2){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR2;

		// llamamos al callback
		callback_ExtInt2();
	}
}
void EXTI3_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_3 del GPIO_X
	if(EXTI->PR & EXTI_PR_PR3){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR3;

		// llamamos al callback
		callback_ExtInt3();
	}
}
void EXTI4_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_4 del GPIO_X
	if(EXTI->PR & EXTI_PR_PR4){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR4;

		// llamamos al callback
		callback_ExtInt4();
	}
}

/*
 * Se agregan las interrupcion EXTI pendientes con el if correspondiente
 * para verificar que se está atendiendo
 */


/* ISR de la interrupción canales 9_5
 * Observe que debe agregar totos los posibles casos, los cuales
 * son identificados por un bloque if() y el analisis de la bandera
 * (pending register -> EXTI_PR)
 */
void EXTI9_5_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR5){
    	// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR5;

		// llamamos al callback
		callback_ExtInt5();
    }else if(EXTI->PR & EXTI_PR_PR6){
    	// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR6;

		// llamamos al callback
		callback_ExtInt6();
    }else if(EXTI->PR & EXTI_PR_PR7){
    	// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR7;

		// llamamos al callback
		callback_ExtInt7();
    }else if(EXTI->PR & EXTI_PR_PR8){
    	// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR8;

		// llamamos al callback
		callback_ExtInt8();
    }else if(EXTI->PR & EXTI_PR_PR9){
    	// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR9;

		// llamamos al callback
		callback_ExtInt9();
    }
}

/* ISR de la interrupción canales 15_10
 * Observe que debe agregar totos los posibles casos, los cuales
 * son identificados por un bloque if() y el analisis de la bandera
 * (pending register -> EXTI_PR)
 */
void EXTI15_10_IRQHandler(void){
	// Evaluamos si la interrupción que se lanzo corresponde al PIN_Y_15
	if(EXTI->PR & EXTI_PR_PR10){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR10;

		// llamamos al callback
		callback_ExtInt10();
	}else if(EXTI->PR & EXTI_PR_PR11){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR11;

		// llamamos al callback
		callback_ExtInt11();
	}else if(EXTI->PR & EXTI_PR_PR12){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR12;

		// llamamos al callback
		callback_ExtInt12();
	}else if(EXTI->PR & EXTI_PR_PR13){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR13;

		// llamamos al callback
		callback_ExtInt13();
	}else if(EXTI->PR & EXTI_PR_PR14){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR14;

		// llamamos al callback
		callback_ExtInt14();
	}else if(EXTI->PR & EXTI_PR_PR15){
		// Bajamos la bandera correspondiente
		EXTI->PR |= EXTI_PR_PR15;

		// llamamos al callback
		callback_ExtInt15();
	}
}
