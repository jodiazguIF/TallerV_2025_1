/*
 * stm32f4xx_hal.h
 *
 *  Created on: Mayo 7, 2025
 *      Author: Jose A. Díaz
 * Este archivo contiene la información más básica del micro:
 * - Valores del reloj principal
 * - Distribución básica de la memoria (Descrito en la figura 14 de la hoja de datos del micro)
 * - Posiciones de memoria de los periféricos disponibles en el micro descrito en la tabla 1 (Memory Mab)
 * - Incluir los demás drivers de los periféricos
 * - Definiciones de las constantes más básicas
 *
 * NOTA: La definición del NVIC será realizada al momento de describir el uso de las interrupciones
 */


#ifndef STM32F4XX_HAL_H_
#define STM32F4XX_HAL_H_

#include <stdint.h>
#include <stddef.h>

#define HSI_CLOCK_SPEED			8000000		//Valor de la señal principal de reloj (HSI -> High Speed Internal)
#define HSE_CLOCK_SPEED			16000000	//Valor de la señal principal de reloj (HSE -> High Speed External)

//#define NOP ()	(_asm_("NOP"))
#define NOP()		asm("NOP")
#define __weak		__attribute__((weak))

/*Direcciones base de la memoria Flash y SRAM
 * Datasheet, Memory Map, Figura 14
 * Recordar: 1KByte = 1024 bytes
 */
#define	FLASH_BASE_ADDR		0x08000000U		//Esta es la memoria del programa: 512 KB
#define SRAM_BASE_ADDR		0x20000000U		//Esta es la memoria RAM, 128 KB

/* NOTA: Observar que existen unos registros específicos del Cortex M4 en la región 0xE0000000U
 * Los controladores de las interrupciones se encuentran allí, por ejemplo. Se verá a su debido tiempo...
 */

/*NOTA:
 * Ahora agregamos la dirección de memoria base para cada uno de los periféricos que posee el micro
 * En el datasheet del micro, figura 14 (Memory Map) se encuentrea el mapa de los buses:
 * - APB1 (Advance Peripherical Bus)
 * - APB2
 * - AHB1 (Advance High performance Bus)
 * - AHB2
 */
/*
 * Direcciones de los Bus periféricos AHBx y APBx
 */
#define APB1_BASE_ADDR		0x40000000U
#define APB2_BASE_ADDR		0x40010000U
#define AHB1_BASE_ADDR		0x40020000U
#define AHB2_BASE_ADDR		0x50000000U
/*
 * Ahora se debe hacer lo mismo pero para cada una de las posiciones de memoria de cada uno de
 * los periféricos descritos en la Tabla 1 del manual de referencia del micro.
 * Observe que en dicha tabla está, a su vez, dividida en cuatro segmentos, cada uno correspondiente a
 * APB1, APB2, AHB1, AHB2
 *
 * Comenzar de arriba hacia abajo como se muestra en la tabla. Inicia USB_OTG_FS (AHB2)
 */

/* Posiciones de memoria para periféricos del HB2 */
#define USB_OTG_FS_BASE_ADDR	(AHB2_BASEADDR + 0x0000U)

/*Posiciones de memoria para los periféricos del AHB1*/
#define DMA2_BASE_ADDR		(AHB1_BASE_ADDR + 0X6400U)
#define DMA1_BASE_ADDR		(AHB1_BASE_ADDR + 0X6000U)
#define FIR_BASE_ADDR		(AHB1_BASE_ADDR + 0x3C00U)
#define RCC_BASE_ADDR		(AHB1_BASE_ADDR + 0x3800U)
#define CRC_BASE_ADDR		(AHB1_BASE_ADDR + 0x3000U)
#define GPIOH_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00U)
#define GPIOE_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000U)
#define GPIOD_BASE_ADDR		(AHB1_BASE_ADDR + 0x0C00U)
#define GPIOC_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800U)
#define GPIOB_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400U)
#define GPIOA_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000U)

//Posiciones de memoria para periféricos del APB2

#define	SPI5_BASE_ADDR		(APB2_BASE_ADDR + 0x5000U)
#define TIM11_BASE_ADDR		(APB2_BASE_ADDR + 0x4800U)
#define TIM10_BASE_ADDR		(APB2_BASE_ADDR + 0x4400U)
#define TIM9_BASE_ADDR		(APB2_BASE_ADDR + 0x4000U)
#define EXTI_BASE_ADDR		(APB2_BASE_ADDR + 0x3C00U)
#define SYSCFG_BASE_ADDR	(APB2_BASE_ADDR + 0x3800U)
#define SPI4_BASE_ADDR		(APB2_BASE_ADDR + 0x3400U)
#define SPI1_BASE_ADDR		(APB2_BASE_ADDR + 0x3000U)
#define SDI0_BASE_ADDR		(APB2_BASE_ADDR + 0x2C00U)
#define ADC1_BASE_ADDR		(APB2_BASE_ADDR + 0x2000U)
#define USART6_BASE_ADDR	(APB2_BASE_ADDR + 0x1400U)
#define USART1_BASE_ADDR	(APB2_BASE_ADDR + 0x1000U)
#define TIM1_BASE_ADDR		(APB2_BASE_ADDR + 0x0000U)


//Posiciones de memoria para periféricos del APB1
#define PWR_BASE_ADDR		(APB1_BASE_ADDR + 0x7000U)
#define I2C3_BASE_ADDR		(APB1_BASE_ADDR + 0x5C00U)
#define I2C2_BASE_ADDR		(APB1_BASE_ADDR + 0x5800U)
#define I2C1_BASE_ADDR		(APB1_BASE_ADDR + 0x5400U)
#define USART2_BASE_ADDR	(APB1_BASE_ADDR + 0x4400U)
#define I2Sext_BASE_ADDR	(APB1_BASE_ADDR + 0x4000U)
#define SPI3_BASE_ADDR		(APB1_BASE_ADDR + 0x3C00U)
#define SPI2_BASE_ADDR		(APB1_BASE_ADDR + 0x3800U)
#define I2S2ext_BASE_ADDR	(APB1_BASE_ADDR + 0x3400U)
#define IWDG_BASE_ADDR		(APB1_BASE_ADDR + 0x3000U)
#define WWDG_BASE_ADDR		(APB1_BASE_ADDR + 0x2C00U)
#define RTC_BASE_ADDR		(APB1_BASE_ADDR + 0x2800U)
#define TIM5_BASE_ADDR		(APB1_BASE_ADDR + 0x0C00U)
#define TIM4_BASE_ADDR		(APB1_BASE_ADDR + 0x0800U)
#define TIM3_BASE_ADDR		(APB1_BASE_ADDR + 0x0400U)
#define TIM2_BASE_ADDR		(APB1_BASE_ADDR + 0x0000U)

//Macros Genéricos
#define DISABLE 		(0)
#define ENABLE			(1)
#define SET				ENABLE
#define CLEAR			DISABLE
#define RESET			DISABLE
#define	FLAG_SET		SET
#define	FLAG_RESET		RESET
#define I2C_WRITE		(0)
#define I2C_READ		(1)

/* +++=========== INICIO de la descripción de los elementos que componen el periférico =======+++ */
/* Definición de la estructura de datos que representa a cada uno de los registros que
 * componen el periférico RCC
 * Debido a los temas que se van a manehar en el curso, solo se deben definir los bits de los registros:
 * 6.3.1 (RCC_CR) hasta el 6.3.12 (RCC_APB2ENR), 6.3.17 (RCC_BDCR) y 6.3.18 (RCC_CSR)
 * NOTA: La posición de memoria (offset) debe encajar perfectamente con la posición de memoria indicada
 * en la hoja de datos del equipos. Observe que los elementos "reservedx" también están presentes allí
 */

typedef struct{
	volatile uint32_t 	CR;				//Clock Control Register					ADDR_OFFSET:	0x00
	volatile uint32_t 	PLLCFGR;		//PLL Configuration Register				ADDR_OFFSET:	0x04
	volatile uint32_t 	CFGR;			//Clock Configuration Register				ADDR_OFFSET:	0x08
	volatile uint32_t  	CIR;			//Clock Interrupt Register					ADDR_OFFSET:	0x0C
	volatile uint32_t	AHB1RSTR;		//AHB1 Peripherical Reset Register			ADDR_OFFSET:	0x10
	volatile uint32_t	AHB2RSTR;		//AHB2 Peripherical Reset Register			ADDR_OFFSET:	0x14
	volatile uint32_t	reserved0; 		//reserved									ADDR_OFFSET:	0x18
	volatile uint32_t	reserved1;		//reserved 									ADDR_OFFSET:	0x1C
	volatile uint32_t	APB1RSTR; 		//APB1 Peripherical Reset Register 			ADDR_OFFSET:	0x20
	volatile uint32_t	APB2RSTR;		//APB2 Peripherical Reset Register			ADDR_OFFSET:	0x24
	volatile uint32_t	reserved2; 		//reserved									ADDR_OFFSET:	0x28
	volatile uint32_t	reserved3; 		//reserved									ADDR_OFFSET:	0x2C
	volatile uint32_t 	AHB1ENR;		//AHB1 Peripherical Clock Enable Register	ADDR_OFFSET:	0x30
	volatile uint32_t	AHB2ENR;		//AHB2 Peripherical Clock Enable Register	ADDR_OFFSET:	0x34
	volatile uint32_t	reserved4; 		//reserved									ADDR_OFFSET:	0x38
	volatile uint32_t	reserved5; 		//reserved									ADDR_OFFSET:	0x3C
	volatile uint32_t	APB1ENR;		//APB1 Peripherical Clock Enable Register	ADDR_OFFSET:	0x40
	volatile uint32_t	APB2ENR;		//APB2 Peripherical Clock Enable Register	ADDR_OFFSET:	0x44
	volatile uint32_t	reserved6; 		//reserved									ADDR_OFFSET:	0x48
	volatile uint32_t	reserved7; 		//reserved									ADDR_OFFSET:	0x4C
	volatile uint32_t	AHB1LPENR;		//AHB1 Clock Enable Low Power Register		ADDR_OFFSET:	0x50
	volatile uint32_t	AHB2LPENR;		//AHB2 Clock Enable Low Power Register		ADDR_OFFSET:	0x54
	volatile uint32_t	reserved8; 		//reserved									ADDR_OFFSET:	0x58
	volatile uint32_t 	reserved9; 		//reserved									ADDR_OFFSET:	0x5C
	volatile uint32_t	APB1LPENR;		//APB1 Clock Enable Low Power Register		ADDR_OFFSET:	0x60
	volatile uint32_t	APB2LPENR;		//APB2 Clock Enable Low Power Register		ADDR_OFFSET:	0x64
	volatile uint32_t	reserved10; 	//reserved									ADDR_OFFSET:	0x68
	volatile uint32_t	reserved11; 	//reserved									ADDR_OFFSET:	0x6C
	volatile uint32_t	BDCR;			//Backup Domain Control Register			ADDR_OFFSET:	0x70
	volatile uint32_t	CSR; 			//Clock Control & Status Register			ADDR_OFFSET:	0x74
	volatile uint32_t	reserved12; 	//reserved									ADDR_OFFSET:	0x78
	volatile uint32_t	reserved13; 	//reserved									ADDR_OFFSET:	0x7C
	volatile uint32_t	SSCGR;			//Spread Spectrum Clock generation Reg		ADDR_OFFSET:	0x80
	volatile uint32_t	PLLI2SCFGR; 	//PLLI2S Configuration Register				ADDR_OFFSET:	0x84
	volatile uint32_t	reserved14;		//reserved									ADDR_OFFSET:	0x88
	volatile uint32_t	DCKCFGR;		//Dedicated Clocks Configuration Reg		ADDR_OFFSET:	0x8C
} RCC_RegDef_t;

/*
 *Hacemos como un "merge" en el cual se ubica la estructura RCC_RegDeft_t a apuntar a la posición de memoria
 *correspondiente, de forma que cada variable dentro de la estrucutra coincide con cada uno
 */
#define RCC				((RCC_RegDef_t *) RCC_BASE_ADDR)

/* Descripción bit a bit de cada uno de los registros del que componen al periférico RCC
 * 6.3.1 RCC_CR
 * 6.3.2 RCC_PLLCFGR
 * 6.3.3 RCC_CFGR
 * 6.3.4 RCC_CIR
 * 6.3.5 RCC_AHB1RSTR
 * 6.3.6 RCC_AHB2RSTR
 * 6.3.7 RCC_APB1RSTR
 * 6.3.8 RCC_APB2RSTR
 */

/* 6.3.9 RCC_AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define	RCC_AHB1ENR_GPIOCEN			2
#define	RCC_AHB1ENR_GPIODEN			3
#define	RCC_AHB1ENR_GPIOEEN			4
#define	RCC_AHB1ENR_GPIOHEN			7
#define	RCC_AHB1ENR_CRCEN			12
#define	RCC_AHB1ENR_DMA1EN			21
#define	RCC_AHB1ENR_DMA2EN			22

/* 6.3.10 RCC_AHB2ENR
 * 6.3.11 RCC_APB1ENR
 * 6.3.12 RCC_APB2ENR
 * 6.3.17 RCC_BDCR
 * 6.3.18 RCC_CSR
 */

/* ============ Fin de la descripción de los elementos que componen el periférico ============
 * Definición de la estructura de datos que representa a cada uno de los registros que
 * componen el periférico GPIO
 *
 * Debudo a que el periférico GPIOx es muy simple, no es muy necesario crear la descripción bit a bit
 * de cada uno de los registros que componen dicho periférico, pero si es necesario comprender qué
 * hace cada registro para poder cargar correctamente la configuración.
 */

typedef struct{
	volatile uint32_t MODER;		//Port Mode Register				ADDR_OFFSET:	0x00
	volatile uint32_t OTYPER; 		//Port Output Type Register			ADDR_OFFSET:	0x04
	volatile uint32_t OSPEEDR; 		//Port Output Speed Register		ADDR_OFFSET:	0x08
	volatile uint32_t PUPDR;		//Port pull-up/pull-down register	ADDR_OFFSET:	0x0C
	volatile uint32_t IDR;			//Port Input Data Register			ADDR_OFFSET:	0x10
	volatile uint32_t ODR;			//Port Output Data Register			ADDR_OFFSET:	0x14
	volatile uint32_t BSRR; 		//Port bit set/reset Register		ADDR_OFFSET:	0x18
	volatile uint32_t LCKR; 		//Port Configuration Lock Register	ADDR_OFFSET:	0x1C
	volatile uint32_t AFR[2];		//Port Alternate Function Registers	ADDR_OFFSET:	0x20 - 0x24
}	GPIO_TypeDef;

/* Creamos un objeto de la estrucutra definida y hacemos que quede ubicada excatamente sobre
 * la posición de memoria donde se encuentra en registro.
 * Debido a que son varios periféricos GPIOx, es necesario hacer la definición de cada uno
 *
 * Tener cuidado que cada elemento coincida con su respectiva dirección base.
 */
#define GPIOA		((GPIO_TypeDef *) GPIOA_BASE_ADDR)
#define GPIOB		((GPIO_TypeDef *) GPIOB_BASE_ADDR)
#define	GPIOC		((GPIO_TypeDef *) GPIOC_BASE_ADDR)
#define	GPIOD		((GPIO_TypeDef *) GPIOD_BASE_ADDR)
#define GPIOE		((GPIO_TypeDef *) GPIOE_BASE_ADDR)
#define GPIOH		((GPIO_TypeDef *) GPIOH_BASE_ADDR)

/*For testing assert parameters */
#define IS_GPIO_ALL_INSTANCE(GPIOx) (GPIOx == GPIOA) || \
									(GPIOx == GPIOB) || \
									(GPIOx == GPIOC) || \
									(GPIOx == GPIOD) || \
									(GPIOx == GPIOE) || \
									(GPIOx == GPIOH)

typedef struct{
	volatile uint32_t dummy;	//Dummy expample register	ADDR_OFFSET 0x00
} DUMMY_RegDef_t;

/* No es necesario hacer nada más en este archivo, para este primer proyecto
 *
 * Incluimos las librerías de cada periféricos
 */

#endif /* STM32F4XX_HAL_H_ */
