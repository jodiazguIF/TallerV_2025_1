/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Jose Andrés Díaz Gutiérrez
 * @brief          : Breve introducción a la programación de MCUS
 ******************************************************************************
 *
 ******************************************************************************
 */


#include <stdint.h>
#include <stdio.h>

#define RCC_BASE_ADRESS 	0X40023800UL //Las dos letras al final significan unsigned long
#define RCC_AHB1ENR_OFFSET 	0x30
#define RCC_AHB1ENR 		(RCC_BASE_ADRESS + RCC_AHB1ENR_OFFSET) //Elemento en memoria, posición inicial + posición de memoria

#define GPIOA_BASE_ADRESS 		0x40020000 	//Posición de memoria donde está definido el puerto GPIOA
#define GPIOA_MODE_REG_OFFSET	0X00UL
#define GPIOA_ODR_REG_OFFSET	0X14UL		//Output Data Register
#define GPIOA_MODE_REG			(GPIOA_BASE_ADRESS + GPIOA_MODE_REG_OFFSET)
#define GPIOA_ODR_REG			(GPIOA_BASE_ADRESS + GPIOA_ODR_REG_OFFSET)



int main(void){
    uint32_t *registerAHB1enb = (uint32_t *)RCC_AHB1ENR; //Variable tipo puntero, Haciendo que la "variable puntero" registerAHB1enb apunte a donde se encuentra el registro
    /* Utilizando las operaciones bitwise, deseamos escribir 1 en la posición 0 del registro registerAHB1enb*/
    *registerAHB1enb &= ~(1 << 0);		//Se escribe un 0 en la posición 0 -> borrando lo que esté ahí
    *registerAHB1enb |= (1 << 0);		//Escribiendo un 1 en la posición 0 del registro RCC_AHB1ENR -> Activando la seañal de reloj

    uint32_t *registerGPIOA_MODE = (uint32_t *)GPIOA_MODE_REG; //Creando una variable que apunta al registro del GPIOA_MODE_REG
    *registerGPIOA_MODE &= ~(0b11 << 10); 	//Limpiando la posición que controla al pin 5 del GPIOA
    *registerGPIOA_MODE |= (0b01 << 10);  	//Activando al pin GPIOA_5 en modo de salida de propósito general

    uint32_t *registerGPIOA_ODR = (uint32_t *)GPIOA_ODR_REG; //Creando una variable que apunta al registro GPIOA_ODR_REG
    *registerGPIOA_ODR |= (1 << 5); //Se enciende el pin GPIOA_5 -> LED verde de la boarde debería encenderse

    /*Loop pa siempre*/
    while(1){

    }
    return 0 ;
}
