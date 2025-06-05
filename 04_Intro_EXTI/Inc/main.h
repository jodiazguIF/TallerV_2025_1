/*
 * main.h
 *
 *  Created on: Jun 2, 2025
 *      Author: jose
 */

#ifndef MAIN_H_
#define MAIN_H_

// Se definen los estados finitos que usaremos para la máquina de estados
typedef enum{
	STATE_REFRESH,
	STATE_RGB_FEEDBACK,
	STATE_TAXIMETER_FEEDBACK,
	STATE_CHANGE_REFRESH,
} State_t;
State_t Current_State = STATE_REFRESH; // Se fija el estado por defecto

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
uint8_t digito = 0;
//Se inicializa una variable que lleva el tiempo en ms que ha pasado desde el inicio
uint32_t contador_Tiempo = 0;

//Variables auxiliares para llevar los digitos que se deben mostrar en el taxímetro
uint8_t miles_contador_Taximetro 	= 0;
uint8_t centenas_contador_Taximetro = 0;
uint8_t decenas_contador_Taximetro 	= 0;
uint8_t unidades_contador_Taximetro = 0;


#endif /* MAIN_H_ */
