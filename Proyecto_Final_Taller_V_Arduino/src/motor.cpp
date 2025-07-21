#include <Arduino.h>
#include "motor.h"

#define BRAKE_PIN 7
#define DIR_PIN 8
#define PWM_PIN 9

void initmotor(void){
    //Primero definimos el modo de funcionamiento de los pines
    pinMode(BRAKE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    
    digitalWrite(BRAKE_PIN, HIGH); // Start line HIGH = motor activo
    digitalWrite(DIR_PIN, LOW);    // Dirección de rotación
    analogWrite(PWM_PIN, 200);     // Velocidad (prueba con 200–255)
}

void loopmotor(void){
    //Nothing
}