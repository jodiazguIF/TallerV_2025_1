//Includes para el proyecto
#include <Arduino.h>
#include "own_mpu6050.h"
#include "WiFi.h"
#include "frames.h"
#include "motor.h"


void setup() {
  Serial.begin(9600); //Iniciamos la comunicación serial
  initWiFi(); //Llamamos a la función que maneja la conexión WiFi y el UDP
  initMPU6050(); //Iniciamos el MPU6050
  init_displayframe(); // Iniciamos el marco de visualización
  initmotor();
}

void loop() {
  loopWiFi();     //Llamamos a la función que maneja la conexión WiFi y el UDP
  loopMPU6050();  // Llamamos a la función que maneja el MPU6050
  loopmotor();
} 
