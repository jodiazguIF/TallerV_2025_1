#include "own_mpu6050.h"
#include <MPU6050.h> 
#include "I2Cdev.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito

/*Definiciones*/
MPU6050 mpu6050;
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

void initMPU6050() {
  Wire.begin();           //Iniciando I2C  
  mpu6050.initialize();    //Iniciando el sensor

  if (mpu6050.testConnection()) Serial.println("Sensor MPU6050 iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loopMPU6050() {
  // Leer las aceleraciones y velocidades angulares
  mpu6050.getAcceleration(&ax, &ay, &az);
  mpu6050.getRotation(&gx, &gy, &gz);
  //Mostrar las lecturas separadas por un [tab]
  Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax); Serial.print("\t"); // "\t" es un tabulador
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
}