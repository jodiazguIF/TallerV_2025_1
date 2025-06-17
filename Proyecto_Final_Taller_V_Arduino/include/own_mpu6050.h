#ifndef MPU_APP_H
#define MPU_APP_H

#include <Arduino.h>
#include <MPU6050.h> // Ensure this file exists and the path/case matches exactly

extern MPU6050 mpu6050;
extern int16_t ax, ay, az, gx, gy, gz;

void initMPU6050();
void loopMPU6050();

#endif
