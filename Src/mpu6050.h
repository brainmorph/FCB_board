/*
 * mpu6050.h
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

void ReadAcceleration(float* floatX, float* floatY, float* floatZ);
void ReadGyro(float* floatX, float* floatY, float* floatZ);

void InitMPU(void);

#endif /* MPU6050_H */
