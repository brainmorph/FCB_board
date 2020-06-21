/*
 * pitchRollYaw.c
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#include "pitchRollYaw.h"
#include "mpu6050.h"
#include "timer.h"
#include "math.h"
#include "main.h"


static float accelX = 0.0;
static float accelY = 0.0;
static float accelZ = 0.0;
static float gyroX = 0.0;
static float gyroY = 0.0;
static float gyroZ = 0.0;

/* Initial measurements of each MPU register */
//static float envAccelX=0.0, envAccelY=0.0, envAccelZ=0.0;
static float envGyroX=0.0, envGyroY=0.0, envGyroZ = 0.0;


static float pitchAngle = 0.0;
static float rollAngle = 0.0;
static float yawAngle = 0.0;


float CurrentPitchAngle(void)
{
	return pitchAngle;
}

float CurrentRollAngle(void)
{
	return rollAngle;
}

float CurrentYawAngle(void)
{
	return yawAngle;
}

static void ReadMPU0650(void)
{
	ReadAcceleration(&accelX, &accelY, &accelZ);
	ReadGyro(&gyroX, &gyroY, &gyroZ);
}

/* Calculate pitch, roll, and yaw angles from latest sensor data */
float lpfAx = 0.0; // low pass filter accelerometer but not gyro
float lpfAy = 0.0; // low pass filter accelerometer but not gyro
float lpfAz = 0.0; // low pass filter accelerometer but not gyro
static int PRY_Count = 0; // count how many times this loop has been executed
static uint32_t PRY_Timer = 0;
void CalculatePitchRollYaw(void)
{
	ReadMPU0650(); // get most recent sensor data

	// subtract out initial gyro values
	gyroX -= envGyroX;
	gyroY -= envGyroY;
	gyroZ -= envGyroZ;

	float deltaT = Elapsed_Ms_Since_Timer_Start(&PRY_Timer);
	deltaT /= 1000; // convert to units of Seconds

	Ms_Timer_Start(&PRY_Timer); // restart timer

//	if(PRY_Count < 10)
//		deltaT = 0.01;

	// x axis of gyro points from left to right (as you look from behind quad)
	// y axis of gyro points straight forward
	float gyroRollDelta = 1.0 * gyroY * deltaT;
	float gyroPitchDelta = 1.0 * gyroZ * deltaT;
	float gyroYawDelta = 1.0 * gyroZ * deltaT;

	rollAngle += gyroRollDelta;
	pitchAngle += gyroPitchDelta;
	yawAngle += gyroYawDelta;

	//If the IMU has yaw-ed transfer the roll angle to the pitch angle
	pitchAngle += rollAngle * sin(gyroYawDelta * (3.14/180.0));
	//If the IMU has yaw-ed transfer the pitch angle to the roll angle
	rollAngle += pitchAngle * sin(gyroYawDelta * (3.14/180.0));


	// average out acceleration but not gyro
	lpfAx = 0.5 * accelX + 0.5 * lpfAx;
	lpfAy = 0.5 * accelY + 0.5 * lpfAy;
	lpfAz = 0.5 * accelZ + 0.5 * lpfAz;

	// calculate roll angle from acceleration
//	float accelRollAngle = atan2f(lpfAy, lpfAz); // sign flip to align with accelerometer orientation
//	accelRollAngle *= (180.0 / 3.1415); // convert to degrees
//
//	//calculate pitch angle from acceleration
//	float accelPitchAngle = atan2f(-1.0*lpfAx, lpfAz);
//	accelPitchAngle *= (180.0 / 3.1415); // convert to degrees

	float accelRollAngle = atan2f(lpfAx, lpfAz);
	accelRollAngle *= (180.0 / 3.1415);

	float accelPitchAngle = atan2f(lpfAy, lpfAz);
	accelPitchAngle *= (180.0 / 3.1415);


	// complementary filter to correct drift error over time
	rollAngle = 0.99 * rollAngle + 0.01 * accelRollAngle;
	pitchAngle = 0.99 * pitchAngle + 0.01 * accelPitchAngle;

	PRY_Count++;
}


void CollectInitalSensorValues(void)
{
	// average some samples at the beginning
	for(int i=0; i<400; i++)
	{
		// read acceleration, filter with a running average
//		ReadAcceleration(&accelX, &accelY, &accelZ);
		ReadGyro(&gyroX, &gyroY, &gyroZ);

//		envAccelX += accelX / 400.0;
//		envAccelY += accelY / 400.0;
//		envAccelZ += accelZ / 400.0;
		envGyroX += gyroX / 400.0;
		envGyroY += gyroY / 400.0;
		envGyroZ += gyroZ / 400.0;

		HAL_Delay(10);
	}
}
