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
#include "sensorLogging.h"

static float accelX = 0.0;
static float accelY = 0.0;
static float accelZ = 0.0;
static float gyroX = 0.0;
static float gyroY = 0.0;
static float gyroZ = 0.0;

typedef struct
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float dt;

} SensorLogPacket;

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
static float deltaT = 0.0f;
void CalculatePitchRollYaw(void)
{
	/* Gather latest IMU data */
	ReadMPU0650();

	/* Get deltaT  since last calculation */
	deltaT = Elapsed_Ms_Since_Timer_Start(&PRY_Timer);
	Ms_Timer_Start(&PRY_Timer); // restart timer
	deltaT /= 1000; // convert to seconds

	/* Subtract out initial gyro values (calibrate) */
	gyroX -= envGyroX;
	gyroY -= envGyroY;
	gyroZ -= envGyroZ;

	/* Log data  */
	//HAL_Delay(20);
	log_mpu6050(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, deltaT);


//	if(PRY_Count < 10)
//		deltaT = 0.01;

	// x axis of gyro points to the right (as you look from behind quad)
	// y axis of gyro points straight forward (as you look from behind quad)
	float gyroRollDelta = 1.0 * gyroY * deltaT;
	float gyroPitchDelta = 1.0 * gyroZ * deltaT;
	float gyroYawDelta = 1.0 * gyroZ * deltaT;

	rollAngle += gyroRollDelta;
	pitchAngle += gyroPitchDelta;
	yawAngle += gyroYawDelta;

	static float lpfYawDelta = 0.0;
	lpfYawDelta = 0.3 * gyroYawDelta + (1 - 0.3) * lpfYawDelta;

	//If the IMU has yaw-ed transfer the roll angle to the pitch angle
	pitchAngle += rollAngle * sin(lpfYawDelta * (3.14/180.0));
	//If the IMU has yaw-ed transfer the pitch angle to the roll angle
	rollAngle -= pitchAngle * sin(lpfYawDelta * (3.14/180.0));


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
	rollAngle = 1.0 * rollAngle + 0.0 * accelRollAngle;
	pitchAngle = 1.0 * pitchAngle + 0.0 * accelPitchAngle;

	PRY_Count++;
}

float LastDeltaT(void)
{
	return deltaT;
}

void CollectInitalSensorValues(void)
{
	/* Throw out the first 100 samples */
	for(int i=0; i<100; i++)
	{
		// Throw out samples
		ReadAcceleration(&accelX, &accelY, &accelZ);
		ReadGyro(&gyroX, &gyroY, &gyroZ);
	}

	/* Average N samples */
	for(int i=0; i<400; i++)
	{
		// read acceleration, filter with a running average
		ReadAcceleration(&accelX, &accelY, &accelZ);
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
