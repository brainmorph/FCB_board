/*
 * motorControl.c
 *
 *  Created on: Jul 7, 2020
 *      Author: DC
 */

#include "motorControl.h"
#include "tim.h"
#include "pitchRollyaw.h"
#include "altitude.h"
#include "logging.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>



StateData_t stateData = {0, 0.0, 0.0, 0.0, 0.0};


void mixPWM(float thrust, float roll, float pitch, float yaw)
{
	// TODO: move these multipliers into 3 separate PID loops, one for each control axis
	pitch *= 1.1; // make pitch a little bit stronger than roll since the battery packs lie on this axis
	//roll *= 1.0;
	yaw /= 1.5; // yaw needs to be cut back heavily

	float FR = thrust - yaw + pitch - roll;
	float FL = thrust + yaw + pitch + roll;
	float BR = thrust + yaw - pitch - roll;
	float BL = thrust - yaw - pitch + roll;

	static int arm = 0;
	if(thrust > 0.0)
	{
		arm = 1;
	}
	else
	{
		arm = 0;
	}

	setPWM(arm, FL, FR, BR, BL);
}

//static void changePWMincrementally(float* motorID, float* motorIDsetting)
//{
//	int stepSize = 5;
//	if(*motorID > *motorIDsetting)
//	{
//		if(*motorID - *motorIDsetting > stepSize)
//		{
//			*motorIDsetting += 1;
//		}
//		else
//		{
//			*motorIDsetting += stepSize;
//		}
//	}
//	else if(*motorID < *motorIDsetting)
//	{
//		if(*motorIDsetting - *motorID > stepSize)
//		{
//			*motorIDsetting -= 1;
//		}
//		else
//		{
//			*motorIDsetting -= stepSize;
//		}
//	}
//}

// each setting represents motor throttle from 0 to 100%
float motor1Setting=0.0, motor2Setting=0.0, motor3Setting=0.0, motor4Setting=0.0;
void setPWM(int arm, float motor1, float motor2, float motor3, float motor4)
{

	float motorMin = 8.0;

	/* Clip min motor output */
	if(motor1 < motorMin)
		motor1 = 0.0;
	if(motor2 < motorMin)
		motor2 = 0.0;
	if(motor3 < motorMin)
		motor3 = 0.0;
	if(motor4 < motorMin)
		motor4 = 0.0;

//	/* Prevent motors from turning completely off if quad is armed */
//	if(arm != 0)
//	{
//		if(motor1 < motorMin)
//			motor1 = motorMin;
//		if(motor2 < motorMin)
//			motor2 = motorMin;
//		if(motor3 < motorMin)
//			motor3 = motorMin;
//		if(motor4 < motorMin)
//			motor4 = motorMin;
//	}
//	else // if quad is un-armed, turn off all props
//	{
//		motor1 = 0;
//		motor2 = 0;
//		motor3 = 0;
//		motor4 = 0;
//	}

	/* Clip max motor output */
	float motorMax = 60;
	if(motor1 > motorMax)
		motor1 = motorMax;
	if(motor2 > motorMax)
		motor2 = motorMax;
	if(motor3 > motorMax)
		motor3 = motorMax;
	if(motor4 > motorMax)
		motor4 = motorMax;

	// transition PWM incrementally
//	changePWMincrementally(&motor1, &motor1Setting);
//	changePWMincrementally(&motor2, &motor2Setting);
//	changePWMincrementally(&motor3, &motor3Setting);
//	changePWMincrementally(&motor4, &motor4Setting);

	motor1Setting = motor1;
	motor2Setting = motor2;
	motor3Setting = motor3;
	motor4Setting = motor4;


	// TODO: update min and max values to match new timer settings (I want higher resolution control)
	// TODO: new timer will be 80MHz with pre-scalar of 20 and counter period 80,000
//		int max = 40; // based on 80MHz TIM4 with pre-scalar of 4000 and counter period 400
//		int min = 20; // based on 80MHz TIM4 with pre-scalar of 4000 and counter period 400

	int max = 4000; // 80MHz with pre-scalar 40 and counter period 40,000
	int min = 2000; // 80MHz with pre-scalar 40 and counter period 40,000

	int range = max - min;

	// motor 1
	int setting = (motor1Setting/100.0) * (float)range; // take a percentage out of max allowable range
	setting += min; // add new value to minimum setting
	htim2.Instance->CCR1 = setting;

	// motor 2
	setting = (motor2Setting/100.0) * (float)range; // take a percentage out of max allowable range
	setting += min;
	htim2.Instance->CCR2 = setting;

	// motor 3
	setting = (motor3Setting/100.0) * (float)range; // take a percentage out of max allowable range
	setting += min;
	htim2.Instance->CCR3 = setting;

	// motor 4
	setting = (motor4Setting/100.0) * (float)range; // take a percentage out of max allowable range
	setting += min;
	htim2.Instance->CCR4 = setting;

//		uint8_t uartData[100] = {0};
//		snprintf(uartData, sizeof(uartData), "[%02.2f, %02.2f, %02.2f, %02.2f]   ",
//				motor1, motor2, motor3, motor4);
//		HAL_UART_Transmit(&huart4, uartData, 100, 5);
}




void GatherSensorData()
{
	/* Gather all relevant sensor data */
	CalculatePitchRollYaw();

	stateData.altitude = CurrentAltitude();
	stateData.pitch = CurrentPitchAngle(); // ideally from -180 to 180
	stateData.roll = CurrentRollAngle(); // ideally from -180 to 180
	stateData.yaw = CurrentYawAngle(); // ideally from -180 to 180
	stateData.deltaT = LastDeltaT();
}

void CalculatePID(float throttleSet, float rollSet, float pitchSet, float yawSet, float kpOffset, float kdOffset)
{
	GatherSensorData();


	/* Calculate PID error terms */
	static float errorRoll, errorPitch, errorYaw;
	errorRoll = rollSet - stateData.roll;		// error roll is negative if quad will have to roll in negative direction
	errorPitch = pitchSet - stateData.pitch;	// error pitch is negative if quad will have to pitch in negative direction
	errorYaw = yawSet - stateData.yaw;			// error yaw is negative if quad will have to yaw in negative direction


	/* LPF the error terms to calculate derivative terms */
	static float lpfErrorRoll=0.0, lpfErrorPitch=0.0, lpfErrorRollOLD = 0.0, lpfErrorPitchOLD = 0.0;
	static float gamma = 0.5;
	lpfErrorRoll = gamma * lpfErrorRoll + (1 - gamma) * errorRoll;
	lpfErrorPitch = gamma * lpfErrorPitch + (1 - gamma) * errorPitch;

	/* Calculate derivative of error terms */
	float derivativeRollError = (lpfErrorRoll - lpfErrorRollOLD) / stateData.deltaT; // take derivative of lpf signal
	float derivativePitchError = (lpfErrorPitch - lpfErrorPitchOLD) / stateData.deltaT; // take derivative of lpf signal

	lpfErrorRollOLD = lpfErrorRoll; // update last measurement
	lpfErrorPitchOLD = lpfErrorPitch; // update last measurement

#undef UART_DEBUG
#ifdef UART_DEBUG
	snprintf(debugMessage, 100, "derivativeRoll= %f     derivativePitch = %f\r\n", derivativeRollError, derivativePitchError);
	HAL_UART_Transmit(&huart6, (uint8_t *)debugMessage, strlen(debugMessage), 10); // print success with 10 ms timeout
#endif



	//static float kp = 0.1;	// close to the last value used before drone crash
	//static float kd = 0.04;	// close to the last value used before drone crash
	static float kp = 0.0;
	static float kd = 0.0;

	/* Add offset from radio commands */
	static float totalKp, totalKd;
	totalKp = kp + kpOffset;
	totalKd = kd + kdOffset;

	/* Cap minimum allowable values */
	if(totalKp < 0.0)
		totalKp = 0;
	if(totalKd < 0.0)
		totalKd = 0;

	volatile static float rollCmd=0.0, pitchCmd=0.0, yawCmd=0.0;
	rollCmd = totalKp * errorRoll + totalKd * derivativeRollError; // negative roll command means roll in negative direction
	pitchCmd = totalKp * errorPitch + totalKd * derivativePitchError; // negative pitch command means pitch in negative direction
	yawCmd = totalKp * errorYaw; // WAS:  "+ (kd + kdoOffset) * derivativeYaw"	// negative yaw command means yaw in negative direction
	yawCmd = 0.0; // TURN OFF YAW TEMPORARILY

#undef UART_DEBUG
#ifdef UART_DEBUG
//	snprintf(debugMessage, 100, "kp = %f, kpOffset = %f     kd = %f, kdOffset = %f     ", kp, kpOffset, kd, kdOffset);
//	HAL_UART_Transmit(&huart6, (uint8_t *)debugMessage, strlen(debugMessage), 10); // print success with 10 ms timeout

	snprintf(debugMessage, 100, "rollCmd = %f     pitchCmd = %f \r\n", rollCmd, pitchCmd);
	HAL_UART_Transmit(&huart6, (uint8_t *)debugMessage, strlen(debugMessage), 10); // print success with 10 ms timeout
#endif


//#define EXPERIMENTAL
#ifdef EXPERIMENTAL
	/* Low Pass Filter the command signals */
	static float lpfRollCmd=0.0, lpfPitchCmd=0.0;
	float beta = 0.9;
	lpfRollCmd = beta * lpfRollCmd + (1 - beta) * rollCmd;
	lpfPitchCmd = beta * lpfPitchCmd + (1 - beta) * pitchCmd;

	/* Scale signal back up */
	float scaleFactor = 1.1;
	float scaledRollCmd = lpfRollCmd * scaleFactor;
	float scaledPitchCmd = lpfPitchCmd * scaleFactor;


	/* Low Pass Filter altitude signal */
	static float lpfAltitude=0.0;
	float altitudeFilter = 0.95;
	lpfAltitude = altitudeFilter * lpfAltitude + (1 - altitudeFilter) * stateData.altitude;
#endif

//#define UART_DEBUG
#ifdef UART_DEBUG
	char debugMessage[120];

//	snprintf(debugMessage, 100, "errorRoll = %f     errorPitch = %f     errorYaw = %f     ", errorRoll, errorPitch, errorYaw);
	snprintf(debugMessage, 120, "ok,%f,%f,%f,%f,%f,%f,%f \r\n",
			errorRoll, rollCmd, scaledRollCmd,
			errorPitch, pitchCmd, scaledPitchCmd,
			lpfAltitude);
	HAL_UART_Transmit(&huart6, (uint8_t *)debugMessage, strlen(debugMessage), 10);
#endif


	mixPWM(throttleSet, rollCmd, pitchCmd, yawCmd);
}

