/*
 * motorControl.c
 *
 *  Created on: Jul 7, 2020
 *      Author: DC
 */

#include "motorControl.h"
#include "tim.h"


void mixPWM(float thrust, float roll, float pitch, float yaw)
{
	// TODO: move these multipliers into 3 separate PID loops, one for each control axis
	pitch *= 1.1; // make pitch a little bit stronger than roll since the battery packs lie on this axis
	//roll *= 1.0;
	yaw /= 8.0; // yaw needs to be cut back heavily

	float FR = thrust - yaw + pitch - roll;
	float FL = thrust + yaw + pitch + roll;
	float BR = thrust + yaw - pitch - roll;
	float BL = thrust - yaw - pitch + roll;

	setPWM(FL, FR, BR, BL);
}

// each setting represents motor throttle from 0 to 100%
float motor1Setting=0.0, motor2Setting=0.0, motor3Setting=0.0, motor4Setting=0.0;
void setPWM(float motor1, float motor2, float motor3, float motor4)
{

	/* Clip min motor output */
	if(motor1 < 5)
		motor1 = 0;
	if(motor2 < 5)
		motor2 = 0;
	if(motor3 < 5)
		motor3 = 0;
	if(motor4 < 5)
		motor4 = 0;

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

	// transition speed one step at a time
	if(motor1 > motor1Setting)
		motor1Setting += 1;
	if(motor2 > motor2Setting)
		motor2Setting += 1;
	if(motor3 > motor3Setting)
		motor3Setting += 1;
	if(motor4 > motor4Setting)
		motor4Setting += 1;

	if(motor1 < motor1Setting)
		motor1Setting -= 1;
	if(motor2 < motor2Setting)
		motor2Setting -= 1;
	if(motor3 < motor3Setting)
		motor3Setting -= 1;
	if(motor4 < motor4Setting)
		motor4Setting -= 1;


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

