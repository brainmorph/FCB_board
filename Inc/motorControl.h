/*
 * motorControl.h
 *
 *  Created on: Jul 7, 2020
 *      Author: DC
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

typedef struct StateData_t
{
	float altitude; // 4 bytes

	float pitch; 	// 4 bytes
	float roll; 	// 4 bytes
	float yaw; 		// 4 bytes

	float errorPitch;	// 4 bytes
	float errorRoll;	// 4 bytes
	float errorYaw;		// 4 bytes

	float deltaT;	// 4 bytes

}StateData_t; // this packet MUST BE MAXIMUM 32 bytes in size


void mixPWM(float thrust, float roll, float pitch, float yaw);
void setPWM(int arm, float motor1, float motor2, float motor3, float motor4);

void CalculatePID(float throttleSet, float rollSet, float pitchSet, float yawSet, float kpOffset, float kdOffset);


#endif /* MOTORCONTROL_H_ */
