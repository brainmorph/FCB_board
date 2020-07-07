/*
 * motorControl.h
 *
 *  Created on: Jul 7, 2020
 *      Author: DC
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

void mixPWM(float thrust, float roll, float pitch, float yaw);
void setPWM(float motor1, float motor2, float motor3, float motor4);


#endif /* MOTORCONTROL_H_ */
