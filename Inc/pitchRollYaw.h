/*
 * pitchRollYaw.h
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#ifndef PITCHROLLYAW_H_
#define PITCHROLLYAW_H_

float CurrentPitchAngle(void);
float CurrentRollAngle(void);
float CurrentYawAngle(void);

void CalculatePitchRollYaw(void);
void CollectInitalSensorValues(void);

#endif /* PITCHROLLYAW_H_ */
