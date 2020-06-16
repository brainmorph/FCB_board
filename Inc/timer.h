/*
 * timer.h
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

void Ms_Timer_Start(uint32_t* timer);
uint32_t Elapsed_Ms_Since_Timer_Start(uint32_t* timer);

#endif /* TIMER_H_ */
