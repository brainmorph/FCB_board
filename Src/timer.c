/*
 * timer.c
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#include "timer.h"
#include "main.h"


/* Restart the millisecond timer */
void Ms_Timer_Start(uint32_t* timer)
{
	*timer = HAL_GetTick(); // take note of current ms tick count
}

/* Return milliseconds since timer was started */
uint32_t Elapsed_Ms_Since_Timer_Start(uint32_t* timer)
{
	return HAL_GetTick() - *timer;
}
