/*
 * logging.c
 *
 *  Created on: Jun 8, 2020
 *      Author: DC
 */

#include "logging.h"
#include "usart.h"

static int systemErrorCount = 0;

// Increase the total system error count
void log_incrementErrorCount()
{
	systemErrorCount++;
}

// Return the total system errors encountered so far
int log_totalErrorCount()
{
	return systemErrorCount;
}

