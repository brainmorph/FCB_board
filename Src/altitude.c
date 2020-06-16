/*
 * altitude.c
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#include "altitude.h"
#include "bme280.h"


/* Static Variables */
static volatile float filteredAltitude = 0;
static float alpha = 0.1;


/* Function Definitions */
float CurrentAltitude(void)
{
    /* Compute altitude from BME280 pressure */
	volatile float altitude = getLastAltitude();

	/* Filter the altitude readings with weight */
	filteredAltitude = (alpha * altitude) + ((1-alpha) * filteredAltitude);

	return filteredAltitude;
}
