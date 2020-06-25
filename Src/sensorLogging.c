/*
 * sensorLogging.c
 *
 *  Created on: Jun 25, 2020
 *      Author: DC
 */

#include "sensorLogging.h"
#include "usart.h"
#include "main.h"
//#include <stdio.h>
#include <string.h>


typedef struct
{
	char key1;			// 1 byte packet identifier
	char key2;			// 1 byte packet identifier
	char key3;			// 1 byte packet identifier
	float ax, ay, az;	// 12 bytes
	float gx, gy, gz;	// 12 bytes
	float dt;			// 4 bytes
} mpu6050_t;

static mpu6050_t mpuData = {'&', '&', '&'};  // load up the first 3 members with packet key identifier

void log_mpu6050(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
	/* Reordering of bytes might be necessary in the future */
	mpuData.ax = ax;
	mpuData.ay = ay;
	mpuData.az = az;
	mpuData.gx = gx;
	mpuData.gy = gy;
	mpuData.gz = gz;
	mpuData.dt = dt;

	/* Send UART */
	volatile int uartBufLength = sizeof(mpuData); // 32 bytes because automatic structure alignment
	uartBufLength = uartBufLength;

	uint8_t uartBuf[uartBufLength];
	memcpy(uartBuf, &mpuData, uartBufLength);

	HAL_UART_Transmit(&huart6, uartBuf,
			uartBufLength, 10); // 10 ms timeout
}
