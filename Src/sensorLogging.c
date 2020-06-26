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
	char emptyByte;		// 1 fill byte
	float ax, ay, az;	// 12 bytes
	float gx, gy, gz;	// 12 bytes
	float dt;			// 4 bytes
	uint32_t count;		// 4 bytes
} mpu6050_t;

static mpu6050_t mpuData = {'&', '&', '&', '#', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};  // load up the first 3 members with packet key identifier

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
	mpuData.count++;  // increment count to keep track of how many times log_mpu6050 was called

	/* Send UART */
	volatile int uartBufLength = sizeof(mpuData);
	uartBufLength = uartBufLength;

	uint8_t uartBuf[uartBufLength];
	memcpy(uartBuf, &mpuData, uartBufLength);

	/* Leave the first 3 bytes as ampersands and then modify the rest of the bytes */
	memcpy(&uartBuf[4], &mpuData.ax, 4);
	memcpy(&uartBuf[8], &mpuData.ay, 4);
	memcpy(&uartBuf[12], &mpuData.az, 4);
	memcpy(&uartBuf[16], &mpuData.gx, 4);
	memcpy(&uartBuf[20], &mpuData.gy, 4);
	memcpy(&uartBuf[24], &mpuData.gz, 4);
	memcpy(&uartBuf[28], &mpuData.dt, 4);
	memcpy(&uartBuf[32], &mpuData.count, 4);

	HAL_UART_Transmit(&huart6, uartBuf,
			uartBufLength, 10); // 10 ms timeout
}
