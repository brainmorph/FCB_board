/*
 * mpu6050.c
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#include "mpu6050.h"
#include "i2c.h"

/* Static Function Declarations */
static uint8_t readMPUreg(uint8_t reg);
static void writeMPUreg(uint8_t reg, uint8_t value);
static void configMPUFilter();


void InitMPU(void)
{
	//read a register over I2C
	readMPUreg(0x75);
	readMPUreg(0x6B);
	writeMPUreg(0x6B, 0x00); // wake the IMU
	readMPUreg(0x6B);
	readMPUreg(0x6B);

	readMPUreg(0x1C); // read accel config register
	writeMPUreg(0x1C, 0x10); // configure fullscale for +-8 g
	readMPUreg(0x1C); // confirm

	readMPUreg(0x1B); // read gyro config register
	writeMPUreg(0x1B, 0x08); // configure fullscale for +- 500 degress/s
	readMPUreg(0x1B); // confirm

	configMPUFilter(); // apply filtering to IMU readings
}

/*
 * Read specified register from MPU6050 module.
 *
 * Input: register address
 *
 * Output: return 8bit register value
 */
static uint8_t readMPUreg(uint8_t reg)
{
	uint16_t deviceAddress = 0x68;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	uint8_t value = 0;
	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, &value, 1, 1000); //read from register
	return value;
}

/*
 * Write value to specified register
 *
 * Input: register address, value to write
 */
static void writeMPUreg(uint8_t reg, uint8_t value) // TODO: move to separate module
{
	uint16_t deviceAddress = 0x68;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	pData[1] = value; //value to write
	uint16_t Size = 2; //we need to send 2 bytes of data (check out mpu datasheet... write register operation is defined this way)
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, Size, 1000); //select register and write to it all in one
	if(status != HAL_OK)
	{
		// TODO: log error
	}
}

static void configMPUFilter()
{
	// Read register
	volatile int16_t config = 0;
	config = readMPUreg(0x1A);

	config &= 0xF8;
	config |= 0x0; // this is the value that goes into register

	writeMPUreg(0x1A, config);
}

void ReadAcceleration(float* floatX, float* floatY, float* floatZ)
{
	// Read x,y,z acceleration registers. TODO: guarantee that these are from same sample
	volatile int16_t accelX = 0;
	accelX = readMPUreg(0x3B); //read accel X MSB value
	accelX = accelX << 8;
	accelX |= (0x00FF) & readMPUreg(0x3C); //read accel X LSB value

	volatile int16_t accelY = 0;
	accelY = readMPUreg(0x3D);
	accelY = accelY << 8;
	accelY |= (0x00FF) & readMPUreg(0x3E); // LSB

	volatile int16_t accelZ = 0;
	accelZ = readMPUreg(0x3F);
	accelZ = accelZ << 8;
	accelZ |= (0x00FF) & readMPUreg(0x40); // LSB

//	logValues[logIndex].ax = accelX;
//	logValues[logIndex].ay = accelY;
//	logValues[logIndex].az = accelZ;

//	volatile int16_t temp = 0; // temperature
//	temp = readMPUreg(0x41); // MSB
//	temp = temp << 8;
//	temp |= (0x00FF) & readMPUreg(0x42); // LSB

//	*floatX = (float)accelX * (float)(1.0/16384.0); //multiply reading with Full Scale value
//	*floatY = (float)accelY * (float)(1.0/16384.0); //multiply reading with Full Scale value
//	*floatZ = (float)accelZ * (float)(1.0/16384.0); //multiply reading with Full Scale value
//	//*floatTemp = ((float)temp / 340.0) + 36.53;

	*floatX = (float)accelX * (8.0/32767.0); // FS=+-8g
	*floatY = (float)accelY * (8.0/32767.0);
	*floatZ = (float)accelZ * (8.0/32767.0);

	//convert from g to m/s^2
	*floatX *= 9.807;
	*floatY *= 9.807;
	*floatZ *= 9.807;
}



void ReadGyro(float* floatX, float* floatY, float* floatZ)
{
	// Read x,y,z acceleration registers. TODO: guarantee that these are from same sample
	volatile int16_t gyroX = 0;
	gyroX = readMPUreg(0x43); //read accel X MSB value
	gyroX = gyroX << 8;
	gyroX |= (0x00FF) & readMPUreg(0x44); //read gyro X LSB value

	volatile int16_t gyroY = 0;
	gyroY = readMPUreg(0x45);
	gyroY = gyroY << 8;
	gyroY |= (0x00FF) & readMPUreg(0x46); // LSB

	volatile int16_t gyroZ = 0;
	gyroZ = readMPUreg(0x47);
	gyroZ = gyroZ << 8;
	gyroZ |= (0x00FF) & readMPUreg(0x48); // LSB

//	logValues[logIndex].gx = gyroX;
//	logValues[logIndex].gy = gyroY;
//	logValues[logIndex].gz = gyroZ;

//	*floatX = (float)gyroX * (float)(1.0/131.0); //multiply reading with Full Scale value
//	*floatY = (float)gyroY * (float)(1.0/131.0); //multiply reading with Full Scale value
//	*floatZ = (float)gyroZ * (float)(1.0/131.0); //multiply reading with Full Scale value

	*floatX = (float)gyroX * (500.0/32767.0); // FS=+-500 deg/s
	*floatY = (float)gyroY * (500.0/32767.0);
	*floatZ = (float)gyroZ * (500.0/32767.0);

}
