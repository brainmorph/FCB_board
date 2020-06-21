/*
 * mpu6050.c
 *
 *  Created on: Jun 16, 2020
 *      Author: DC
 */

#include "mpu6050.h"
#include "i2c.h"
#include "logging.h"

/* Static Function Declarations */
static uint8_t readMPUreg(uint8_t reg);
static void readMPUregs(uint8_t reg, uint16_t size, uint8_t* data);
static void writeMPUreg(uint8_t reg, uint8_t value);
//static void configMPUFilter();


void InitMPU(void)
{
	volatile uint8_t value = 0;

	//read a register over I2C
	value = readMPUreg(0x75);
	value = readMPUreg(0x6B);
	writeMPUreg(0x6B, 0x00); // wake the IMU
	readMPUreg(0x6B);
	readMPUreg(0x6B);

	readMPUreg(0x1C); // read accel config register
	writeMPUreg(0x1C, 0x10); // configure fullscale for +-8 g
	value = readMPUreg(0x1C); // confirm

	readMPUreg(0x1B); // read gyro config register
	writeMPUreg(0x1B, 0x08); // configure fullscale for +- 500 degress/s
	value = readMPUreg(0x1B); // confirm

	value = value;

//	configMPUFilter(); // apply filtering to IMU readings
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
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, 1, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
		log_incrementErrorCount();
	}

	uint8_t value = 0;
	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, &value, 1, 1000); //read from register
	return value;
}

/*
 * Read multiple registers from MPU6050 module.
 *
 * Input: register address
 *
 * Output: fill data buffer with register values
 */
static void readMPUregs(uint8_t reg, uint16_t size, uint8_t* data)
{
	uint16_t deviceAddress = 0x68;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, 1, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
		log_incrementErrorCount();
	}

	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, data, size, 1000); //read from register
	if(status != HAL_OK)
	{
		// TODO: log error
		log_incrementErrorCount();
	}
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

//static void configMPUFilter()
//{
//	// Read register
//	volatile int16_t config = 0;
//	config = readMPUreg(0x1A);
//
//	config &= 0xF8;
//	config |= 0x0; // this is the value that goes into register
//
//	writeMPUreg(0x1A, config);
//	volatile uint8_t test = readMPUreg(0x1A);
//	test = test;
//}

void ReadAcceleration(float* floatX, float* floatY, float* floatZ)
{
	volatile uint8_t test1 = readMPUreg(0x1C);
	test1 = test1;

	// Read x,y,z all acceleration in one go to guarantee they're from same sample
	uint8_t data[10] = {0};
	readMPUregs(0x3B, 8, data); // read consecutive bytes

	volatile int16_t aX = (data[0] << 8) | data[1];
	volatile int16_t aY = (data[2] << 8) | data[3];
	volatile int16_t aZ = (data[4] << 8) | data[5];

	volatile int16_t temp = (data[6] << 8) | data[7];

	volatile float temperature = ((float)temp / 340.0) + 36.53;
	temperature = temperature;


	*floatX = (float)aX * (8.0/32767.0); // FS=+-8g
	*floatY = (float)aY * (8.0/32767.0);
	*floatZ = (float)aZ * (8.0/32767.0);

	//convert from g to m/s^2
	*floatX *= 9.807;
	*floatY *= 9.807;
	*floatZ *= 9.807;
}



void ReadGyro(float* floatX, float* floatY, float* floatZ)
{
	// Read x,y,z all gyro in one go to guarantee they're from same sample
	uint8_t data[10] = {0};
	readMPUregs(0x43, 8, data); // read consecutive bytes

	volatile int16_t gX = (data[0] << 8) | data[1];
	volatile int16_t gY = (data[2] << 8) | data[3];
	volatile int16_t gZ = (data[4] << 8) | data[5];


	*floatX = (float)gX * (500.0/32767.0); // FS=+-500 deg/s
	*floatY = (float)gY * (500.0/32767.0);
	*floatZ = (float)gZ * (500.0/32767.0);

}
