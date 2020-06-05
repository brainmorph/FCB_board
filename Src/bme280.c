/*
 * bme280.c
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#include <stdint.h>
#include "bme280.h"
#include "i2c.h"
#include "math.h"

#define DEVICE_ADDRESS 0x76; // use 0x76 even though data-sheet says 0x77

static int32_t t_fine;

void BMFC_BME280_Init(void)
{
  volatile uint8_t registerVal = bme280ReadReg(0xD0);
  // TODO: does a delay ever need to happen between successive I2C comms?
  bme280WriteReg(0xF4, 0x27); // wake the BME280 sensor and enable temperature and pressure

  int32_t tRaw = 0;
  int32_t pRaw = 0;
  int32_t hRaw = 0;

  BME280_Read_Calibration();

  HAL_Delay(100);

  //Do 1 altitude calculation as a check
  {
	  bme280ReadAllRaw(&tRaw, &pRaw, &hRaw);

	  volatile uint32_t temperature = BME280_CalcT(tRaw);
	  volatile uint32_t paPressure = BME280_CalcP(pRaw);
	  volatile float pascalFloat = ((float)paPressure)/256.0;
	  volatile float hpaPressure = pascalFloat / 100.0;
	  uint32_t dummy = pRaw;
	  float dummy2 = pascalFloat;


	  // Human-readable temperature, pressure and humidity value
	  volatile uint32_t pressure;
	  volatile uint32_t humidity;

	  // Human-readable altitude value
	  volatile float altitude = BME280_Altitude_Meters(hpaPressure);
	  volatile int32_t dummy99 = altitude;
  }
  //---------------------------------
}

void BMFC_BME280_TriggerAltitudeCalculation(void)
{
	int32_t tRaw = 0;
	int32_t pRaw = 0;
	int32_t hRaw = 0;

	bme280ReadAllRaw(&tRaw, &pRaw, &hRaw);

	volatile uint32_t temperature = BME280_CalcT(tRaw);
	volatile uint32_t paPressure = BME280_CalcP(pRaw);
	volatile float pascalFloat = ((float)paPressure)/256.0; // convert from Q24.8 format to pascal (decimal) value
	volatile float hpaPressure = pascalFloat / 100.0;
	uint32_t dummy = pRaw;
	float dummy2 = pascalFloat;


	// Human-readable temperature, pressure and humidity value
	volatile uint32_t pressure;
	volatile uint32_t humidity;

	// Human-readable altitude value
	volatile float altitude = BME280_Altitude_Meters(hpaPressure);
	volatile int32_t dummy99 = altitude;
}


uint8_t bme280ReadReg(uint8_t reg)
{
	uint16_t deviceAddress = DEVICE_ADDRESS;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[3] = {0};
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

void bme280ReadRegs(uint8_t reg, uint16_t size, uint8_t* data)
{
	uint16_t deviceAddress = DEVICE_ADDRESS;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[3] = {0};
	pData[0] = reg; //register in question
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, data, size, 1000); //read from register


}

void bme280WriteReg(uint8_t reg, uint8_t value)
{
	uint16_t deviceAddress = DEVICE_ADDRESS;
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

uint32_t bme280ReadPressure()
{
	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[3] = {0};
	pData[0] = 0xF7; // Register of MSB pressure value.  Plan is to read 2 more registers after this one for LSB and XSB
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, pData, 3, 1000); //read from selected register

	uint32_t pressure = (uint32_t)(pData[0] << 12 | pData[1] << 4 | pData[2] >> 4);
	return pressure;
}

void bme280ReadAllRaw(int32_t *UT, int32_t *UP, int32_t *UH)
{
	// Clear result values
	*UT = 0x80000;
	*UP = 0x80000;
	*UH = 0x8000;

	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[8] = {0};
	pData[0] = 0xF7; // Register of MSB pressure value.  Plan is to read 2 more registers after this one for LSB and XSB
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error.  Generate a non-blocking error flag that starts to get printed out
		return; // try again next time
	}

	Size = 6; // Since temperature and pressure are enabled only, and since they each need 3 bytes, we need to read a total of 6 bytes
	status = HAL_I2C_Master_Receive(&hi2c3, shiftedAddress, pData, Size, 1000);

	*UP = (int32_t)((pData[0] << 12) | (pData[1] << 4) | (pData[2] >> 4));
	*UT = (int32_t)((pData[3] << 12) | (pData[4] << 4) | (pData[5] >> 4));
	*UH = (int32_t)((pData[6] <<  8) |  pData[7]);

}

// Read calibration data
void BME280_Read_Calibration(void) {
	// Read pressure and temperature calibration data (calib00..calib23)
	bme280ReadRegs(0x88, 24, (uint8_t *)&cal_param);

	// Skip one byte (calib24) and read A1 (calib25)
	cal_param.dig_H1 = bme280ReadReg(0xA1);

	// Read humidity calibration data (calib26..calib41)
	uint8_t buf[7];
	bme280ReadRegs(0xE1, 7, buf);

	// Unpack data
	cal_param.dig_H2 = (int16_t)((((int8_t)buf[1]) << 8) | buf[0]);
	cal_param.dig_H3 = buf[2];
	cal_param.dig_H4 = (int16_t)((((int8_t)buf[3]) << 4) | (buf[4] & 0x0f));
	cal_param.dig_H5 = (int16_t)((((int8_t)buf[5]) << 4) | (buf[4]  >>  4));
	cal_param.dig_H6 = (int8_t)buf[6];

}

// Lifted from https://github.com/LonelyWolf/stm32/blob/master/bme280/bme280.c
// DATASHEET SUGGESTS TO LIFT THE FOLLOWING CONVERSION CODE STRAIGHT FROM BME280 DRIVER REPO
// Calculate pressure from raw value, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa as unsigned 32-bit integer in Q24.8 format (24 integer and 8 fractional bits)
// note: output value of '24674867' represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// note: BME280_CalcT must be called before calling this function
// note: using 64-bit calculations
// note: code from the BME280 datasheet (rev 1.1)
uint32_t BME280_CalcP(int32_t UP) {
	int32_t v1,v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)cal_param.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)cal_param.dig_P4) << 16);
	v1 = (((cal_param.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13 )) >> 3) + ((((int32_t)cal_param.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)cal_param.dig_P1)) >> 15;
	if (v1 == 0) return 0; // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576) - UP) - (v2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)v1);
	} else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)cal_param.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + cal_param.dig_P7) >> 4));

	// Convert pressure to Q24.8 format (fractional part always be .000)
	p <<= 8;

	return (uint32_t)p;
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
int32_t BME280_CalcT(int32_t UT) {
	t_fine  = ((((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1))) * ((int32_t)cal_param.dig_T2)) >> 11;
	t_fine += (((((UT >> 4) - ((int32_t)cal_param.dig_T1)) * ((UT >> 4) - ((int32_t)cal_param.dig_T1))) >> 12) * ((int32_t)cal_param.dig_T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
}

float lastAltitude = 0.0f;
float BME280_Altitude_Meters(float localhPa)
{
	volatile float P = localhPa / 1013.25;	// 1013.25 is sea level pressure in hPa
	volatile float subtract = pow(P, (1/5.255));
	volatile float multiply = 1 - subtract;
	volatile float altitude = multiply * 44330;

	lastAltitude = altitude;
	return altitude;
}

float getCurrentAltitude(void)
{
	return lastAltitude;
}

// Convert pressure in Pascals to altitude in millimeters via barometric formula
// input:
//   P - pressure in Pascals
// return: altitude in millimeters
int32_t BME280_Pa_to_Alt(uint32_t P) {
	// The value '101325.0' is 'standard' pressure at mean sea level (MSL) with temperature 15C

	// Hypsometric formula (for altitudes below 11km)
	// h = ((powf(P0 / P,1 / 5.257) - 1.0) * (T + 273.15)) / 0.0065

	// Original barometric formula
//	float alt_f = (44330.0 * (1.0 - powf(P / 101325.0,1 / 5.255))) * 1000;

	// Replace the powf() function with Taylor series expansion at point P = 101325
	// Using WolframAlpha to convert (44330.0 * (1.0 - powf(P / 101325.0,1 / 5.255)) into Taylor series
	// http://www.wolframalpha.com/input/?i=44330*%281+-+%28%28P%2F101325%29^%281%2F5.255%29%29%29+taylor+series+|+P%3D101325
	// The two terms of the series is enough for good result, take three of them to slightly improve precision
	//   -0.0832546 (P-101325)+3.32651�10^-7 (P-101325)^2-1.98043�10^-12 (P-101325)^3
	int32_t p1 = P - 101325; // Substitute for equation
	int32_t p2 = p1 * p1; // (P - 101325)^2
//	int32_t p3 = p1 * p2; // (P - 101325)^3
	// Calculate altitude centered at 'standard' sea level pressure (101325Pa)
//	float alt_t = ((-0.0832546 * p1) + (3.32651E-7 * p2) - (1.98043E-12 * p3)) * 1000.0;
	float alt_t = ((-0.0832546 * p1) + (3.32651E-7 * p2)) * 1000.0;

	// Taylor series with integers only (centered at 'standard' sea level pressure 101325Pa)
	int32_t alt_i = 0; // 0th term of series
	alt_i -= (p1 * 21313) >> 8; // 1th term Q24.8: 0.0832546 * 1000 -> (83 << 8) + (0.2546 * 256) -> 21313
	alt_i += (((p1 * p1) >> 8) * 22) >> 8; // 2nd term Q24.8: 3.32651E-7 * 1000 * 256 -> 22
	// It can be calculated by only shift and add:
	// ((p2 >> 8) * 22) >> 8 --> substitute (pp = p2 >> 8) --> (pp >> 4) + (pp >> 6) + (pp >> 7)
	// alt_i += (p2 >> 12) + (p2 >> 14) + (p2 >> 15); // 2nd term
	// ...but with same code size this will takes 3 more CPU clocks (STM32 hardware multiplier rocks =)

	// TODO: since the sensor operating range is 300 to 1100hPa, it is worth to center the Taylor series
	//       at other pressure value, something near of 90000Pa

/*
	printf("P: %uPa\t",P);
	printf("alt_f: %i.%03im\talt_t: %i.%03im\t",
			(int32_t)alt_f / 1000,
			(int32_t)alt_f % 1000,
			(int32_t)alt_t / 1000,
			(int32_t)alt_t % 1000
		);
	printf("diff_ft: %i.%03im\t",
			(int32_t)(alt_f - alt_t) / 1000,
			(int32_t)(alt_f - alt_t) % 1000
		);
	printf("diff_fi: %i.%03im\r\n",
			(int32_t)(alt_f - alt_i) / 1000,
			(int32_t)(alt_f - alt_i) % 1000
		);
*/

	return (int32_t)alt_t;

	// Calculation accuracy comparison: powf() vs Taylor series approximation
	//   P:      0Pa  alt_f: 44330.000m  alt_t:  8993.567m  diff: 35336.432m
	//   P:  10000Pa  alt_f: 15799.133m  alt_t:  7520.170m  diff:  8278.962m
	//   P:  40000Pa  alt_f:  7186.406m  alt_t:  4927.885m  diff:  2258.521m
	//   P:  50000Pa  alt_f:  5575.208m  alt_t:  3720.608m  diff:  1854.599m
	//   P:  60000Pa  alt_f:  4207.018m  alt_t:  4008.579m  diff:   198.438m
	//   P:  70000Pa  alt_f:  3012.615m  alt_t:  2934.363m  diff:    78.251m
	//   P:  80000Pa  alt_f:  1949.274m  alt_t:  1926.678m  diff:    22.595m
	//   P:  90000Pa  alt_f:   988.646m  alt_t:   985.524m  diff:     3.122m
	//   P: 100000Pa  alt_f:   110.901m  alt_t:   110.892m  diff:     0.009m
	//   P: 110000Pa  alt_f:  -698.421m  alt_t:  -697.199m  diff:    -1.221m
	//   P: 120000Pa  alt_f: -1450.201m  alt_t: -1438.769m  diff:   -11.432m
	// Thus higher/lower the sensor goes from centered value of 101325Pa the more error will give
	// a Taylor series calculation, but for altitudes in range of +/-2km from MSL this is more than enough

	// For altitudes which noticeably differs from the MSL, the Taylor series can be centered at appropriate value:
	// http://www.wolframalpha.com/input/?i=44330*%281+-+%28%28P%2F101325%29^%281%2F5.255%29%29%29+taylor+series+|+P%3DXXXXXX
	// where XXXXXX - pressure at centered altitude
}
