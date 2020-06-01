/*
 * bme280.h
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#ifndef BME280_H_
#define BME280_H_

uint8_t bme280ReadReg(uint8_t reg);
void bme280ReadRegs(uint8_t reg, uint16_t size, uint8_t* data);

void bme280WriteReg(uint8_t reg, uint8_t value);

uint32_t bme280ReadPressure();
void bme280ReadAllRaw(int32_t *UT, int32_t *UP, int32_t *UH);
void BME280_Read_Calibration(void);
int32_t BME280_Pa_to_Alt(uint32_t P);

int32_t BME280_CalcT(int32_t UT);
uint32_t BME280_CalcP(int32_t UP);

// Compensation parameters structure
typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} BME280_Compensation_TypeDef;

// Compensation parameters storage
BME280_Compensation_TypeDef cal_param;

#endif /* BME280_H_ */
