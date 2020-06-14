/*
 * flightControl.c
 *
 *  Created on: Jun 9, 2020
 *      Author: DC
 */

#include "flightcontrol.h"
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "MY_NRF24.h"
#include "bme280.h"
#include "logging.h"
#include "transceiver.h"

typedef struct RadioPacket_t
{
	uint32_t count; // 4 bytes
	float altitude; // 4 bytes

	/* Fill in the rest of the struct to make it be 32 bytes in total */
	uint32_t garbage1; // 4 bytes
	uint32_t garbage2; // 4 bytes
	uint32_t garbage3; // 4 bytes
	uint32_t garbage4; // 4 bytes
	uint32_t garbage5; // 4 bytes
	uint32_t garbage6; // 4 bytes
}RadioPacket_t; // this packet MUST BE 32 bytes in size

RadioPacket_t telemetryData = {0, 0.0f};
RadioPacket_t groundData = {0, 0.0f};

typedef struct
{
	volatile int preDecimal;		// signed 32 bit int
	volatile int postDecimal;		// signed 32 bit int
}AltimeterData_t;

AltimeterData_t altimeter = {0, 0};

//most basic NRF transmit mode (NO ACK, just transmit)
uint64_t TxRxpipeAddrs = 0x11223344AA;
char myTxData[32] = "Hello goobers";
char myRxData[50];
volatile float filteredAltitude = 0;
volatile float alpha = 0.1;



void FC_Init(void)
{
	BMFC_BME280_Init(); // Initialize the BME280 sensor

	NRF24_begin(GPIOB, SPI1_CS_Pin, SPI1_CE_Pin, hspi1);
	nrf24_DebugUART_Init(huart6);
	printRadioSettings();
	printConfigReg();


#define TX_SETTINGS // configure this build for NRF24L01 Transmitter Mode
#define RX_SETTINGS // configure this build for NRF24L01 Receiver Mode

	NRF24_stopListening();   // just in case

#ifdef TX_SETTINGS
	NRF24_openWritingPipe(TxRxpipeAddrs);
#endif
#ifdef RX_SETTINGS
	NRF24_openReadingPipe(1, TxRxpipeAddrs);
#endif
	NRF24_setAutoAck(false); // disable ack
	NRF24_setChannel(52);    // choose a channel (why 52?)
	NRF24_setPayloadSize(32);// 32 bytes is maximum for NRF... just use it

#ifdef RX_SETTINGS
//	NRF24_startListening();
#endif
}




static int fcLoopCount = 0;
void FC_Flight_Loop(void)
{
//#define DRONE
#define GROUND_STATION
	while(1)
    {
		FC_Ms_Timer_Start(); // restart timer

#ifdef DRONE
    	if(BMFC_BME280_ConfirmI2C_Comms() == 0) // check for BME280 comm. issues
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1); // turn on LED
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 0); // turn off LED
        }

        if(log_totalErrorCount() != 0) // check for any error occurances
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1); // turn on LED
        }

        /* Compute altitude from BME280 pressure */
		volatile float altitude = getLastAltitude();

		// average the altitude readings with weight
		filteredAltitude = (alpha * altitude) + ((1-alpha) * filteredAltitude);

		telemetryData.altitude = filteredAltitude;

		fcLoopCount++;
		if(fcLoopCount % 10 == 0) // only transmit RF messages every Nth loop cycle
		{
			if(FC_Transmit_32B(&telemetryData)) // transmit data without waiting for ACK
			{
#ifdef UART_DEBUG
				HAL_UART_Transmit(&huart6, (uint8_t *)"Transmit success...\r\n",
					strlen("Transmit success...\r\n"), 10); // print success with 10 ms timeout

				snprintf(myTxData, 32, "Loop # %d   Packet # %lu\r\n",
						fcLoopCount, telemetryData.count);
				HAL_UART_Transmit(&huart6, (uint8_t *)myTxData,
						strlen(myTxData), 10); // 10 ms timeout
#endif

				telemetryData.count++; // increment packet number
			}
		}


		NRF24_startListening();

		HAL_Delay(50);


		if(NRF24_available())
		{
#ifdef UART_DEBUG
			HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n", 
				strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
#endif // UART_DEBUG

			NRF24_read(&groundData, sizeof(groundData)); // remember that NRF radio can at most transmit 32 bytes

			//receivedAltitude = *(float *)myRxData; // handle myRxData as a 4 byte float and read the value from it
			volatile float receivedAltitude = groundData.altitude;

			altimeter.preDecimal = (int) receivedAltitude;
			altimeter.postDecimal = (int)((receivedAltitude - altimeter.preDecimal) * 100);

#ifdef UART_DEBUG
			snprintf(myRxData, 32, "Gnd packets %lu \r\n", groundData.count);
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print success with 10 ms timeout

			static int packetsLost = 0;
			static int lastGroundCount = 0;
			if(groundData.count - (lastGroundCount+1) != 0) // if packets have been dropped
			{
				packetsLost += (groundData.count - (lastGroundCount+1));
			}
			snprintf(myRxData, 32, "Packets lost = %d \r\n", packetsLost);
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print success with 10 ms timeout

			lastGroundCount = groundData.count;
#endif // UART_DEBUG

		} // if(NRF24_available())

		HAL_Delay(50);
#endif // DRONE


#ifdef GROUND_STATION

		fcLoopCount++;
		if(fcLoopCount % 10 == 0) // only transmit RF messages every Nth loop cycle
		{
			if(FC_Transmit_32B(&groundData)) // transmit data without waiting for ACK
			{
#ifdef UART_DEBUG
				HAL_UART_Transmit(&huart6, (uint8_t *)"Transmit success...\r\n",
					strlen("Transmit success...\r\n"), 10); // print success with 10 ms timeout

				snprintf(myTxData, 32, "Sent packet # %lu\r\n",
						groundData.count);
				HAL_UART_Transmit(&huart6, (uint8_t *)myTxData,
						strlen(myTxData), 10); // 10 ms timeout
#endif
				groundData.count++;
			}
		}

		NRF24_startListening();

		HAL_Delay(2);

		if(NRF24_available())
		{
#ifdef UART_DEBUG
			HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n",
				strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
#endif // UART_DEBUG

			NRF24_read(&telemetryData, sizeof(telemetryData)); // remember that NRF radio can at most transmit 32 bytes

			//receivedAltitude = *(float *)myRxData; // handle myRxData as a 4 byte float and read the value from it
			volatile float receivedAltitude = telemetryData.altitude;

			altimeter.preDecimal = (int) receivedAltitude;
			altimeter.postDecimal = (int)((receivedAltitude - altimeter.preDecimal) * 100);
			snprintf(myRxData, 32, "%u alt: %d.%d \r\n", (uint8_t)telemetryData.count, altimeter.preDecimal, altimeter.postDecimal);

#ifdef UART_DEBUG
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print success with 10 ms timeout

			static int packetsLost = 0;
			static int lastCount = 0;
			if(telemetryData.count - (lastCount+1) != 0)
			{
				packetsLost += (telemetryData.count - (lastCount+1));
			}
			snprintf(myRxData, 32, "Packets lost = %d \r\n", packetsLost);
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print success with 10 ms timeout

			lastCount = telemetryData.count;
#endif // UART_DEBUG

		} // if(NRF24_available())

#endif // GROUND_STATION


		volatile uint32_t period = FC_Elapsed_Ms_Since_Timer_Start();
		if(period < 1)
			period = 1;
		volatile float frequency = 1 / (period / 1000.0);
		frequency++;
    } // while(1)
} // void BMFC_Flight_Loop(void)


/* Restart the millisecond timer */
static uint32_t tickstart;
void FC_Ms_Timer_Start(void)
{
	tickstart = HAL_GetTick(); // take note of current ms tick count
}

/* Return milliseconds since timer was started */
uint32_t FC_Elapsed_Ms_Since_Timer_Start(void)
{
	return HAL_GetTick() - tickstart;
}
