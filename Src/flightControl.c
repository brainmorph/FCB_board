/*
 * flightControl.c
 *
 *  Created on: Jun 9, 2020
 *      Author: DC
 */

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
	while(1)
    {
    	if(BMFC_BME280_ConfirmI2C_Comms() == 0) // confirm I2C with BME280 is still ok
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 0);
        }

        if(log_totalErrorCount() != 0)
        {
            HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1);
        }



		volatile float altitude = getLastAltitude();

		/* convert float to string.  STAY BACK */
		altimeter.preDecimal = (int) altitude;
		altimeter.postDecimal = (int)((altitude - altimeter.preDecimal) * 100);


		//	  snprintf(myTxData, 32, "Altitude in meters: %d.%d\r\n",
		//			  altimeter.preDecimal, altimeter.postDecimal);
		//	  HAL_UART_Transmit(&huart6, (uint8_t *)myTxData,
		//			  strlen(myTxData), 10); // print success with 10 ms timeout

		// do the calculation again but for filtered altitude values
		filteredAltitude = (alpha * altitude) + ((1-alpha) * filteredAltitude);
		altimeter.preDecimal = (int) filteredAltitude;
		altimeter.postDecimal = (int)((filteredAltitude - altimeter.preDecimal) * 100);

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
			}
			telemetryData.count++; // increment packet number
		}


		NRF24_startListening();

		HAL_Delay(200);


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

		NRF24_stopListening();   // just in case
		HAL_Delay(200);

    } // while(1)
} // void BMFC_Flight_Loop(void)
