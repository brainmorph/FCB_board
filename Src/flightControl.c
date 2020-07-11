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
#include "tim.h"

#include "MY_NRF24.h"
#include "bme280.h"
#include "logging.h"
#include "transceiver.h"
#include "altitude.h"
#include "timer.h"
#include "pitchRollYaw.h"
#include "mpu6050.h"
#include "motorControl.h"

/* Private Variables */
static uint32_t MainFlightLoopTimer = 0;

typedef struct RadioPacket_t // this packet MUST BE 32 bytes in size
{
	uint32_t count; // 4 bytes
	float altitude; // 4 bytes

	float pitch; 	// 4 bytes
	float roll; 	// 4 bytes
	float yaw; 		// 4 bytes

	float deltaT;	// 4 bytes

	/* Fill in the rest of the struct to make it be 32 bytes in total */
	uint32_t garbage5; // 4 bytes
	uint32_t garbage6; // 4 bytes
}RadioPacket_t; // this packet MUST BE 32 bytes in size

RadioPacket_t telemetryData = {0, 0.0, 0.0, 0.0, 0.0};


typedef struct CommandPacket_t // this packet MUST BE 32 bytes in size
{
	uint32_t count;		// 4 bytes

	float throttleSet;  // 4 bytes
	float rollSet;		// 4 bytes
	float pitchSet;		// 4 bytes
	float yawSet;		// 4 bytes

	uint32_t garbage1;	// 4 bytes
	uint32_t garbage2; 	// 4 bytes
	uint32_t garbage3;  // 4 bytes

}CommandPacket_t; // this packet MUST BE 32 bytes in size

CommandPacket_t commandData = {0, 0.0, 0.0, 0.0, 0.0};



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

	InitMPU();
	CollectInitalSensorValues();


	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}




/* Toggles LED based on state of I2C comms with BME280 AND based on any errors */
void Check_Error_Status() {
	if (BMFC_BME280_ConfirmI2C_Comms() == 0)    // check for BME280 comm. issues
			{
		HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1); // turn on LED
	} else {
		HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 0); // turn off LED
	}
	if (log_totalErrorCount() != 0) // check for any error occurances
			{
		HAL_GPIO_WritePin(GPIOB, BME280_STATUS_LED_Pin, 1); // turn on LED
	}
}

#define UART_DEBUG
extern StateData_t stateData;
static int fcLoopCount = 0;
void FC_Flight_Loop(void)
{
//#define FLIGHT_PLATFORM
#define GROUND_STATION
	while(1)
    {
		Ms_Timer_Start(&MainFlightLoopTimer); // restart timer

#ifdef FLIGHT_PLATFORM
		Check_Error_Status(); // toggle LED based on any error detection


		/* Transmit RF every Nth loop cycle */
		fcLoopCount++;
		if(fcLoopCount % 10 == 0)
		{
			/* Load up the data to send */
			telemetryData.altitude = stateData.altitude;
			telemetryData.pitch = stateData.pitch;
			telemetryData.roll = stateData.roll;
			telemetryData.yaw = stateData.yaw;
			telemetryData.deltaT = stateData.deltaT;

			/* Send data */
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
		//HAL_Delay(2);


		static volatile uint32_t receivedCount = 0;
		static volatile float receivedThrottle = 0.0;
		static volatile float receivedRoll = 0.0;
		static volatile float receivedPitch = 0.0;
		static volatile float receivedYaw = 0.0;
		if(NRF24_available())
		{
#ifdef UART_DEBUG
			HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n", 
				strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
#endif // UART_DEBUG

			NRF24_read(&commandData, sizeof(commandData)); // remember that NRF radio can at most transmit 32 bytes

			receivedCount = commandData.count;


			/* Constrain allowable throttle settings */
			if((commandData.throttleSet >= 0.0) && (commandData.throttleSet < 100.0))
				receivedThrottle = commandData.throttleSet;

			/* Constrain allowable angle settings */
			float cappedAngle = 40.0; // only allow angle settings between -40 < x < 40
			if((commandData.rollSet >= -cappedAngle) && (commandData.rollSet < cappedAngle))
				receivedRoll = commandData.rollSet;
			if((commandData.pitchSet >= -cappedAngle) && (commandData.pitchSet < cappedAngle))
				receivedPitch = commandData.pitchSet;
			if((commandData.yawSet >= -cappedAngle) && (commandData.yawSet < cappedAngle))
				receivedYaw = commandData.yawSet;

			/* Check for dropped packets */
			static uint32_t oldCount = 0;
			static uint32_t droppedPacket = 0;
			if(++oldCount != receivedCount)
			{
				droppedPacket++;
				oldCount = receivedCount;
			}


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

		//HAL_Delay(2);


		/* >>> BY THIS POINT ALL ORIENTATION ANGLES SHOULD BE FULLY COMPUTED <<< */

		CalculatePID(receivedThrottle, receivedRoll, receivedPitch, receivedYaw);



#endif // FLIGHT_PLATFORM


#ifdef GROUND_STATION

		fcLoopCount++;
		if(fcLoopCount % 10 == 0) // only transmit RF messages every Nth loop cycle
		{
			if(FC_Transmit_32B(&commandData)) // transmit data without waiting for ACK
			{
#ifdef UART_DEBUG
//				HAL_UART_Transmit(&huart6, (uint8_t *)"Transmit success...\r\n",
//					strlen("Transmit success...\r\n"), 10); // print success with 10 ms timeout

//				snprintf(myTxData, 32, "Sent packet # %lu\r\n",
//						commandData.count);
//				HAL_UART_Transmit(&huart6, (uint8_t *)myTxData,
//						strlen(myTxData), 10); // 10 ms timeout
#endif
				commandData.count++;
			}
		}

		NRF24_startListening();
		HAL_Delay(1);

		if(NRF24_available())
		{
			static float nrfAfailableCount = 0.0;
			nrfAfailableCount += 1.0;

#ifdef UART_DEBUG
//			HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n",
//				strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
#endif // UART_DEBUG

			NRF24_read(&telemetryData, sizeof(telemetryData)); // remember that NRF radio can at most transmit 32 bytes

			/* Throw away error packets that come in as repeated values */
			if(telemetryData.altitude == telemetryData.pitch)
			{
				continue;
			}




#ifdef UART_DEBUG
			//receivedAltitude = *(float *)myRxData; // handle myRxData as a 4 byte float and read the value from it
			volatile float receivedAltitude = telemetryData.altitude;
			altimeter.preDecimal = (int) receivedAltitude;
			altimeter.postDecimal = (int)((receivedAltitude - altimeter.preDecimal) * 100);
			snprintf(myRxData, 64, "%li alt: %d.%d \r\n", telemetryData.count, altimeter.preDecimal, altimeter.postDecimal);
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print with 10 ms timeout

			static int packetsLost = 0;
			static int lastCount = 0;
			if(telemetryData.count - (lastCount+1) != 0)
			{
				packetsLost += (telemetryData.count - (lastCount+1));
			}


			volatile int lostPacketRatio = (int)(((float)packetsLost/(float)nrfAfailableCount) * 100.0);
			snprintf(myRxData, 64, "Packets lost = %d.  Lost packet ratio = %d %% \r\n", packetsLost, lostPacketRatio);
			HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, strlen(myRxData), 10); // print success with 10 ms timeout

			lastCount = telemetryData.count;
#endif // UART_DEBUG

		} // if(NRF24_available())

#endif // GROUND_STATION


		// Timer test
		volatile uint32_t period = Elapsed_Ms_Since_Timer_Start(&MainFlightLoopTimer);
		if(period < 1)
			period = 1;
		volatile float frequency = 1 / (period / 1000.0);
		frequency = frequency;
    } // while(1)
} // void BMFC_Flight_Loop(void)



