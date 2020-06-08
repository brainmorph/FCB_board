/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
#include "bme280.h"
#include "logging.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  BMFC_BME280_Init(); // Initialize the BME280 sensor

  NRF24_begin(GPIOB, SPI1_CS_Pin, SPI1_CE_Pin, hspi1);
  nrf24_DebugUART_Init(huart6);
  printRadioSettings();
  printConfigReg();

  //most basic NRF transmit mode (NO ACK, just transmit)
  uint64_t TxRxpipeAddrs = 0x11223344AA;
  char myTxData[32] = "Hello goobers";
  char myRxData[50];
  volatile float filteredAltitude = 0;
  volatile float alpha = 0.1;

#define TX_SETTINGS // configure this build for NRF24L01 Transmitter Mode
//#define RX_SETTINGS // configure this build for NRF24L01 Receiver Mode

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
  NRF24_startListening();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  typedef struct
  {
	  volatile int preDecimal;		// signed 32 bit int
	  volatile int postDecimal;		// signed 32 bit int
  }AltimeterData_t;

  AltimeterData_t altimeter = {0, 0};

  int loopCount = 0;
  while (1)
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

#ifdef TX_SETTINGS
	  /* Transmit data without waiting for ACK */

//	  volatile uint8_t ctrlReg = bme280ReadReg(BME280_CTRL_MEAS_REG);
//	  volatile uint8_t configReg = bme280ReadReg(BME280_CONFIG_REG);
//	  volatile uint8_t idReg = bme280ReadReg(0xD0); // should return 0x60

	  //HAL_Delay(50);
	  //BMFC_BME280_Init();
	  //bme280WriteReg(0xF4, 0x27); // wake the BME280 sensor and enable temperature and pressure
	  //HAL_Delay(50);

	  BMFC_BME280_TriggerAltitudeCalculation(); // trigger altitude calculation

	  volatile float altitude = getCurrentAltitude();

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
	  snprintf(myTxData, 32, "Filtered altitude: %d.%d\r\n",
			  altimeter.preDecimal, altimeter.postDecimal);
	  HAL_UART_Transmit(&huart6, (uint8_t *)myTxData,
			  strlen(myTxData), 10); // 10 ms timeout

	  // Transmit over RF
	  memcpy(myTxData, &filteredAltitude,
			  sizeof(filteredAltitude)); // copy float value straight into the beginning of the transmit buffer

	  loopCount++;
	  if(loopCount % 1 == 0) // only transmit RF messages every Nth loop cycle
	  {
		  if(NRF24_write(myTxData, sizeof(myTxData)) != 0)
		  {
			  HAL_UART_Transmit(&huart6, (uint8_t *)"Tx success\r\n",
					  strlen("Tx success\r\n"), 10); // 10 ms timeout
		  }
	  }

//	  HAL_Delay(50); // Is this necessary?

	  HAL_UART_Transmit(&huart6, (uint8_t *)"Retrying...\r\n",
			  strlen("Retrying...\r\n"), 10); // 10 ms timeout
#endif

#ifdef RX_SETTINGS
	  if(NRF24_available())
	  {
		  HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n",
				  strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
		  volatile float receivedAltitude = 0.3;
		  NRF24_read(myRxData, sizeof(myRxData)); // remember that NRF radio can at most transmit 32 bytes
		  myRxData[32] = '\r';
		  myRxData[32 + 1] = '\n';

		  receivedAltitude = *(float *)myRxData; // handle myRxData as a 4 byte float and read the value from it

		  altimeter.preDecimal = (int) receivedAltitude;
		  altimeter.postDecimal = (int)((receivedAltitude - altimeter.preDecimal) * 100);
		  snprintf(myRxData, 32, "Received altitude: %d.%d\r\n", altimeter.preDecimal, altimeter.postDecimal);
		  HAL_UART_Transmit(&huart6, (uint8_t *)myRxData, 32+2, 10); // print success with 10 ms timeout
	  }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
