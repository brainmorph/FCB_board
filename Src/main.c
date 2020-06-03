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
void testReadBME280(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void testReadBME280(void)
{
  //----TEST I2C----- // TODO: remove
  volatile uint8_t registerVal = bme280ReadReg(0xD0);
  bme280WriteReg(0xF4, 0x07); // wake the BME280 sensor

  //Read chip ID register just to make sure BM280 is ok
//  volatile uint8_t chipID = bme280ReadReg(BME280_CHIP_ID_REG);

  int32_t tRaw = 0;
  int32_t pRaw = 0;
  int32_t hRaw = 0;

  BME280_Read_Calibration();

  HAL_Delay(100);

  //while(1)
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

void re_readBME280()
{
	int32_t tRaw = 0;
	int32_t pRaw = 0;
	int32_t hRaw = 0;

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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  NRF24_begin(GPIOB, SPI1_CS_Pin, SPI1_CE_Pin, hspi1);
  nrf24_DebugUART_Init(huart6);
  printRadioSettings();
  printConfigReg();

  //most basic NRF transmit mode (NO ACK, just transmit)
  uint64_t TxRxpipeAddrs = 0x11223344AA;
  char myTxData[32] = "Hello goobers";
  char myRxData[50];

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
  testReadBME280();
  while (1)
  {
#ifdef TX_SETTINGS
	  /* Transmit data without waiting for ACK */
	  while(1)
	  {
		  volatile uint8_t ctrlReg = bme280ReadReg(BME280_CTRL_MEAS_REG);
		  volatile uint8_t configReg = bme280ReadReg(BME280_CONFIG_REG);

		  HAL_Delay(100);
		  re_readBME280();

		  volatile float altitude = getCurrentAltitude();

		  /* convert float to string.  STAY BACK */
		  volatile int preDecimal = (int) altitude;
		  volatile int postDecimal = (int)((altitude - preDecimal) * 100);



		  snprintf(myTxData, 32, "Altitude in meters: %d.%d\r\n", preDecimal, postDecimal);
		  HAL_UART_Transmit(&huart6, (uint8_t *)myTxData, strlen(myTxData), 10); // print success with 10 ms timeout

	  }
	  volatile float altitude = getCurrentAltitude();

	  /* convert float to string.  STAY BACK */
	  volatile int preDecimal = (int) altitude;
	  volatile int postDecimal = (int)((altitude - preDecimal) * 100 );



	  snprintf(myTxData, 32, "Altitude in meters: %d.%d", preDecimal, postDecimal);
	  HAL_UART_Transmit(&huart6, (uint8_t *)myTxData, strlen(myTxData), 10); // print success with 10 ms timeout
	  if(NRF24_write(myTxData, 32) != 0) // NRF maximum payload length is 32 bytes
	  {
		  HAL_UART_Transmit(&huart6, (uint8_t *)"Tx success\r\n", strlen("Tx success\r\n"), 10); // print success with 10 ms timeout
	  }


	  HAL_Delay(1000); // delay a second

	  HAL_UART_Transmit(&huart6, (uint8_t *)"Retrying...\r\n", strlen("Retrying...\r\n"), 10); // print success with 10 ms timeout
#endif

#ifdef RX_SETTINGS
	  if(NRF24_available())
	  {
		  HAL_UART_Transmit(&huart6, (uint8_t *)"Radio data available...\r\n", strlen("Radio data available...\r\n"), 10); // print success with 10 ms timeout
		  NRF24_read(myRxData, 32); // remember that NRF radio can at most transmit 32 bytes
		  myRxData[32] = '\r';
		  myRxData[32 + 1] = '\n';

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
