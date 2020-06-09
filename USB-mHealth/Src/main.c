/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void moveArm(int16_t accel[3], int16_t gyro[3], int16_t mag[3]);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint64_t RxpipeAddrs = 0x11223344AA;
int16_t rx_buffer[10];
int16_t accel[3];
int16_t gyro[3];
int16_t mag[3];
int16_t ppg;
char myAckPayload[32] = "Ack by ST32F103";
char uart_buffer[100];
char usbBuffer[100];

float posS = 100;
float posR = 90;
float posH = 50;
uint8_t rate = 25;
int16_t posR1 = 0;
int16_t posS1 = 0;
int16_t posH1 = 0;
int8_t n = 0;
uint8_t x = 1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
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

  /* USER CODE
   *  BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  NRF24_begin(CSN_Pin_GPIO_Port, CSN_Pin_Pin, CE_Pin_Pin, hspi1);
  nrf24_DebugUART_Init(huart1);

  //**** TRANSMIT - ACK ****//
  NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();

	NRF24_startListening();
  printRadioSettings();

  sprintf(uart_buffer, "G2201 S%i R%i H%i F1000000\r\n", 300, 90, 100);
 HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if(NRF24_available())
	{
		NRF24_read(rx_buffer, sizeof(rx_buffer));
//		HAL_UART_Transmit(&huart1, rx_buffer, sizeof(rx_buffer), 10);

		memcpy(accel, rx_buffer, sizeof(accel));
		memcpy(gyro, &rx_buffer[3], sizeof(gyro));
		memcpy(mag, &rx_buffer[6], sizeof(mag));
//		ppg = rx_buffer[9];


//		sprintf(uart_buffer, "\1(Ax: %i, Ay: %i, Az: %i)\t(Gx: %i, Gy: %i, Gz: %i)\t(Mx: %i, My: %i, Mz: %i)\n\r",
//			accel[0], accel[1], accel[2],
//			gyro[0], gyro[1], gyro[2],
//			mag[0], mag[1], mag[2]);
//		HAL_UART_Transmit(&huart1, uart_buffer, sizeof(uart_buffer), 10);
		moveArm(accel, gyro, mag);
//		sprintf(usbBuffer, "\1%i,%i,%i|%i,%i,%i|%i,%i,%i|%i",
//					accel_x, accel_y, accel_z,
//					gyro_x, gyro_y, gyro_z,
//					mag_x, mag_y, mag_z, ppg);
//		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, rx_buffer, 64);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  }
  /* USER CODE END 3 */
}

void moveArm(int16_t accel[3], int16_t gyro[3], int16_t mag[3]){

	//ICM_20948_AGMT_t agmt;
	float accx = accel[0];
	float accy = accel[1];
	float accz = accel[2];

	float gyrx = gyro[0];
	float gyry = gyro[1];
	float gyrz = gyro[2];

	float magx = mag[0];
	float magy = mag[1];
	float magz = mag[2];

	posR=posR+gyrz*rate/250;

	//int iposR = round(posR);
	if (posR>180){
		posR = 180;
	}else if (posR<0){
		posR = 0;
	}

	posS= 300 - accx * 0.8;
	//int iposS = round(posS);

	if (posS>300){
		posS = 300;
	}else if (posS<10){
		posS = 10;
	}

	posR1=posR;
	posS1=posS1+posS;

	n=n+1;

	if (n==5){

		posS1=posS1/6;
		posH1=posH1/6;

		int16_t iposR = round(posR1);
		int16_t iposS = round(posS1);
		int16_t iposH = 100;

		sprintf(uart_buffer, "G2201 S%i R%i H%i F1000000\r\n", iposS, iposR, iposH);
		HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
		HAL_Delay(50);

		n=0;

		if (iposR < 52 && iposR > 38 && iposS < 250 && iposS > 200  && x == 1){
				sprintf(uart_buffer, "G2201 S225 R45 H45 F10000\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(2000);

				sprintf(uart_buffer, "G2201 S225 R45 H15 F10000\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(2000);

				sprintf(uart_buffer, "M2231 V1\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(500);

				sprintf(uart_buffer, "G2201 S%i R%i H%i F1000000\r\n", iposS, iposR, iposH);
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(500);

			    x = 0;
			 }

			if (iposR < 142 && iposR > 128 && iposS < 200 && iposS > 150 && x== 0){

				sprintf(uart_buffer, "G2201 S175 R135 H45 F10000\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(2000);

				sprintf(uart_buffer, "G2201 S175 R135 H15 F10000\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(2000);

				sprintf(uart_buffer, "M2231 V0\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(500);

				sprintf(uart_buffer, "G2201 S%i R%i H%i F1000000\r\n", iposS, iposR, iposH);
				HAL_UART_Transmit(&huart1, (uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				HAL_Delay(500);

				x = 1;

			}

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin_Pin|CSN_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin_Pin CSN_Pin_Pin */
  GPIO_InitStruct.Pin = CE_Pin_Pin|CSN_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
