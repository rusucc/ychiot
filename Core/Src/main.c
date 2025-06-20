/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/*

 For USB Virtual COM Port:
 -Link for driver (Windows): https://www.st.com/en/development-tools/stsw-stm32102.html
 -For Serial use Putty: https://www.putty.org/

 */
#include "libdw1000/inc/libdw1000.h"
/*

 Open Source library in C for dw1000
 More details:https://github.com/bitcraze/libdw1000

 Also STM32CubeIde will want to have an include path to the library /inc/:


 Project
 |-Properties
 |--C/C++ Build
 |---Settings
 |----Compiler
 |-----Include Paths
 |------Add /inc/ using GUI
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMITTER 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

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
/* USER CODE BEGIN 0 */

//USB send buffer, write to this using sprintf((char *)USB_TX_Buffer,"...\r\n"); and call CDC_Transmit_FS((uint8_t *)USB_TX_Buffer, sizeof(USB_TX_Buffer)); after
char USB_TX_Buffer[255] = "\0";
#define SPI_Transmit_Timeout  100
//Timeout for transmission, in ms

void spiRead(dwDevice_t *dev, const void *header, size_t headerLength,
		void *data, size_t dataLength) {
	//Function for SPI_Read for decawave device, member of dw_ops .spiRead field
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	//Led ON

	HAL_SPI_Transmit(&hspi1, (uint8_t*) header, headerLength,
	SPI_Transmit_Timeout);

	HAL_SPI_Receive(&hspi1, (uint8_t*) data, dataLength, SPI_Transmit_Timeout);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	//Led OFF
}

void spiWrite(dwDevice_t *dev, const void *header, size_t headerLength,
		const void *data, size_t dataLength) {
	//Function for SPI_Write for decawave device, member of dw_ops .spiWrite field
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	//Led ON

	HAL_SPI_Transmit(&hspi1, (uint8_t*) header, headerLength,
	SPI_Transmit_Timeout);

	HAL_SPI_Transmit(&hspi1, (uint8_t*) data, dataLength, SPI_Transmit_Timeout);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	//Led OFF
}

void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed) {
// Needs to restart SPI using HAL, future improvement
}
void delayms(dwDevice_t *dev, unsigned int delay) {
	HAL_Delay(delay);
}

void reset(dwDevice_t *dev) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	//Open drain output, must not be driven High externally
	//See: DW1000 datasheet, page 8
//Hardware RST pin
}
dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;

dwOps_t dw_ops = { .spiRead = spiRead, .spiWrite = spiWrite, .spiSetSpeed =
		spiSetSpeed, .delayms = delayms, .reset = reset };

//RX and TX callbacks
void TX_Callback(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "TX Complete %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
}
void RX_Callback(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "RX Complete %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
}
void receivedFailedCallback(dwDevice_t *dev){
	sprintf((char*) USB_TX_Buffer, "RX Failed %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}
void receivedError(dwDevice_t *dev){
	sprintf((char*) USB_TX_Buffer, "RX Error %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

void DW1000_Error_Handler(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "DW1000 Error %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}
#define MAC802154_HEADER_LENGTH 21
//Data Transmit and Receive Functions:
void send_Data_Over_UWB(char *data) {
	uint8_t txPacket[255];
	size_t dataLength = strlen(data);
	
	// Copy data to packet buffer
	memcpy(txPacket, data, dataLength);
	
	dwNewTransmit(dwm);
	dwSetDefaults(dwm);
	dwSetData(dwm, txPacket, dataLength);
	dwStartTransmit(dwm);
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	sprintf((char*) USB_TX_Buffer, "Starting test \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	//Send over USB the start of program after 1s
//	for (int i = 0; i < 700; i++) {
//		HAL_Delay(5);
//		sprintf((char*) USB_TX_Buffer, "Waiting %d \n\r\0", i);
//		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
//	}

	sprintf((char*) USB_TX_Buffer, "Test:Init for DW1000 \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_Delay(1000);
	//Initialize Decawave

	dwInit(dwm, &dw_ops);

	// Enable NVIC interrupt for DW1000
	dwOpsInit(dwm);

	uint8_t result = dwConfigure(dwm); // Configure the dw1000 chip

	if (result == 0) {
		sprintf((char*) USB_TX_Buffer, "Ok \n\r\0");
		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
		dwEnableAllLeds(dwm);
	} else {
		sprintf((char*) USB_TX_Buffer, "[ERROR]: %s\n\r\0", dwStrError(result));
		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	}
	//Decawave config end

	sprintf((char*) USB_TX_Buffer, "Config End \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_Delay(1000);

	//Continue with config:
	dwTime_t delay = { .full = 0 };
	dwSetAntenaDelay(dwm, delay);

	dwAttachSentHandler(dwm, TX_Callback);
	dwAttachReceivedHandler(dwm, RX_Callback);
	dwAttachErrorHandler(dwm, DW1000_Error_Handler);
	dwAttachReceiveFailedHandler(dwm, receivedFailedCallback);

	dwNewConfiguration(dwm);
	dwSetDefaults(dwm);

	dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(dwm, CHANNEL_2);
	dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

	dwCommitConfiguration(dwm);

	// Enable DW1000 interrupts for receive operations
	dwInterruptOnReceived(dwm, true);
	dwInterruptOnReceiveFailed(dwm, true);
	dwInterruptOnReceiveTimeout(dwm, true);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int i = 0;
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (TRANSMITTER == 1) {
			i++;
			sprintf((char*) USB_TX_Buffer, "Cycle no %d \n\r\0", i);
			CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));

			char mesaj[] = "Hello UWB World! \n\r\0";
			send_Data_Over_UWB(mesaj);
			HAL_Delay(100);
		} else {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			
			// Only start new receive if not already receiving
			if (!dwIsReceiveDone(dwm) && !dwIsReceiveFailed(dwm)) {
				// Already receiving, just wait
				HAL_Delay(10);
			} else {
				// Start new receive cycle
				dwNewReceive(dwm);
				dwSetDefaults(dwm);
				dwStartReceive(dwm);
				HAL_Delay(10);
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_RSTn_GPIO_Port, DW_RSTn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USB_EN_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DW_RSTn_Pin */
  GPIO_InitStruct.Pin = DW_RSTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RSTn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_EN_Pin NSS_Pin */
  GPIO_InitStruct.Pin = USB_EN_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//External interrupt handle
#define DWM_IRQn EXTI9_5_IRQn
#define DWM_IRQ_PIN GPIO_PIN_5
static int checkIrq()
{
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == DWM_IRQ_PIN) // If The INT Source Is EXTI Line9 (A9 Pin)
	{ //GPIO_PIN_5- PB5
		do{
			dwHandleInterrupt(dwm);
		}while(checkIrq() != 0);
		HAL_NVIC_ClearPendingIRQ(DWM_IRQn);
	}
}

void dwOpsInit(dwDevice_t *device) {
	//dev = device;

	NVIC_EnableIRQ(DWM_IRQn);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
