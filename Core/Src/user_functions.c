/**
 ******************************************************************************
 * @file           : user_functions.c
 * @brief          : User defined functions and variables for DW1000 UWB project
 ******************************************************************************
 * @attention
 *
 * This file contains all user-defined functions and global variables
 * organized with setup() and loop() functions for better code structure.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "user_functions.h"

/* User Global Variables -----------------------------------------------------*/
// USB send buffer, write to this using sprintf((char *)USB_TX_Buffer,"...\r\n"); 
// and call CDC_Transmit_FS((uint8_t *)USB_TX_Buffer, sizeof(USB_TX_Buffer)); after
char USB_TX_Buffer[255] = "\0";

dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;

dwOps_t dw_ops = { .spiRead = spiRead, .spiWrite = spiWrite, .spiSetSpeed =
		spiSetSpeed, .delayms = delayms, .reset = reset };

/* External Variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* User Function Implementations ---------------------------------------------*/

/**
 * @brief SPI Read function for decawave device
 * @param dev: pointer to dwDevice_t structure
 * @param header: pointer to header data
 * @param headerLength: length of header data
 * @param data: pointer to data buffer
 * @param dataLength: length of data to read
 * @retval None
 */
void spiRead(dwDevice_t *dev, const void *header, size_t headerLength,
		void *data, size_t dataLength) {
	// Function for SPI_Read for decawave device, member of dw_ops .spiRead field
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // CS Low

	HAL_SPI_Transmit(&hspi1, (uint8_t*) header, headerLength,
	SPI_Transmit_Timeout);
	HAL_SPI_Receive(&hspi1, (uint8_t*) data, dataLength, SPI_Transmit_Timeout);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); // CS High
}

/**
 * @brief SPI Write function for decawave device
 * @param dev: pointer to dwDevice_t structure
 * @param header: pointer to header data
 * @param headerLength: length of header data
 * @param data: pointer to data to write
 * @param dataLength: length of data to write
 * @retval None
 */
void spiWrite(dwDevice_t *dev, const void *header, size_t headerLength,
		const void *data, size_t dataLength) {
	// Function for SPI_Write for decawave device, member of dw_ops .spiWrite field
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // CS Low

	HAL_SPI_Transmit(&hspi1, (uint8_t*) header, headerLength,
	SPI_Transmit_Timeout);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) data, dataLength, SPI_Transmit_Timeout);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); // CS High
}

/**
 * @brief Set SPI speed for decawave device
 * @param dev: pointer to dwDevice_t structure
 * @param speed: SPI speed to set
 * @retval None
 * @note Needs to restart SPI using HAL, future improvement
 */
void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed) {
	// Needs to restart SPI using HAL, future improvement
}

/**
 * @brief Delay function for decawave device
 * @param dev: pointer to dwDevice_t structure
 * @param delay: delay in milliseconds
 * @retval None
 */

void delayms(dwDevice_t *dev, unsigned int delay) {
	HAL_Delay(delay);
}

/**
 * @brief Reset function for decawave device
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void reset(dwDevice_t *dev) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	// Open drain output, must not be driven High externally
	// See: DW1000 datasheet, page 8
	// Hardware RST pin
}

/**
 * @brief TX Complete callback function
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void TX_Callback(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "TX Complete %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
}

/**
 * @brief RX Complete callback function
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void RX_Callback(dwDevice_t *dev) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	HAL_Delay(100);
	sprintf((char*) USB_TX_Buffer, "RX Complete %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
}

/**
 * @brief RX Failed callback function
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void receivedFailedCallback(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "RX Failed %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

/**
 * @brief RX Error callback function
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void receivedError(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "RX Error %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

/**
 * @brief DW1000 Error Handler callback function
 * @param dev: pointer to dwDevice_t structure
 * @retval None
 */
void DW1000_Error_Handler(dwDevice_t *dev) {
	sprintf((char*) USB_TX_Buffer, "DW1000 Error %d \n\r\0", HAL_GetTick());
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

/**
 * @brief Send data over UWB
 * @param data: pointer to data string to send
 * @retval None
 */
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

/**
 * @brief Activate PA/LNA amplifier switching
 * @retval None
 */
void activate_PA_LNA(void) {
	uint32_t reg;
	// Set up MFIO
	reg = dwSpiRead32(dwm, GPIO_CTRL_ID, 0);
	reg |= 0x00014000; // 7 and 8 to mode - to be used with PA
	reg |= 0x00050000; // 8 and 9 to mode - RX/TX testing
	dwSpiWrite32(dwm, GPIO_CTRL_ID, 0x00, reg);
	dwSpiWrite32(dwm, GPIO_CTRL_ID, 0x00, reg);
	// disable fine grain sequencing - this is needed when using PA on the TX
	// Note: dwt_write16bitoffsetreg function call needs to be replaced with proper libdw1000 equivalent
	// dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_DIS_MASK);
}

/**
 * @brief Check IRQ pin status
 * @retval int: IRQ pin status
 */
int checkIrq(void) {
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}

/**
 * @brief Initialize DW1000 operations and enable NVIC interrupt
 * @param device: pointer to dwDevice_t structure
 * @retval None
 */
void dwOpsInit(dwDevice_t *device) {
	NVIC_EnableIRQ(DWM_IRQn);
}

/**
 * @brief GPIO EXTI Callback function
 * @param GPIO_Pin: GPIO pin that triggered the interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == DWM_IRQ_PIN) { // If The INT Source Is EXTI Line9 (A9 Pin)
		// GPIO_PIN_5- PB5
		do {
			dwHandleInterrupt(dwm);
		} while (checkIrq() != 0);
		HAL_NVIC_ClearPendingIRQ(DWM_IRQn);
	}
}

/**
 * @brief Setup function - Initialize DW1000 and configure system
 * @retval None
 */
void setup(void) {
	sprintf((char*) USB_TX_Buffer, "Starting test \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));

	sprintf((char*) USB_TX_Buffer, "Test:Init for DW1000 \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_Delay(1000);

	// Initialize Decawave
	dwInit(dwm, &dw_ops);

	// Enable NVIC interrupt for DW1000
	dwOpsInit(dwm);

	//Device Address:
	uint16_t dwm_Address = 0x11 + TRANSMITTER;
	dwm->networkAndAddress[0] = (dwm_Address & 0xFF);
	dwm->networkAndAddress[1] = ((dwm_Address >> 8) & 0xFF);

	//Device Network:
	uint16_t dwm_Network = 0x10;
	dwm->networkAndAddress[2] = (dwm_Network & 0xFF);
	dwm->networkAndAddress[3] = (dwm_Network >> 8) & 0xFF;

	dwWriteNetworkIdAndDeviceAddress(dwm);
	uint8_t result = dwConfigure(dwm); // Configure the dw1000 chip

	if (result == 0) {
		sprintf((char*) USB_TX_Buffer, "Ok \n\r\0");
		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
		dwEnableAllLeds(dwm);
	} else {
		sprintf((char*) USB_TX_Buffer, "[ERROR]: %s\n\r\0", dwStrError(result));
		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	}

	sprintf((char*) USB_TX_Buffer, "Config End \n\r\0");
	CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));
	HAL_Delay(1000);

	// Continue with config:
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

	// Setup receiver mode if not transmitter
	if (TRANSMITTER == 0) {
		dwNewReceive(dwm);
		dwSetDefaults(dwm);
		dwReceivePermanently(dwm, 1);
		dwStartReceive(dwm);
	}
}

/**
 * @brief Main loop function - Handle transmit/receive operations
 * @retval None
 */
void loop(void) {
	static int i = 0;

	if (TRANSMITTER == 1) {
		i++;
		sprintf((char*) USB_TX_Buffer, "Cycle no %d \n\r\0", i);
		CDC_Transmit_FS((uint8_t*) USB_TX_Buffer, strlen(USB_TX_Buffer));

		char mesaj[] = "Hello UWB World! \n\r\0";
		send_Data_Over_UWB(mesaj);
		HAL_Delay(100);
	} else {
		// Receiver mode - just wait for interrupts
	}
}
