/**
 ******************************************************************************
 * @file           : user_functions.h
 * @brief          : User defined functions, defines and variables for DW1000 UWB project
 ******************************************************************************
 * @attention
 *
 * This file contains all user-defined functions, defines, and global variables
 * that were previously scattered in main.c for better code organization.
 *
 ******************************************************************************
 */

#ifndef __USER_FUNCTIONS_H
#define __USER_FUNCTIONS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
#include <string.h>
#include <stdio.h>

/* User Defines --------------------------------------------------------------*/
#define TRANSMITTER 0
// 1 - TX
// 0 - RX

#define SPI_Transmit_Timeout  100
// Timeout for transmission, in ms

#define MAC802154_HEADER_LENGTH 21

// Registers necessary for PA/LNA amplifier switching
#define GPIO_CTRL_ID            0x26 // Apparently not found, needed for PA/LNA
#define PMSC_ID                 0x36
#define PMSC_TXFINESEQ_OFFSET 	0x26
#define PMSC_TXFINESEQ_DIS_MASK	(0x0)

// External interrupt defines
#define DWM_IRQn EXTI9_5_IRQn
#define DWM_IRQ_PIN GPIO_PIN_5

/* User Global Variables -----------------------------------------------------*/
extern char USB_TX_Buffer[255];
extern dwDevice_t dwm_device;
extern dwDevice_t *dwm;
extern dwOps_t dw_ops;

/* User Function Prototypes --------------------------------------------------*/

// Main setup and loop functions
void setup(void);
void loop(void);

// SPI Interface Functions
void spiRead(dwDevice_t *dev, const void *header, size_t headerLength,
             void *data, size_t dataLength);
void spiWrite(dwDevice_t *dev, const void *header, size_t headerLength,
              const void *data, size_t dataLength);
void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed);
void delayms(dwDevice_t *dev, unsigned int delay);
void reset(dwDevice_t *dev);

// DW1000 Callback Functions
void TX_Callback(dwDevice_t *dev);
void RX_Callback(dwDevice_t *dev);
void receivedFailedCallback(dwDevice_t *dev);
void receivedError(dwDevice_t *dev);
void DW1000_Error_Handler(dwDevice_t *dev);

// Data Communication Functions
void send_Data_Over_UWB(char *data);

// PA/LNA Control Functions
void activate_PA_LNA(void);

// Interrupt Handling Functions
void dwOpsInit(dwDevice_t *device);
int checkIrq(void);

#endif /* __USER_FUNCTIONS_H */
