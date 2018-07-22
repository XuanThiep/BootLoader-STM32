/*
 * BootLoader.h
 *
 *  Created on: May 24, 2018
 *      Author: Xuan Thiep
 */

#ifndef CUSTOMLIBRARY_INC_BOOTLOADER_H_
#define CUSTOMLIBRARY_INC_BOOTLOADER_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stdlib.h"


/***************************** Define Flash Address *********************************************/

#define	FLASH_USER_START_ADDR	ADDR_FLASH_SECTOR_1		/* User program start from Sector 1 */
#define FLASH_USER_END_ADDR		ADDR_FLASH_END

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)	/* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000)	/* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000)	/* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000)	/* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000)	/* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000)	/* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000)	/* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000)	/* Base address of Sector 7, 256 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08100000)	/* Base address of Sector 8, 256 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x08140000)	/* Base address of Sector 9, 256 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x08180000)	/* Base address of Sector 10, 256 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x081C0000)	/* Base address of Sector 11, 256 Kbytes */
#define ADDR_FLASH_END			((uint32_t)0x081FFFFF)


/***************************** Define Value and Length of Data **********************************/

#define	ACK_VALUE				(0xFF)
#define NACK_VALUE				(0x00)
#define	MAX_LENGTH_OF_DATA		(2048)	/* Bytes */
#define MAX_OPTION_BYTE			(15)	/* Bytes */

#define PACKET_LENGTH_OF_FILE	(15)	/* Bytes */
#define PACKET_DATA				(2048 + 13)	/* Bytes */


/***************************** Communication Protocol ***********************************************
 *
 * 				|-------------------- Packet Length of File (for erase flash )----------------------|
 *	Position: 	|   0	|     1    	|     2		|     3		|     4     |     5     |     6		 	|
 * 				|-----------------------------------------------------------------------------------|
 * 	Value:		| '*'	| NData[2] 	| NData[1]	| NData[0]	| !NData[2] | !NData[1]	| !NData[0]	 	|
 * 				|-----------------------------------------------------------------------------------|
 *	Position: 	|   7		|     8    	|     9  	|    10		|    11     |    12     |    13	 	|
 * 				|-----------------------------------------------------------------------------------|
 * 	Value:		| CRC32[3]	| CRC32[2] 	| CRC32[1]	| CRC32[0]	| !CRC32[3] | !CRC32[2]	| !CRC32[1]	|
 * 				|-----------------------------------------------------------------------------------|
 *	Position: 	|   14		|
 * 				|-----------|
 * 	Value:		| !CRC32[0]	|
 * 				|-----------|
 *
 *
 * 				|------------------ Packet Data (max data length = 2048 bytes) ---------------------|
 *	Position: 	|   0	|     1    	|     2		|     3		|     4     |     5     |      ...	 	|
 * 				|-----------------------------------------------------------------------------------|
 * 	Value:		| '!'	| NData[1] 	| NData[0]	| !NData[1]	| !NData[0] |  Data[0]	| 	   ...    	|
 * 				|-----------------------------------------------------------------------------------|
 *	Position: 	|   n+5		|   n+6    	|   n+7  	|   n+8		|   n+9     |   n+10    |   n+11 	|
 * 				|-----------------------------------------------------------------------------------|
 * 	Value:		| Data[n]	| CRC32[3] 	| CRC32[2]	| CRC32[1]	|  CRC32[0] | !CRC32[3]	| !CRC32[2]	|
 * 				|-----------------------------------------------------------------------------------|
 *	Position: 	|  n+12		|	n+13	|
 * 				|-----------------------|
 * 	Value:		| !CRC32[1]	| !CRC32[0]	|
 * 				|-----------|-----------|
 *
 */


/***************************** Define BootLoader Connection *************************************/

/* UART for Bootloader */
#define	UART_BOOT							USART1
#define UART_BOOT_BAUD						(1000000)


#define UART_BOOT_GPIO_CLK_ENABLE			__HAL_RCC_GPIOA_CLK_ENABLE
#define	UART_BOOT_RCC_CLOCK					RCC_PERIPHCLK_USART1
#define UART_BOOT_CLK_SOURCE				RCC_USART1CLKSOURCE_SYSCLK
#define	UART_BOOT_CLK_ENABLE				__HAL_RCC_USART1_CLK_ENABLE

#define UART_BOOT_TX_PIN					GPIO_PIN_9
#define UART_BOOT_RX_PIN					GPIO_PIN_10
#define UART_BOOT_GPIO_AF					GPIO_AF7_USART1
#define UART_BOOT_GPIO_PORT					GPIOA

#define UART_BOOT_CLK_DISABLE				__HAL_RCC_USART1_CLK_DISABLE
#define UART_BOOT_GPIO_CLK_DISABLE			__HAL_RCC_GPIOA_CLK_DISABLE

#define UART_BOOT_FORCE_RESET				__USART1_FORCE_RESET
#define UART_BOOT_RELEASE_RESET				__USART1_RELEASE_RESET

/* UART for Debug  */
#define	UART_DEBUG							USART6
#define	UART_DEBUG_BAUD						(1000000)

#define UART_DEBUG_GPIO_CLK_ENABLE			__HAL_RCC_GPIOC_CLK_ENABLE
#define	UART_DEBUG_RCC_CLOCK				RCC_PERIPHCLK_USART6
#define UART_DEBUG_CLK_SOURCE				RCC_USART6CLKSOURCE_SYSCLK
#define	UART_DEBUG_CLK_ENABLE				__HAL_RCC_USART6_CLK_ENABLE

#define	UART_GPIO_TX_PIN					GPIO_PIN_6
#define	UART_GPIO_AF						GPIO_AF8_USART6
#define	UART_GPIO_PORT						GPIOC

#define UART_DEBUG_GPIO_CLK_DISABLE			__HAL_RCC_GPIOC_CLK_DISABLE
#define	UART_DEBUG_CLK_DISABLE				__HAL_RCC_USART6_CLK_DISABLE

#define UART_DEBUG_FORCE_RESET				__USART6_FORCE_RESET
#define UART_DEBUG_RELEASE_RESET			__USART6_RELEASE_RESET

#define UART_TIMEOUT						(0x2710)	//10s
#define PING_TIMEOUT						(1000)		//1s

#define CRC32INIT_VALUE						(0xFFFFFFFF)
/***************************** Define Function Prototype ***************************************/

typedef void (*pFunction)(void);
extern UART_HandleTypeDef UartDebugHandle;
extern UART_HandleTypeDef UartBootHandle;
extern uint8_t* UART_Buffer;

/***********************************************************************************************/

/**
 * @brief  This function is used to initialize the UART1 for communicate
 * 		with PC application using Low Level Driver.
 * 		UART - 8 bits data length, Baud = 1000000, 1 bit stop, none parity bit
 *
 * @para	None
 * @retval None
 */
void BootUartInit(void);

/**
 * @brief  This function is used to initialize the UART6  for printing Debug information
 * 		using HAL Driver.
 * 		UART - 8 bits data length, Baud = 1000000, 1 bit stop, none parity bit
 *
 * @para	None
 * @retval None
 */
void DebugUartInit(void);

/**
 * @brief  This function is used to jump to main application
 * @para	None
 * @retval None
 */
void BootLoader_JumpToApp(void);

/**
 * @brief  	This function is used to free all resource
 * @para	None
 * @retval 	None
 */
void BootLoader_FreeResource(void);


/**
 * @brief  This function is used to erase main application in flash
 * @para	None
 * @retval None
 */
void BootLoader_EraseApplicationFlash(uint32_t SizeofProgram);

/**
 * @brief  This function writes a data buffer in flash (data are 8-bit aligned).
 * @note   After writing data buffer, the flash content is checked.
 * @param  FlashAddress: start address for writing data buffer
 * @param  Data: pointer on data buffer (this is 8 bit array)
 * @param  DataLength: length of data buffer (unit is 8-bit)
 * @retval 0: Data successfully written to Flash memory
 *         1: Error occurred while writing data in Flash memory
 *         2: Written Data in flash memory is different from expected one
 */
uint32_t BootLoader_WriteFlash(uint32_t FlashAddress, uint8_t* Data ,uint32_t DataLength);


/**
 * @brief  	This function is used to calculate CRC32 from data buffer
 * @para	None
 * @retval 	CRC32 Value
 */
uint32_t CalculateCRCBuffer(uint32_t initialValue, uint8_t* buffer, uint32_t bufferLength);


/**
 * @brief  	This function is used to send ACK or NACK to PC
 * @para	None
 * @retval 	CRC32 Value
 */
void BootLoaderSendACK(uint8_t ackValue);

#endif /* CUSTOMLIBRARY_INC_BOOTLOADER_H_ */


