/*
 * BootLoader.c
 *
 *  Created on: May 24, 2018
 *      Author: Xuan Thiep
 */

#include "BootLoader.h"


static void BootUartDeInit(void);
static uint32_t GetSector(uint32_t Address);
static uint32_t calculateCRCWord(uint32_t initial_value, uint8_t data);


/**
 * @brief  This function is used to initialize the UART1 for communicate
 * 		with PC application using Low Level Driver.
 * 		UART - 8 bits data length, Baud = 1000000, 1 bit stop, none parity bit
 *
 * @para	None
 * @retval None
 */
void BootUartInit(void)
{
	UartBootHandle.Instance = UART_BOOT;
	UartBootHandle.Init.BaudRate = UART_BOOT_BAUD;
	UartBootHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartBootHandle.Init.StopBits = UART_STOPBITS_1;
	UartBootHandle.Init.Parity = UART_PARITY_NONE;
	UartBootHandle.Init.Mode = UART_MODE_TX_RX;
	UartBootHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartBootHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	UartBootHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	UartBootHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&UartBootHandle);
}


/**
 * @brief BootUartDMADeInit
 * @param None
 * @retval None
 */
static void BootUartDeInit(void)
{
	UART_BOOT->CR1 &=  ~USART_CR1_UE;
	UART_BOOT->CR1 = 0x0U;
	UART_BOOT->CR2 = 0x0U;
	UART_BOOT->CR3 = 0x0U;

	UART_BOOT_CLK_DISABLE();
	UART_BOOT_GPIO_CLK_DISABLE();
}

/**
 * @brief  This function is used to initialize the UART6  for printing Debug information
 * 		using HAL Driver.
 * 		UART - 8 bits data length, Baud = 1000000, 1 bit stop, none parity bit
 *
 * @para	None
 * @retval None
 */
void DebugUartInit(void)
{
	UartDebugHandle.Instance = UART_DEBUG;
	UartDebugHandle.Init.BaudRate = UART_DEBUG_BAUD;
	UartDebugHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartDebugHandle.Init.StopBits = UART_STOPBITS_1;
	UartDebugHandle.Init.Parity = UART_PARITY_NONE;
	UartDebugHandle.Init.Mode = UART_MODE_TX;
	UartDebugHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartDebugHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	UartDebugHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	UartDebugHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&UartDebugHandle);
}


/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UartDebugHandle.Instance)
	{
		GPIO_InitTypeDef  GPIO_InitStruct;

		RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

		/* Enable Clock for GPIO TX of UART Debug */
		UART_DEBUG_GPIO_CLK_ENABLE();

		/* Set clock source for UART Debug */
		RCC_PeriphClkInit.PeriphClockSelection = UART_DEBUG_RCC_CLOCK;
		RCC_PeriphClkInit.Usart6ClockSelection = UART_DEBUG_CLK_SOURCE;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

		UART_DEBUG_CLK_ENABLE();

		GPIO_InitStruct.Pin = UART_GPIO_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = UART_GPIO_AF;

		HAL_GPIO_Init(UART_GPIO_PORT,&GPIO_InitStruct);
	}
	else if(huart->Instance == UartBootHandle.Instance)
	{
		GPIO_InitTypeDef  GPIO_InitStruct;

		RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

		/* Enable Clock for GPIO TX of UART BOOT */
		UART_BOOT_GPIO_CLK_ENABLE();

		/* Set clock source for UART BOOT */
		RCC_PeriphClkInit.PeriphClockSelection = UART_BOOT_RCC_CLOCK;
		RCC_PeriphClkInit.Usart1ClockSelection = UART_BOOT_CLK_SOURCE;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

		UART_BOOT_CLK_ENABLE();

		GPIO_InitStruct.Pin = UART_BOOT_TX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = UART_BOOT_GPIO_AF;

		HAL_GPIO_Init(UART_BOOT_GPIO_PORT,&GPIO_InitStruct);

		GPIO_InitStruct.Pin = UART_BOOT_RX_PIN;
		HAL_GPIO_Init(UART_BOOT_GPIO_PORT,&GPIO_InitStruct);

	}

}


/**
 * @brief UART MSP DeInit
 * @param huart uart handle
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UartDebugHandle.Instance)
	{
		UART_DEBUG_FORCE_RESET();
		UART_DEBUG_RELEASE_RESET();
		UART_DEBUG_GPIO_CLK_DISABLE();
		UART_DEBUG_CLK_DISABLE();
		HAL_GPIO_DeInit(UART_GPIO_PORT, UART_GPIO_TX_PIN);
	}
	else if (huart->Instance == UartBootHandle.Instance)
	{
		UART_BOOT_FORCE_RESET();
		UART_BOOT_RELEASE_RESET();
		UART_BOOT_GPIO_CLK_DISABLE();
		UART_BOOT_CLK_DISABLE();
		HAL_GPIO_DeInit(UART_BOOT_GPIO_PORT, UART_BOOT_TX_PIN);
		HAL_GPIO_DeInit(UART_BOOT_GPIO_PORT, UART_BOOT_RX_PIN);
	}
}


/**
 * @brief  	This function is used to jump to main application
 * @para	None
 * @retval 	None
 */
void BootLoader_JumpToApp(void)
{
	uint32_t  JumpAddress = *(volatile uint32_t*)(FLASH_USER_START_ADDR + 4);
	pFunction Jump = (pFunction)JumpAddress;
	printf("\r\nJump To Application");

	free(UART_Buffer);
	BootLoader_FreeResource();

	/* Set the application stack pointer */
	__set_MSP(*(volatile uint32_t*)FLASH_USER_START_ADDR);

	Jump();
	while(1)
	{
		/* If CPU jump here -> error occur */
		__NOP();
	}
}



/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
static uint32_t GetSector(uint32_t Address)
{
	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		return FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		return FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		return FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		return FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		return FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		return FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		return FLASH_SECTOR_6;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		return FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		return FLASH_SECTOR_8;
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		return FLASH_SECTOR_9;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		return FLASH_SECTOR_10;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
		return FLASH_SECTOR_11;
	}
}

/**
 * @brief  This function is used to free all resource
 * @para	None
 * @retval None
 */
void BootLoader_FreeResource(void)
{
	/* free all resource before jump to main application */
	HAL_UART_DeInit(&UartDebugHandle);
	BootUartDeInit();


	HAL_RCC_DeInit();
	HAL_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;
}




/**
 * @brief  This function is used to erase main application in flash
 * @para	None
 * @retval None
 */
void BootLoader_EraseApplicationFlash(uint32_t SizeofProgram)
{
	uint32_t FirstSector = GetSector(FLASH_USER_START_ADDR);
	uint32_t NbOfSector  = GetSector(FLASH_USER_START_ADDR + SizeofProgram) - FirstSector + 1;
	uint32_t SECTORError;
	FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase 		= FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange 	= FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector 			= FirstSector;
	EraseInitStruct.NbSectors		= NbOfSector;
	HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);

	HAL_FLASH_Lock();
}



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
uint32_t BootLoader_WriteFlash(uint32_t FlashAddress, uint8_t* Data ,uint32_t DataLength)
{
	uint32_t i = 0;
	uint8_t returnValue = 0;

	HAL_FLASH_Unlock();

	/* Integer part */
	for (i = 0; (i < DataLength) && (FlashAddress <= ( FLASH_USER_END_ADDR-4)); i++)
	{
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, FlashAddress, *(uint8_t*)(Data+i)) == HAL_OK)
		{
			/* Check the written value */
			if (*(uint8_t*)FlashAddress != *(uint8_t*)(Data+i))
			{
				/* Flash content doesn't match SRAM content */
				returnValue = (2);
			}
			/* Increment FLASH destination address */
			FlashAddress += 1;
		}
		else
		{
			/* Error occurred while writing data in Flash memory */
			returnValue = (1);
		}
	}

	HAL_FLASH_Lock();
	return returnValue;
}


/**
 * @brief  	This function is used to calculate CRC32 from data
 * @para	None
 * @retval 	CRC32 Value
 */
static uint32_t calculateCRCWord(uint32_t initial_value, uint8_t data)
{
	uint32_t temp = initial_value ^ data;

	for (uint32_t i = 0; i < 32; i++)
	{
		if ((temp & 0x80000000) != 0)
		{
			temp = (temp << 1) ^ 0x04C11DB7; // Polynomial used in STM32
		}
		else
		{
			temp <<= 1;
		}
	}

	return (temp);
}



/**
 * @brief  	This function is used to calculate CRC32 from data buffer
 * @para	None
 * @retval 	CRC32 Value
 */
uint32_t CalculateCRCBuffer(uint32_t initialValue, uint8_t* buffer, uint32_t bufferLength)
{
	uint32_t temp = initialValue;
	for (uint32_t i = 0; i < bufferLength; i++)
	{
		temp = calculateCRCWord(temp, *(buffer+i));
	}

	return temp;
}



/**
 * @brief  	This function is used to send ACK or NACK to PC
 * @para	None
 * @retval 	CRC32 Value
 */
void BootLoaderSendACK(uint8_t ackValue)
{
	uint8_t sendData = ackValue;
	HAL_UART_Transmit(&UartBootHandle,(uint8_t* )&sendData,1,UART_TIMEOUT);
}
