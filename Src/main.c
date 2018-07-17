
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Application.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef UartDebugHandle;
UART_HandleTypeDef UartBootHandle;
//volatile uint8_t UART_Buffer[MAX_LENGTH_OF_DATA + MAX_OPTION_BYTE];
uint8_t* UART_Buffer;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void BootLoader_TaskFunction(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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


	/* USER CODE BEGIN 2 */
	DebugUartInit();
	BootUartInit();

	setbuf(stdout, NULL);	//Options to print immediately
	DebugPrintf("\r\nBoot Loader Start !!!");

	UART_Buffer = (uint8_t*) malloc(sizeof(uint8_t) * (MAX_LENGTH_OF_DATA + MAX_OPTION_BYTE));
	if(UART_Buffer != NULL)
	{
		DebugPrintf("\r\nMalloc OK ");
		BootLoader_TaskFunction();
	}
	else
	{
		DebugPrintf("\r\nMalloc Fail");
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */



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
 */


static void BootLoader_TaskFunction(void)
{
	/* Wait receive PING, which is send from PC */
	if(HAL_UART_Receive(&UartBootHandle,(uint8_t* )UART_Buffer,1,PING_TIMEOUT) == HAL_OK)
	{
		if(*UART_Buffer == ACK_VALUE)
		{
			/* Send ACK back to PC */
			BootLoaderSendACK(ACK_VALUE);
			DebugPrintf("\r\nACK PING!");

			HAL_Delay(100);

			if(__HAL_USART_GET_FLAG(&UartBootHandle,USART_FLAG_RXNE))
			{
				__HAL_USART_SEND_REQ(&UartBootHandle,USART_RXDATA_FLUSH_REQUEST);
			}

			/* Receive Packet length of file for erase flash */
			HAL_UART_Receive(&UartBootHandle,(uint8_t* )UART_Buffer,PACKET_LENGTH_OF_FILE,UART_TIMEOUT);

			/* Processing Packet length of file */
			if(*UART_Buffer == '*')
			{
				volatile uint32_t lengthOfFile,NlengthOfFile;
				lengthOfFile = (UART_Buffer[1] << 16) | (UART_Buffer[2] << 8) | (UART_Buffer[3]);
				NlengthOfFile = (UART_Buffer[4] << 16) | (UART_Buffer[5] << 8) | (UART_Buffer[6]);

				if(lengthOfFile == (NlengthOfFile ^ 0xffffff)) //3Bytes length of file
				{
					volatile uint32_t CRC32,NCRC32;
					CRC32 = (UART_Buffer[7] << 24) | (UART_Buffer[8] << 16) | (UART_Buffer[9] <<8) | (UART_Buffer[10]);
					NCRC32 = (UART_Buffer[11] << 24) | (UART_Buffer[12] << 16) | (UART_Buffer[13] <<8) | (UART_Buffer[14]);
					if(CRC32 == (NCRC32 ^ 0xffffffff))
					{
						if(CalculateCRCBuffer(CRC32INIT_VALUE,(uint8_t*) (UART_Buffer + 1), 6) == CRC32)
						{
							DebugPrintf("\r\nData Length Packet CRC is Correct [%lu] bytes",lengthOfFile);

							/* Erase flash for new data program */
							BootLoader_EraseApplicationFlash(lengthOfFile);
							DebugPrintf("\r\nErase flash done");

							/* Send ACK back to PC */
							BootLoaderSendACK(ACK_VALUE);
							DebugPrintf("\r\nACK PACKET LENGTH OF FILE!");

							/*Receive Packet Data */
							volatile uint32_t countData = 0;
							while(countData != lengthOfFile)
							{
								HAL_UART_Receive(&UartBootHandle,(uint8_t* )UART_Buffer,PACKET_DATA,UART_TIMEOUT);

								if(*UART_Buffer == '!')
								{
									volatile uint16_t datalength = (UART_Buffer[1] << 8) | (UART_Buffer[2]);
									volatile uint16_t Ndatalength = (UART_Buffer[3] << 8) | (UART_Buffer[4]);
									if(datalength == (Ndatalength ^ 0xffff))
									{
										CRC32 = 	(UART_Buffer[5 + datalength] << 24)	|
												(UART_Buffer[5 + datalength +1] << 16) 	|
												(UART_Buffer[5 + datalength + 2] <<8) 	|
												(UART_Buffer[5 + datalength + 3]);

										NCRC32 = 	(UART_Buffer[5 + datalength + 4] << 24) |
												(UART_Buffer[5 + datalength + 5] << 16) |
												(UART_Buffer[5 + datalength + 6] <<8) 	|
												(UART_Buffer[5 + datalength + 7]);
										if(CRC32 == (NCRC32 ^ 0xffffffff))
										{
											if(CalculateCRCBuffer(CRC32INIT_VALUE,(uint8_t*)UART_Buffer+1, datalength + 4) == CRC32)
											{
												uint8_t resultValue = BootLoader_WriteFlash(FLASH_USER_START_ADDR + countData, (uint8_t*)(UART_Buffer+5), datalength);
												if( resultValue == 0)
												{
													DebugPrintf("\r\nACK PACKET DATA [%d] bytes",datalength);
													countData += datalength;
													BootLoaderSendACK(ACK_VALUE);
													if(countData == lengthOfFile)
													{
														DebugPrintf("\r\nProgram Success");
														break;
													}
												}
												else
												{
													DebugPrintf("\r\nWrite Flash Error code = %d",resultValue);
												}
											}
											else
											{
												DebugPrintf("\r\nCRC Data is NOT correct");
											}
										}
									}
								}
							}
							BootLoader_JumpToApp();
						}
						else
						{
							DebugPrintf("\r\nData Length Packet CRC is NOT Correct");
						}
					}
				}
			}
		}
	}
	else
	{
		DebugPrintf("\r\nBootLoader Timeout !!!");
		BootLoader_JumpToApp();
	}
}



/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartDebugHandle, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar( *ptr++ );
	}
	return len;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
		printf("\r\nError file %s line %d",file,line);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
