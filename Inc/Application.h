/*
 * Application.h
 *
 *  Created on: May 24, 2018
 *      Author: Xuan Thiep
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "stm32f7xx_hal.h"
#include "BootLoader.h"


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



extern uint8_t* UART_Buffer;

extern UART_HandleTypeDef UartDebugHandle;
extern UART_HandleTypeDef UartBootHandle;



#define	DebugPrintf			printf

#endif /* APPLICATION_H_ */
