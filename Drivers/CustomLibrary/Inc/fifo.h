#ifndef __MY_FIFO_H
#define __MY_FIFO_H

#include "Application.h"
#include "stdlib.h"

typedef struct
{
	uint32_t 	_Number_of_Element;
	uint32_t 	_Size;
	uint8_t*	_Data_Array;
	uint32_t 	_index_put;
	uint32_t 	_index_get;
}FIFO_Handle;



void FIFO_Init(FIFO_Handle* FIFO_Handle_X,uint32_t Size_of_FIFO);

uint8_t FIFO_Put(FIFO_Handle* FIFO_Handle_X, uint8_t data);

uint8_t FIFO_Get(FIFO_Handle* FIFO_Handle_X, uint8_t* data);



#endif


