#include "fifo.h"


void FIFO_Init(FIFO_Handle* FIFO_Handle_X,uint32_t Size_of_FIFO)
{
	FIFO_Handle_X->_Number_of_Element = Size_of_FIFO;
	FIFO_Handle_X->_Size = Size_of_FIFO+1;
	FIFO_Handle_X->_Data_Array = (uint8_t*)malloc(sizeof(uint8_t)*Size_of_FIFO);
	FIFO_Handle_X->_index_get = 0;
	FIFO_Handle_X->_index_put = 0;
}


uint8_t FIFO_Put(FIFO_Handle* FIFO_Handle_X, uint8_t data)
{
	if(FIFO_Handle_X->_index_put == ((FIFO_Handle_X->_index_get - 1 + FIFO_Handle_X->_Size) % FIFO_Handle_X->_Size))
	{
		return 0; //FIFO Full
	}
	*(FIFO_Handle_X->_Data_Array +FIFO_Handle_X->_index_put) = data;
	FIFO_Handle_X->_index_put = (FIFO_Handle_X->_index_put + 1) % FIFO_Handle_X->_Size;
	return 1;//OK
}


uint8_t FIFO_Get(FIFO_Handle* FIFO_Handle_X, uint8_t* data)
{
	if(FIFO_Handle_X->_index_get == FIFO_Handle_X->_index_put)
	{
		return 0; // Fifo is empty
	}
	*data = *(FIFO_Handle_X->_Data_Array+FIFO_Handle_X->_index_get);
	FIFO_Handle_X->_index_get = (FIFO_Handle_X->_index_get + 1) % FIFO_Handle_X->_Size;
	return 1; //OK
}
