
#include "transceiver.h"
#include "MY_NRF24.h"
//#include "main.h"

// This function sends the first 32 bytes starting from memory address of buffer
int FC_Transmit_Xbytes(void* bufferAddr, uint8_t len)
{
	NRF24_stopListening();   // TODO: How long does this call take and is it necessary?

	//HAL_Delay(2); // TODO: Is delay necessary here?

	if(NRF24_write(bufferAddr, len) != 0) // transmit 32 bytes starting from buffer address
	{
		// Success
		return 1;
	}
	else
	{
		return 0;
	}
}

// This function sends the first 32 bytes starting from memory address of buffer
int FC_Transmit_32B(void* bufferAddr)
{
	NRF24_stopListening();   // TODO: How long does this call take and is it necessary?

	//HAL_Delay(2); // TODO: Is delay necessary here?

	if(NRF24_write(bufferAddr, 32) != 0) // transmit 32 bytes starting from buffer address
	{
		// Success
		return 1;
	}
	else
	{
		return 0;
	}
}
