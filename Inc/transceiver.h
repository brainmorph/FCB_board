
#ifndef TRANSCEIVER_H_
#define TRANSCEIVER_H_

#include <stdint.h>
int FC_Transmit_32B(void* bufferAddr); // this expects 32 byte buffer of any type
int FC_Transmit_Xbytes(void* bufferAddr, uint8_t len);

#endif // TRANSCEIVER_H_
