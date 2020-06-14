

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FC_H_
#define FC_H_

#ifdef __cplusplus
 extern "C" {
#endif


#include "stdint.h"

//#define UART_DEBUG

void FC_Init(void);
void FC_Flight_Loop(void);

void FC_Ms_Timer_Start(void);
uint32_t FC_Elapsed_Ms_Since_Timer_Start(void);

#endif // FC_H_
