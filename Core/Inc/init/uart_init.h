#ifndef __UART_INIT_H
#define __UART_INIT_H

#include "main.h"

void UART_Init(void);
UART_HandleTypeDef* UART_GetHandle(void);

#endif /* __UART_INIT_H */
