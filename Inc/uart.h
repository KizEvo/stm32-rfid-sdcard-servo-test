#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f10x.h"

void UART_InitConfig(uint32_t baudRate);
void UART_TransmitByte(USART_TypeDef *USARTx, char byte);

#endif
