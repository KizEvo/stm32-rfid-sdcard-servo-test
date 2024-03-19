#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f10x.h"

void UART_InitConfig(uint32_t baudRate);
void UART_TransmitByte(USART_TypeDef *USARTx, uint8_t byte);
void UART_WriteString(USART_TypeDef *USARTx, char *string);
void UART_WriteNumberInt(USART_TypeDef *USARTx, int32_t numb);
void UART_WriteHex(USART_TypeDef *USARTx, int32_t numb);
void UART_WriteNumberInt(USART_TypeDef *USARTx, int32_t numb);

#endif
