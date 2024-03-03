/* ============================================================================
 *
 * uart.c
 *
 * Created on: Feb 26, 2024
 * Author: Nguyen Duc Phu
 *
 * ============================================================================
 */

#include "uart.h"
#include "stm32f10x.h"

/* ============================================================================
 *
 * Setup UART1 in transmit mode on APB2 bus
 * Accept uint32_t data type to specify the baudrate for UART communication
 * 
 * ============================================================================
 */
void UART_InitConfig(uint32_t baudRate)
{
	// Turn on UART clock
	RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);

	// Setup IO pins for UART
	GPIO_InitTypeDef TX, RX;
	TX.GPIO_Pin = GPIO_Pin_9;
	TX.GPIO_Mode = GPIO_Mode_AF_PP;
	TX.GPIO_Speed = GPIO_Speed_50MHz;
	
	RX.GPIO_Pin = GPIO_Pin_10;
	RX.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_Init(GPIOA, &TX);
	GPIO_Init(GPIOA, &RX);
	
	// Setup UART parameters
	USART_InitTypeDef UART1_Reg;
	UART1_Reg.USART_BaudRate = baudRate;
	UART1_Reg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART1_Reg.USART_Mode = USART_Mode_Tx | USART_Mode_Tx;
	UART1_Reg.USART_Parity = USART_Parity_No;
	UART1_Reg.USART_StopBits = USART_StopBits_1;
	UART1_Reg.USART_WordLength = USART_WordLength_9b;
	
	USART_Cmd(USART1, ENABLE);
	USART_Init(USART1, &UART1_Reg);
}


/* ============================================================================
 *
 * Transmit a byte to another device on the bus
 * Accept USART_TypeDef and char data type as parameter
 * 
 * ============================================================================
 */
void UART_TransmitByte(USART_TypeDef *USARTx, uint8_t byte)
{
	USARTx->DR = byte;
	
	while(!((USARTx->SR & USART_SR_TC) != 0));
}
