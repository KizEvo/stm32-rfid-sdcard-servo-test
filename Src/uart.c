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

static uint8_t getNumberLength(int32_t x);
static uint32_t getTenMultiplier(uint8_t x);

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
 * Transmit a byte to another device on the serial line
 * Accept USART_TypeDef and char data type as parameter
 * 
 * ============================================================================
 */
void UART_TransmitByte(USART_TypeDef *USARTx, uint8_t byte)
{
	USARTx->DR = byte;
	
	while(!((USARTx->SR & USART_SR_TC) != 0));
}


/* ============================================================================
 *
 * Transmit a string on the serial line
 * Accept USART_TypeDef *USARTx and char *string
 * 
 * ============================================================================
 */
void UART_WriteString(USART_TypeDef *USARTx, char *string)
{
	for(uint16_t i = 0; string[i] != '\0'; i++)
	{
		UART_TransmitByte(USARTx, string[i]);
	}
}


/* ============================================================================
 *
 * Transmit a string on the serial line, maximum length is for a number is 10
 * Accept USART_TypeDef *USARTx and char *string
 * 
 * ============================================================================
 */
void UART_WriteNumberInt(USART_TypeDef *USARTx, int32_t numb)
{
	// Check if numb is < 0, if yes then the number in the string starts at position 1, index 0 is reserved for '-' sign
	uint8_t signedFlag = numb < 0;
	// Convert to positive number
	numb = numb < 0 ? numb * (-1) : numb;
	uint8_t numbLen = signedFlag + getNumberLength(numb);
	char string[12] = {}; // +2 for null terminated char and '-' sign
	string[numbLen] = '\0';
	for(uint8_t i = signedFlag; i < numbLen; i++)
	{
	  uint32_t tmpMul = getTenMultiplier(numbLen - i);
		string[i] = (char)(numb/tmpMul) + '0';
		numb = numb % tmpMul;
	}
	UART_WriteString(USARTx, string);
}


/* ============================================================================
 *
 * Get number length helper function
 * Accept signed 32 bits integer
 * 
 * ============================================================================
 */
static uint8_t getNumberLength(int32_t x)
{
		if (x >= 1000000000) return 10;
    if (x >= 100000000)  return 9;
    if (x >= 10000000)   return 8;
    if (x >= 1000000)    return 7;
    if (x >= 100000)     return 6;
    if (x >= 10000)      return 5;
    if (x >= 1000)       return 4;
    if (x >= 100)        return 3;
    if (x >= 10)         return 2;
    return 1;
}


/* ============================================================================
 *
 * Get the ten multiplier, if x is 2 then we get 10, or x is 4 then we get 1000,
 * The maximum value for x is 10
 * Accept uint8_t number
 * 
 * ============================================================================
 */
static uint32_t getTenMultiplier(uint8_t x)
{
	switch(x)
	{
		case 10:
			return 1000000000;
		case 9:
			return 100000000;
		case 8:
			return 10000000;
		case 7:
			return 1000000;
		case 6:
			return 100000;
		case 5:
			return 10000;
		case 4:
			return 1000;
		case 3:
			return 100;
		case 2:
			return 10;
		case 1:
			return 1;
		default:
			return 0;
	}
}
