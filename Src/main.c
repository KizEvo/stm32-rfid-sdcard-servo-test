/* ============================================================================
 *
 * main.c
 *
 * Created on: Feb 25, 2024
 * Author: Nguyen Duc Phu
 *
 * ============================================================================
 */

#include <stdint.h>
#include <string.h>
#include "stm32f10x.h"	// Device header
#include "tim.h"	// My TIM header
#include "spi.h"	// My SPI header
#include "uart.h" // My UART header
#include "mfrc522.h" // My MFRC522 header

void RCC_InitCommonIOPx(void);

int main(void)
{
	// APB2 = 56MHz; APB1 = 56MHz / 2; ADCCLK = APB2 
	// (should change the prescaler if you wish to use ADC, cause max = 14MHz)
	
	RCC_InitCommonIOPx();
	SPI_InitConfig(); // Config SPI
	TIM_PWM_InitConfig(); // Config TIM in PWM mode and enable PWM waveform output pin
	UART_InitConfig(9600);
	
	MFRC522_Init();
	
	GPIO_InitTypeDef ledRed;
	ledRed.GPIO_Mode = GPIO_Mode_Out_PP;
	ledRed.GPIO_Pin = GPIO_Pin_13;
	ledRed.GPIO_Speed = GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOC, &ledRed);

	GPIO_WriteBit(GPIOC, ledRed.GPIO_Pin, Bit_RESET);
	char result;
	while(1)
	{
		result = MFRC522_ReadByteReg(VersionReg);
		UART_TransmitByte(USART1, result);
		delayMs(200);
	}
}

void RCC_InitCommonIOPx(void)
{
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN);
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
}
