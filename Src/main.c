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
#include "stm32f10x.h"                  // Device header
#include "tim.h"												// My TIM header
#include "spi.h"												// My SPI header

int main(void)
{
	// APB2 = 56MHz; APB1 = 56MHz / 2; ADCCLK = APB2 
	// (should change the prescaler if you wish to use ADC, cause max = 14MHz)
	SPI_InitConfig();
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIO_InitTypeDef ledRed;
	ledRed.GPIO_Mode	= GPIO_Mode_Out_PP;
	ledRed.GPIO_Pin		= GPIO_Pin_13;
	ledRed.GPIO_Speed	= GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOC, &ledRed);

	GPIO_WriteBit(GPIOC, ledRed.GPIO_Pin, Bit_RESET);
	while(1)
	{
		SPI_TransmitByte(SPI1, 0xAA);
		delayMs(50);
	}
}
