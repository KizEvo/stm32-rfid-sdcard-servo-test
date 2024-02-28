/* ============================================================================
 *
 * spi.c
 *
 * Created on: Feb 25, 2024
 * Author: Nguyen Duc Phu
 *
 * ============================================================================
 */

#include "spi.h"
#include "tim.h"
#include "stm32f10x.h" // Device header
#include "stm32f10x_gpio.h" // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_spi.h" // Keil::Device:StdPeriph Drivers:SPI


/* ============================================================================
 *
 * Setup SPI1 in Master mode on APB2 bus.
 * Baud rate prescaler = 8, Data size is 8 bits and MSB is sent first 
 *
 * ============================================================================
 */
void SPI_InitConfig(void)
{
	// Turn on SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // SPI 
	
	// Config IO for SPI1 peripheral
	GPIO_InitTypeDef SPI1_NSS, SPI1_SCK, SPI1_MOSI, SPI1_MISO;
	
	SPI1_NSS.GPIO_Mode = GPIO_Mode_Out_PP;
	SPI1_NSS.GPIO_Pin	= GPIO_Pin_4;
	SPI1_NSS.GPIO_Speed = GPIO_Speed_10MHz;
		
	SPI1_SCK.GPIO_Mode = GPIO_Mode_AF_PP;
	SPI1_SCK.GPIO_Pin	= GPIO_Pin_5;
	SPI1_SCK.GPIO_Speed = GPIO_Speed_50MHz;

	SPI1_MISO.GPIO_Mode	= GPIO_Mode_IN_FLOATING;
	SPI1_MISO.GPIO_Pin = GPIO_Pin_6;
	
	SPI1_MOSI.GPIO_Mode = GPIO_Mode_AF_PP;
	SPI1_MOSI.GPIO_Pin = GPIO_Pin_7;
	SPI1_MOSI.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &SPI1_NSS);
	GPIO_Init(GPIOA, &SPI1_SCK);
	GPIO_Init(GPIOA, &SPI1_MISO);
	GPIO_Init(GPIOA, &SPI1_MOSI);
	
	// Setup SPI1 parameters
	SPI_InitTypeDef SPI1_Reg;
	SPI1_Reg.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI1_Reg.SPI_CPHA = SPI_CPHA_1Edge;
	SPI1_Reg.SPI_CPOL = SPI_CPOL_Low;
	SPI1_Reg.SPI_DataSize = SPI_DataSize_8b;
	SPI1_Reg.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI1_Reg.SPI_Mode	= SPI_Mode_Master;
	SPI1_Reg.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI1_Reg.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI1, &SPI1_Reg);
	SPI_Cmd(SPI1, ENABLE);
}

/* ============================================================================
 *
 * Transmit a byte to Slave on MOSI wire
 * Accept SPI_TypeDef and char data type as parameter
 *
 * ============================================================================
 */
void SPI_TransmitByte(SPI_TypeDef *SPIx, char byte)
{
	SPIx->DR = byte;
	
	while((SPIx->SR & SPI_SR_BSY) != 0);
}

/* ============================================================================
 *
 * Transmit a byte to Slave on MOSI wire
 * Accept SPI_TypeDef and char data type as parameter
 * Return 0 if timeout expired else data received successfully
 *
 * ============================================================================
 */
uint8_t SPI_ReceiveByte(SPI_TypeDef *SPIx, char *byte, uint16_t timeout)
{
	uint8_t returnStatus;
	startTimeOutMs(timeout);
	while(!(checkTimeOut()) && !((SPIx->SR & SPI_SR_RXNE) != 0));	// Wait for data arrival or timeout expired
	
	if(checkTimeOut()) // Timeout expired
	{
		endTimeOutMs();
		returnStatus = 0;
	}
	else // Data received successfully before timeout expired
	{
		endTimeOutMs();
		*byte = (SPIx->DR & 0xFF);
		returnStatus = 1;
	}
	
	return returnStatus;
}

