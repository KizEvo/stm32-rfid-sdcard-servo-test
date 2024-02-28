/* ============================================================================
 *
 * mfrc522.c
 *
 * Created on: Feb 28, 2024
 * Author: Nguyen Duc Phu
 *
 * ============================================================================
 */

#include <stdint.h>
#include "mfrc522.h"
#include "spi.h"
#include "tim.h"
#include "stm32f10x.h"

// Define the required address byte format Section 8.1.2.3
#define MFRC522_ADDRESS_BYTE_FORMAT(mode, address) ((char)(((mode) | ((address) << 1)) & 0xFE))
// Mode for MFRC522 address byte
#define ADDRESS_BYTE_FORMAT_WRITE 0x00
#define ADDRESS_BYTE_FORMAT_READ	0x80

// Reset MFRC522 and turn on analog part of receiver
// Delay after issuing soft reset command doing this let crystal starts up (Section 8.8.2)
#define MFRC522_SOFT_RESET \
						MFRC522_WriteByteReg(CommandReg, 0x0F);\
						delayMs(1000); 
  

/*============================================================================
 *
 * Initialize MFRC522 module, setup various registers
 *
 *============================================================================
 */
void MFRC522_Init(void)
{
	MFRC522_SOFT_RESET;

	// Reset data rate during transmission/reception, set bit rate 106 kBd
	MFRC522_WriteByteReg(TxModeReg, 0x00);
	MFRC522_WriteByteReg(RxModeReg, 0x00);
	// Reset Miller modulation width to default value
	MFRC522_WriteByteReg(ModWidthReg, 0x26);
	
	// Config timer to enable timeout checks
	MFRC522_WriteByteReg(TModeReg, 0x80); // Timer start automatically at the end of transmission in all communication mode
	MFRC522_WriteByteReg(TPrescalerReg, 0xA9); // ftimer = 13.56 MHz / (2*TPreScaler+1). => Ttimer = 25 us
	MFRC522_WriteByteReg(TReloadRegH, 0x03); // ----------------
	MFRC522_WriteByteReg(TReloadRegL, 0xE8); // Set TReloadReg = 0x3E8 = 1000 => Ttimer * 1000 = 25ms before timeout
	MFRC522_WriteByteReg(TxASKReg, 0x40); // Write to bit 6 => forces a 100 % ASK modulation independent of the ModGsPReg register setting
	MFRC522_WriteByteReg(ModeReg, 0x3D); // Change CRC preset value to 0x6363
	
	// Enable the antenna - 0x80 is the default value, 0x03 to turn on TX1 and TX2
	MFRC522_WriteByteReg(TxControlReg, 0x83);
}

/*============================================================================
 *
 * Write a byte to a MFRC522 register address using SPI interface
 *
 *============================================================================
 */
void MFRC522_WriteByteReg(char regAddr, char valueToWrite)
{
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); 
	// Format address according to datasheet
	char addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_WRITE, regAddr);
	// Write sequence, [addr, data] Section 8.1.2.2
	SPI_TransmitByte(SPI1, addr);
	SPI_TransmitByte(SPI1, valueToWrite);
	// NSS pin, pulled high to disable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

/*============================================================================
 *
 * Write byte stream to a MFRC522 register address using SPI interface
 *
 *============================================================================
 */
void MFRC522_WriteStreamReg(char regAddr, char *valueStream, char streamLen)
{
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); 
	// Format address according to datasheet
	char addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_WRITE, regAddr);
	// Write sequence, [addr, data0, data1, data2, ...] Section 8.1.2.2
	SPI_TransmitByte(SPI1, addr);
	for(char i = 0; i < streamLen; i++)
	{
		SPI_TransmitByte(SPI1, valueStream[i]);
	}
	// NSS pin, pulled high to disable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

/*============================================================================
 *
 * Read a byte from a MFRC522 register address using SPI interface
 *
 *============================================================================
 */
char MFRC522_ReadByteReg(char regAddr)
{
	char result;
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	// Format address according to datasheet
	char addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_READ, regAddr);
	// Read sequence, [addr, data] Section 8.1.2.2
	SPI_TransmitByte(SPI1, addr);
	SPI_ReceiveByte(SPI1, &result, 5);
	// NSS pin, pulled high to disable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	
	return result;
}
