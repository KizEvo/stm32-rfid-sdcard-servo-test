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
#include <stdlib.h>
#include "mfrc522.h"
#include "spi.h"
#include "tim.h"
#include "stm32f10x.h"

// Define the required address byte format Section 8.1.2.3
#define MFRC522_ADDRESS_BYTE_FORMAT(mode, address) ((uint8_t)(((mode) | ((address) << 1)) & 0xFE))
// Mode for MFRC522 address byte
#define ADDRESS_BYTE_FORMAT_WRITE 0x00
#define ADDRESS_BYTE_FORMAT_READ	0x80

// Reset MFRC522 and turn on analog part of receiver
// Delay after issuing soft reset command doing this let crystal starts up (Section 8.8.2)
#define MFRC522_SOFT_RESET \
	MFRC522_WriteByteReg(CommandReg, SoftReset);\
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
	MFRC522_WriteByteReg(TReloadRegH, 0x03); // Upper 4 bits of TReloadReg
	MFRC522_WriteByteReg(TReloadRegL, 0xE8); // Lower 8 bits of TReloadReg, set TReloadReg = 0x3E8 = 1000 => Ttimer * 1000 = 25ms before timeout
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
void MFRC522_WriteByteReg(uint8_t regAddr, uint8_t valueToWrite)
{
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); 
	// Format address according to datasheet
	uint8_t addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_WRITE, regAddr);
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
void MFRC522_WriteStreamReg(uint8_t regAddr, uint8_t *valueStream, uint8_t streamLen)
{
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); 
	// Format address according to datasheet
	uint8_t addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_WRITE, regAddr);
	// Write sequence, [addr, data0, data1, data2, ...] Section 8.1.2.2
	SPI_TransmitByte(SPI1, addr);
	for(uint8_t i = 0; i < streamLen; i++)
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
uint8_t MFRC522_ReadByteReg(uint8_t regAddr)
{
	uint8_t result;
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	// Format address according to datasheet
	uint8_t addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_READ, regAddr);
	// Read sequence Section 8.1.2.1
	// MOSI = [addr0, addr1, addr2, ...]
	// MISO = [  X  , data0, data1, ...], X = dont care
	SPI_TransmitByte(SPI1, addr);      // send addr0, addr0 contains value we want to read from
	SPI_ReceiveByte(SPI1, &result, 5); // result = X
	SPI_TransmitByte(SPI1, 0x00);      // Send 0x00 to stop reading
	SPI_ReceiveByte(SPI1, &result, 5); // result = data0
	// NSS pin, pulled high to disable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	
	return result;
}

/*============================================================================
 *
 * Read byte stream from a MFRC522 register address using SPI interface
 *
 *============================================================================
 */
void MFRC522_ReadStreamReg(uint8_t regAddr, uint8_t *valueStream, uint8_t streamLen)
{
	// NSS (Chip Select) pin, pulled low to enable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	// Format address according to datasheet
	uint8_t addr = MFRC522_ADDRESS_BYTE_FORMAT(ADDRESS_BYTE_FORMAT_READ, regAddr);
	// Read sequence Section 8.1.2.1
	// MOSI = [addr0, addr1, addr2, ...]
	// MISO = [  X  , data0, data1, ...], X = dont care
	uint8_t tmp;
	SPI_TransmitByte(SPI1, addr); // addr0
	SPI_ReceiveByte(SPI1, &tmp, 5); // X
	for(uint8_t i = 0; i < streamLen - 1; i++) // streamLen - 1 because the last read is outside loop
	{
		SPI_TransmitByte(SPI1, addr); // addr1, addr2, ...
		SPI_ReceiveByte(SPI1, &(valueStream[i]), 5); // data0, data1, ...
	}
	SPI_TransmitByte(SPI1, 0x00); // Send 0x00 to stop reading.
	SPI_ReceiveByte(SPI1, &(valueStream[streamLen - 1]), 5); // Read the final byte. 
	// NSS pin, pulled high to disable slave
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

/*============================================================================
 *
 * Set the bits specified in 'bitmask' in register 'regAddr'
 *
 *============================================================================
 */
void MFRC522_SetRegisterBitMask(uint8_t regAddr, uint8_t bitmask)
{
	uint8_t regValueTmp = MFRC522_ReadByteReg(regAddr);
	MFRC522_WriteByteReg(regAddr, regValueTmp | bitmask);
}


/*============================================================================
 *
 * Clear the bits specified in 'bitmask' in register 'regAddr'
 *
 *============================================================================
 */
void MFRC522_ClearRegisterBitMask(uint8_t regAddr, uint8_t bitmask)
{
	uint8_t regValueTmp = MFRC522_ReadByteReg(regAddr);
	MFRC522_WriteByteReg(regAddr, regValueTmp & (~(bitmask)));
}

/*============================================================================
 *
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * xIRq : indicates which bit in the ComIrqReg should be checked to handle communication appropriately
 * validBits : used for transmission of bit oriented frames: defines the number of bits of the last byte that will be transmitted 
 * rxAlign : used for reception of bit-oriented frames: defines the bit position for the first bit received to be stored in the FIFO buffer
 *============================================================================
 */
uint8_t MFRC522_PICC_Communication(uint8_t command, uint8_t xIRq, uint8_t *sendValStream, uint8_t *sendValLen, uint8_t *receiveValStream, uint8_t *receiveValLen, uint8_t *validBits, uint8_t rxAlign)
{
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) | txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	MFRC522_WriteByteReg(CommandReg, Idle); // No action, cancels current command execution Table 149 - 10.3 MFRC522
	MFRC522_WriteByteReg(ComIrqReg, 0x7F); // Bit 7 = 0 => indicates that the marked bits in the ComIrqReg register are cleared
	MFRC522_WriteByteReg(FIFOLevelReg, 0x80); // Flush FIFO register
	MFRC522_WriteStreamReg(FIFODataReg, sendValStream, *sendValLen); // Write stream to FIFO register (total 64 bytes)
	MFRC522_WriteByteReg(BitFramingReg, bitFraming); // Config bit frame for transmission and reception
	MFRC522_WriteByteReg(CommandReg, command); // Execute the command

	if(command == Transceive) // Transmission of data start, need to activate bit 7th of BitFramingReg if command is Transceive to start sending data to PICC
	{
		MFRC522_SetRegisterBitMask(BitFramingReg, 0x80);
	}
	
	startTimeOutMs(25); // Wait 25ms at the end of transmission (because we set the TAuto bit in TModeReg) for other command to complete
	uint8_t completed = 0;
	uint8_t irqReg;
	while(!(checkTimeOut()))
	{
		irqReg = MFRC522_ReadByteReg(ComIrqReg);
		if(irqReg & xIRq) // Interrupt signals success communication
		{
			endTimeOutMs();
			completed = 1;
			break;
		}
		if(irqReg & 0x01) // Timeout by internal timer of MFRC522
		{
			endTimeOutMs();
			return 0;
		}
	}
	endTimeOutMs();
	
	if(!completed) // Timeout by systick interrupt
	{
		return 0;
	}
	
	uint8_t errorReg = MFRC522_ReadByteReg(ErrorReg);
	if(errorReg & 0x13) // BufferOvfl, ParityErr, ProtocolErr checks
	{
		return 1;
	}
	
	uint8_t validBitsReceived = 0;
	
	if(receiveValStream && receiveValLen)
	{
		uint8_t byteCountInFIFO = MFRC522_ReadByteReg(FIFOLevelReg); // Get number of bytes in FIFO
		if (byteCountInFIFO > *receiveValLen) // Not enough room to write back data from MFRC522_FIFO to MCU
		{
			return 2;
		}
		*receiveValLen = byteCountInFIFO;
		MFRC522_ReadStreamReg(FIFODataReg, receiveValStream, *receiveValLen);
		validBitsReceived = MFRC522_ReadByteReg(ControlReg) & 0x07;
		if(validBits) // If no error, after this assignment validBits should be 0b000 indicates the whole byte is valid
		{
			*validBits = validBitsReceived;
		}
	}
	
	if (errorReg & 0x08) // CollErr
	{
		return 3;
	}
	
	// Perform no CRC check
	
	return UINT8_MAX; // Success
}


/*============================================================================
 *
 * Execute Transceive command
 *
 *============================================================================
 */
uint8_t MFRC522_PICC_TransceiveData(uint8_t *sendValStream, uint8_t *sendValLen, uint8_t *receiveValStream, uint8_t *receiveValLen, uint8_t *validBits, uint8_t rxAlign)
{
	uint8_t xIRq = 0x30; // RxIRq and IdleIRq
	return MFRC522_PICC_Communication(Transceive, xIRq, sendValStream, sendValLen, receiveValStream, receiveValLen, validBits, rxAlign);
}

/*============================================================================
 *
 * Transmit REQA commands to PICC to change PICC state from HALT to READY state
 *
 *============================================================================
 */
uint8_t MFRC522_PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
	uint8_t validBits = 7;
	uint8_t sendValStream = 0x26; // REQA command, Section 6.3.1 ISO-IEC 14443-3_E
	uint8_t sendValLen = 1;

	if(bufferATQA == NULL || *bufferSize < 2)
	{
		return 0;
	}
	
	MFRC522_ClearRegisterBitMask(CollReg, 0x80);
	uint8_t status = MFRC522_PICC_TransceiveData(&sendValStream, &sendValLen, bufferATQA, bufferSize, &validBits, 0);
	
	if (status != UINT8_MAX)
	{
		return status;
	}
	if (*bufferSize != 2 || validBits != 0)
	{
		return 1;
	}
	return UINT8_MAX; // Success
}

/*============================================================================
 *
 * Use the CRC coprocessor in the MFRC522 to calculate CRC_A 
 * *result : Low byte is written first, result[0] = RegLow, result[1] = RegHigh, Check Annex B CRC_A and CRC_B encoding
 *
 *============================================================================
 */
uint8_t MFRC522_CalculateCRC(uint8_t *valueStream, uint8_t streamLen, uint8_t *result)
{
	MFRC522_WriteByteReg(CommandReg, Idle); // No action, cancels current command execution Table 149 - 10.3 MFRC522
	MFRC522_WriteByteReg(DivIrqReg, 0x04); // bit 7 = 0 => indicates that the marked bits in the DivIrqReg register are cleared => CRCIrq[2] is cleared
	MFRC522_WriteByteReg(FIFOLevelReg, 0x80); // Flush FIFO register
	MFRC522_WriteStreamReg(FIFODataReg, valueStream, streamLen); // Write stream to FIFO register (total 64 bytes)
	MFRC522_WriteByteReg(CommandReg, 0x03); // Activates the CRC coprocessor
	
	startTimeOutMs(90); // Wait 90ms at the end of transmission for the CRC command to complete
	while(!(checkTimeOut()))
	{
		uint8_t regValue = MFRC522_ReadByteReg(DivIrqReg);
		if(regValue & 0x04) // CRCIrq is set => the CalcCRC command is active and all data is processed
		{
			MFRC522_WriteByteReg(CommandReg, Idle); // Cancels the CalcCRC command
			endTimeOutMs(); // Clear timeout flag
			result[0] = MFRC522_ReadByteReg(CRCResultRegL);
			result[1] = MFRC522_ReadByteReg(CRCResultRegH);
			return UINT8_MAX; // Calculate CRC success
		}
	}
	endTimeOutMs(); // Clear timeout flag
	return 0; // Timeout
}

/*============================================================================
 *
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Take a loot of the Annex A, Communication example Type A ISO-IEC 14443-3
 *
 *============================================================================
 */
uint8_t MFRC522_PICC_Select(UID *uid)
{
	// Variables implement here
	uint8_t cascadeLevel = 1;
	uint8_t uidCompleted;
	uint8_t useCascadeTag;
	uint8_t selectDone;
	uint8_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t rxAlign;                // Used in BitFramingReg. Defines the bit position for the first bit received
	uint8_t txLastBits;             // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t uidIndex;               // Index in uid->uidByte[uidIndex] that is used in the current Cascade level
	uint8_t currentLvlKnownBits;    // The number of known UID bits in the current Cascade Level
	uint8_t *responseBuffer;
	uint8_t responseLength;
	uint8_t bufferUsed;             // The number of bytes to transfer to the FIFO.
	uint8_t buffIndex;              // Buffer index
	uint8_t buffer[9];
	
	// Description of buffer structure:
	//		Byte 0: SEL               Indicates the Cascade Level: 1, 2 or 2
	//		Byte 1: NVB               Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					      Block Check Character - XOR of byte 2 to byte 5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	
	MFRC522_ClearRegisterBitMask(CollReg, 0x80);
	
	// Repeat Cascade level loop until we have a complete UID.
	uidCompleted = 0;
	while(!uidCompleted)
	{
		switch(cascadeLevel)
		{
			case 1:
				buffer[0] = 0x93; // AntiCollision Cascade level 1, 6.4.3.2 Coding of SEL (Select code)
				uidIndex = 0;
				useCascadeTag = (uid->size > 4);
				break;
			case 2:
				buffer[0] = 0x95; // Cascade level 2
				uidIndex = 3;
				useCascadeTag = (uid->size > 7);
				break;
			case 3:
				buffer[0] = 0x97; // Cascade level 3
				uidIndex = 6;
				useCascadeTag = 0;
			default:
				return 0; // Internal error
		}
		// Check how many uid bits are known in this Cascade level
		currentLvlKnownBits = 8 * uidIndex;
		
		// Copy the known bits from uid->uidByte[] to buffer[]
		buffIndex = 2;
		if(useCascadeTag)
		{
			buffer[buffIndex++] = 0x88; // Cascade tag, check Annex A for sample communication with Cascade tag
		}
		uint8_t bytesToCopy = currentLvlKnownBits / 8;
		if(bytesToCopy)
		{
			uint8_t maxByte = useCascadeTag ? 3 : 4; // Max 4 bytes of UID in each Cascade Level. Only 3 bytes of UID left if we use the Cascade Tag
			if(bytesToCopy > maxByte)
			{
				bytesToCopy = maxByte;
			}
			for(count = 0; count < bytesToCopy; count++)
			{
				buffer[buffIndex++] = uid->uidByte[uidIndex + count];
			}
		}
		
		// Now that the data has been copied we need to include the 8 bits in CT in currentLvlKnownBits
		if (useCascadeTag) 
		{
			currentLvlKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iteration
		selectDone = 0;
		while(!selectDone)
		{
			// Find out how many bits and bytes to send and receive
			if(currentLvlKnownBits >= 32) // This is a SELECT full frame and can be sent to PICC.
			{
				buffer[1] = 0x70; // NVB - number of valid bytes: seven whole bytes, SEL -> BCC (exclude 2 bytes CRC_A)
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = MFRC522_CalculateCRC(buffer, 7, &buffer[7]);
				if(result != UINT8_MAX) // If NOT OK then return
				{
					return result;
				}
				txLastBits = 0; // 0b000 => All bits of the last byte is valid.
				bufferUsed = 9;
				
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else // ANTICOLLISION
			{
				// Section 6.4.3.3
				txLastBits = currentLvlKnownBits % 8; // The lower 4 bits are called ¡°bit count¡± and specify the number of all valid data bits transmitted by the PCD (including SEL and NVB) modulo 8.
				count = currentLvlKnownBits / 8; // Number of whole bytes in the UID part
				buffIndex = 2 + count; // Number of whole bytes: SEL + NVB + UIDs
				buffer[1] = (buffIndex << 4) | txLastBits; // NVB - Number of Valid Bits
				bufferUsed = buffIndex + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[buffIndex];
				responseLength	= sizeof(buffer) - buffIndex;
			}
			
			// Set bit adjustment
			rxAlign = txLastBits;
			MFRC522_WriteByteReg(BitFramingReg, (rxAlign << 4) | txLastBits); // rxAlign = BitFramingReg[6..4]. txLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response
			result = MFRC522_PICC_TransceiveData(buffer, &bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if(result == 3) // More than one PICC in the field => collision error
			{
				uint8_t valueOfCollReg = MFRC522_ReadByteReg(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) // CollPosNotValid
				{
					return 3; // Without a valid collision position we cannot continue
				}
				
				uint8_t collisionPos = valueOfCollReg & 0x1F; //shows the bit position of the first detected collision in a received frame, -> check bit from 0-31, 0x00 means bit 32nd
				if (collisionPos == 0)
				{
					collisionPos = 32;
				}
				if (collisionPos <= currentLvlKnownBits) // No progress - should not happen
				{
					return 0; // Internal error
				}
				// Choose the PICC with the bit set, for example in Annex A ISO IEC 14443-3, first bit collision happened at bit 4th
				currentLvlKnownBits = collisionPos;
				count = currentLvlKnownBits % 8; // count = 4
				checkBit = (currentLvlKnownBits - 1) % 8; // checkBit = 3
				buffIndex = 1 + (currentLvlKnownBits / 8) + (count ? 1 : 0); // First byte is index 0 // buffIndex = 1 + 0 + 1 = 2;
				buffer[buffIndex] |= (1 << checkBit); // buffer order: SEL NVB UID0 UID1 UID2 UID3
				                                      //                0   1   2    3    4    5
				                                      // So this means we set the first collision bit in UID0
			}
			else if(result != UINT8_MAX)
			{
				return result;
			}
			else // Status ok => UINT8_MAX
			{
				if (currentLvlKnownBits >= 32) // This was a SELECT
				{
					selectDone = 1; // No more anti collision
					// Continue below outside the while
				}
				else // This was an anti collision
				{
					// We now have all 32 bits of the UID in this Cascade level
					currentLvlKnownBits = 32;
					// Run loop again to do the SELECT
				}
			}
		} // End of while(!selectDone)
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		buffIndex = (buffer[2] == 0x88) ? 3 : 2; // 0x88 Cascade tag // source index in buffer[]
		bytesToCopy = (buffer[2] == 0x88) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++)
		{
			uid->uidByte[uidIndex + count] = buffer[buffIndex++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0)
		{
			return 1;
		}
		//  Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
	  result = MFRC522_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if(result != UINT8_MAX)
		{
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return 2;
		}
		if(responseBuffer[0] & 0x04) // Cascade bit set - UID not complete yes
		{
			cascadeLevel++;
		}
		else
		{
			uidCompleted = 1;
			uid->sak = responseBuffer[0];
		}
	} // End of while(!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;
	
	return UINT8_MAX;
}


/*============================================================================
 *
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 *============================================================================
 */
uint8_t MFRC522_PICC_IsNewCardPresent(void)
{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	
	MFRC522_WriteByteReg(TxModeReg, 0x00);
	MFRC522_WriteByteReg(RxModeReg, 0x00);
	MFRC522_WriteByteReg(ModWidthReg, 0x26);
	
	return MFRC522_PICC_RequestA(bufferATQA, &bufferSize);
}


/*============================================================================
 *
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 *============================================================================
 */
uint8_t MFRC522_PICC_SelectCard(UID *mfrc522)
{
	return MFRC522_PICC_Select(mfrc522);
}

/*============================================================================
 *
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 *============================================================================
 */
PICC_TYPE MFRC522_GetType(uint8_t sak)
{
	// Application Note AN10833
	// Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
}


/*============================================================================
 *
 * Perform a digital self test. The self test is started by using the following procedure:
 * 1. Perform a soft reset.
 * 2. Clear the internal buffer by writing 25 bytes of 00h and implement the Config command.
 * 3. Enable the self test by writing 09h to the AutoTestReg register.
 * 4. Write 00h to the FIFO buffer.
 * 5. Start the self test with the CalcCRC command.
 * 6. The self test is initiated.
 * 7. When the self test has completed, the FIFO buffer contains the following 64 bytes:
 *
 *============================================================================
 */
void MFRC522_PICC_TestSignal(uint8_t *bufferOut)
{
	// 1. Soft reset
	MFRC522_SOFT_RESET;
	
	// 2. Clear internal buffer
	uint8_t bufferIn[25] = {0x00};
	MFRC522_WriteByteReg(FIFOLevelReg, 0x80); // flush FIFO
	MFRC522_WriteStreamReg(FIFODataReg, bufferIn, 25); // write 25 zero bytes
	MFRC522_WriteByteReg(CommandReg, Mem); // transfer 25 bytes to internal buffer
	// 3. Enable self test
	MFRC522_WriteByteReg(AutoTestReg, 0x09);
	// 4. Write 0x00 to the FIFO buffer
	MFRC522_WriteByteReg(FIFODataReg, 0x00);
	// 5. Perform self test with CalcCRC command
	MFRC522_WriteByteReg(CommandReg, CalcCRC);
	
	// Wait for self test
	while(MFRC522_ReadByteReg(FIFOLevelReg) < 64);
	
	MFRC522_WriteByteReg(CommandReg, Idle);
	// Done, reading 64 bytes from FIFO
	MFRC522_ReadStreamReg(FIFODataReg, bufferOut, 64);
	// Reset AutoTestReg
	MFRC522_WriteByteReg(AutoTestReg, 0x00);
	// Re-init
	MFRC522_Init();
}
