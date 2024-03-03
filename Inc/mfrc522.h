#ifndef MFRC522_H
#define MFRC522_H

#include <stdint.h>

//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     ComlEnReg            	0x02
#define     DivlEnReg             0x03
#define     ComIrqReg            	0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxASKReg             	0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MfTxReg             	0x1C
#define     MfRxReg            		0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegH         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F

// MFRC522 Commands
#define			Idle									0x00
#define			Mem										0x01
#define			GenerateRandomID			0x02
#define			CalcCRC								0x03
#define			Transmit							0x04
#define			NoCmdChange						0x07
#define			Receive								0x08
#define			Transceive						0x0C
#define			MFAuthent							0x0E
#define			SoftReset							0x0F

enum PICC_TYPE {PICC_TYPE_NOT_COMPLETE, 
                PICC_TYPE_UNKNOWN, 
                PICC_TYPE_MIFARE_MINI, 
                PICC_TYPE_MIFARE_1K, 
                PICC_TYPE_MIFARE_4K, 
                PICC_TYPE_MIFARE_UL, 
                PICC_TYPE_MIFARE_PLUS, 
                PICC_TYPE_TNP3XXX, 
                PICC_TYPE_ISO_14443_4, 
                PICC_TYPE_ISO_18092};

typedef enum PICC_TYPE PICC_TYPE;
								
typedef struct {
	uint8_t uidByte[10]; // Number of bytes in the UID (4, 7 or 10), Section 6.4.4 ISO-IEC 14443-3_E
	uint8_t size; // Size in number
	uint8_t sak; // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} UID;

// MCU to PCD (MFRC522) communication function prototypes
void MFRC522_Init(void);
void MFRC522_WriteByteReg(uint8_t regAddr, uint8_t valueToWrite);
void MFRC522_WriteStreamReg(uint8_t regAddr, uint8_t *valueStream, uint8_t streamLen);
void MFRC522_SetRegisterBitMask(uint8_t regAddr, uint8_t bitmask);
void MFRC522_ClearRegisterBitMask(uint8_t regAddr, uint8_t bitmask);
uint8_t MFRC522_CalculateCRC(uint8_t *valueStream, uint8_t streamLen, uint8_t *result);
uint8_t MFRC522_ReadByteReg(uint8_t regAddr);
void MFRC522_ReadStreamReg(uint8_t regAddr, uint8_t *valueStream, uint8_t streamLen);
// PCD to PICC (contactless card) communication function prototypes
uint8_t MFRC522_PICC_Communication(uint8_t command, uint8_t xIRq, uint8_t *sendValStream, uint8_t *sendValLen, uint8_t *receiveValStream, uint8_t *receiveValLen, uint8_t *validBits, uint8_t rxAlign);
uint8_t MFRC522_PICC_TransceiveData(uint8_t *sendValStream, uint8_t *sendValLen, uint8_t *receiveValStream, uint8_t *receiveValLen, uint8_t *validBits, uint8_t rxAlign);
uint8_t MFRC522_PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
uint8_t MFRC522_PICC_Select(UID *uid);
uint8_t MFRC522_PICC_IsNewCardPresent(void);
uint8_t MFRC522_PICC_SelectCard(UID *mfrc522);
void MFRC522_PICC_TestSignal(uint8_t *bufferOut);
PICC_TYPE MFRC522_GetType(uint8_t sak);

#endif
