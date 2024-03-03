#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "stm32f10x.h"

void SPI_InitConfig(void);
void SPI_TransmitByte(SPI_TypeDef *SPIx, uint8_t byte);
uint8_t SPI_ReceiveByte(SPI_TypeDef *SPIx, uint8_t *byte, uint16_t timeout);

#endif
