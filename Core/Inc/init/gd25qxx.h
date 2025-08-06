#ifndef __GD25QXX_H
#define __GD25QXX_H
#include "stm32h7xx_hal.h"

void GD25QXX_WriteByte(uint32_t addr, uint8_t data);
void GD25QXX_WriteBytes(uint32_t addr, const uint8_t *data, uint32_t datalen);
uint8_t GD25QXX_ReadByte(uint32_t addr);
void GD25QXX_EraseSector(uint32_t addr);
void GD25QXX_ReadBytes(uint32_t addr, uint8_t *data, uint32_t datalen);

#endif 