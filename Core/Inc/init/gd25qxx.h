#ifndef __GD25QXX_H
#define __GD25QXX_H
#include "stm32h7xx_hal.h"

void GD25QXX_WriteByte(uint32_t addr, uint8_t data);
void GD25QXX_WriteBytes(uint32_t addr, const uint8_t *data, uint32_t datalen);
uint8_t GD25QXX_ReadByte(uint32_t addr);
void GD25QXX_EraseSector(uint32_t addr);
void GD25QXX_ReadBytes(uint32_t addr, uint8_t *data, uint32_t datalen);

/* XIP相关函数 */
HAL_StatusTypeDef GD25QXX_EnableXIP(void);
HAL_StatusTypeDef GD25QXX_DisableXIP(void);
uint8_t GD25QXX_CheckXIPStatus(void);
void GD25QXX_TestXIP(void);
void GD25QXX_TestAlternativeXIPAddresses(void);
HAL_StatusTypeDef GD25QXX_ReadID(uint8_t *id);

#endif