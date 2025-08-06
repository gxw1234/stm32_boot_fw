#ifndef HANDLER_SPI_H
#define HANDLER_SPI_H

// 解决math.h问题的宏定义
#define ARM_MATH_CM7
#define __FPU_PRESENT 1U

#include "stm32h7xx_hal.h"
#include "command_handler.h"


#define SPI_INDEX_1     1   // SPI5
#define SPI_INDEX_2     2   // 其他SPI接口...


HAL_StatusTypeDef Handler_SPI_Init(uint8_t spi_index, PSPI_CONFIG pConfig);

HAL_StatusTypeDef Handler_SPI_Transmit(uint8_t spi_index, uint8_t *pTxData, uint8_t *pRxData, uint16_t DataSize, uint32_t Timeout);

// 新增的中断相关函数声明
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void SPI5_IRQHandler(void);

#endif /* HANDLER_SPI_H */