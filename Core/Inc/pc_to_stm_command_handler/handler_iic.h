#ifndef __HANDLER_IIC_H__
#define __HANDLER_IIC_H__

#include "main.h"

// IIC配置结构体
typedef struct _IIC_CONFIG {
    uint32_t    ClockSpeedHz;   // IIC时钟频率:单位为Hz
    uint16_t    OwnAddr;        // STM32为从机时自己的地址
    uint8_t     Master;         // 主从选择控制:0-从机，1-主机
    uint8_t     AddrBits;       // 从机地址模式，7-7bit模式，10-10bit模式
    uint8_t     EnablePu;       // 使能引脚芯片内部上拉电阻
} IIC_CONFIG, *PIIC_CONFIG;

// IIC句柄索引定义
#define IIC_INDEX_0    0  // I2C3

// 初始化IIC接口
HAL_StatusTypeDef Handler_IIC_Init(uint8_t iic_index, PIIC_CONFIG pConfig);

// IIC从机写数据
HAL_StatusTypeDef Handler_IIC_SlaveWriteBytes(uint8_t iic_index, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif // __HANDLER_IIC_H__