#include "pc_to_stm_command_handler/handler_iic.h"
#include "main.h"
#include <stdio.h>

#include "usbd_cdc_if.h"


// 声明I2C句柄
// static I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c3_test;


/**
 * @brief 初始化IIC接口
 * 
 * @param iic_index IIC接口索引
 * @param pConfig IIC配置参数
 * @return HAL_StatusTypeDef 初始化状态
 */
HAL_StatusTypeDef Handler_IIC_Init(uint8_t iic_index, PIIC_CONFIG pConfig)
{
    // // 使能 GPIO 时钟
    // __HAL_RCC_GPIOA_CLK_ENABLE();
    // __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // // 配置GPIO
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // // 配置PA8作为I2C3 SCL
    // GPIO_InitStruct.Pin = GPIO_PIN_8;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // // 配置PC9作为I2C3 SDA
    // GPIO_InitStruct.Pin = GPIO_PIN_9;
    // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // // 使能I2C3时钟
    // __HAL_RCC_I2C3_CLK_ENABLE();
    
    // // 配置I2C参数
    // hi2c3_test.Instance = I2C3;
    // hi2c3_test.Init.Timing = 0x10D0A9FF; // 快速模式 (400 KHz)
    // hi2c3_test.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // hi2c3_test.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // hi2c3_test.Init.OwnAddress1 = 0x30; // 从机地址设置为0x30
    // hi2c3_test.Init.OwnAddress2 = 0;
    // hi2c3_test.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    // hi2c3_test.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // hi2c3_test.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    // return HAL_I2C_Init(&hi2c3_test);
    
}

/**
 * @brief  IIC从机发送数据
 * @param  iic_index: IIC索引
 * @param  pData: 要发送的数据指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间
 * @retval HAL状态
 */
HAL_StatusTypeDef Handler_IIC_SlaveWriteBytes(uint8_t iic_index, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    printf("Starting I2C3 slave transmit test...\r\n");
    
    uint8_t test_data[4] = {0x11, 0x22, 0x33, 0x44};
    HAL_StatusTypeDef status;
    
    printf("HAL_I2C_Slave_Transmit will be called, timeout = 1000ms\r\n");
    status = HAL_I2C_Slave_Transmit(&hi2c3_test, test_data, 4, 1000);
    printf("HAL_I2C_Slave_Transmit completed, status = %d\r\n", status);
    
    return status;
}