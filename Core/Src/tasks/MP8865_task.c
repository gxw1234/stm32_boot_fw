#include "tasks/MP8865_task.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define DATA_SIZE 40960  // 恢复到原始工作值
uint8_t myData[DATA_SIZE];

I2C_HandleTypeDef hi2c4;



#define I2C_DEVICE_ADDR    0x60    
#define I2C_REG_ADDR_01       0x01   
#define I2C_REG_ADDR_00       0x00    
#define I2C_DATA_VALUE     0b11111111   
#define MP8865_AMPLIFIER_GAIN 1.72f
// #define MP8865_AMPLIFIER_GAIN 5.11f

uint8_t MP8865_VoltageToRegValue(float voltage_v)
{

    if (voltage_v < 0.60f) voltage_v = 0.60f;  // 最小电压限制
    if (voltage_v > 1.87f) voltage_v = 1.87f;  // 最大电压限制（7位最大值127: 0.60 + 1.27 = 1.87V）
    
    uint8_t voltage_bits = (uint8_t)((voltage_v - 0.60f) * 100.0f);
    if (voltage_bits > 127) voltage_bits = 127;
    // 添加固定的控制位 (最高位为1)
    uint8_t reg_value = 0x80 | voltage_bits;  // 0x80 = 10000000
    return reg_value;
}


HAL_StatusTypeDef MP8865_SetVoltageV(float voltage_v)
{

    //GAIN=1.72 电压范围1.034V--3.22V

    float actual_set_voltage = voltage_v / MP8865_AMPLIFIER_GAIN;

    uint8_t reg_value = MP8865_VoltageToRegValue(actual_set_voltage);
    return MP8865_SetVoltage(reg_value);
}


HAL_StatusTypeDef MP8865_SetVoltage(uint8_t voltage_value)
{
    

    HAL_StatusTypeDef status;
    uint8_t data[2];
    data[0] = 0x01;
    data[1] = 0xE0; // 11100000 - 启用GO_BIT
    status = HAL_I2C_Master_Transmit(&hi2c4, I2C_DEVICE_ADDR << 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status;
    data[0] = 0x00;
    data[1] = voltage_value;
    status = HAL_I2C_Master_Transmit(&hi2c4, I2C_DEVICE_ADDR << 1, data, 2, HAL_MAX_DELAY);
    return status;

}

void MP8865_Init(void)
{
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x00707CBB;
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK)
    {
      Error_Handler();
    }
  
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
    {
      Error_Handler();
    }

}



void MP8865_Task(void *pvParameters)
{
    MP8865_Init();
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    HAL_StatusTypeDef status = MP8865_SetVoltageV(2.00f);  
    if (status != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
