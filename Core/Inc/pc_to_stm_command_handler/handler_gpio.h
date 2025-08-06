/**
 * @file handler_gpio.h
 * @brief GPIO处理相关函数头文件
 */
#ifndef HANDLER_GPIO_H
#define HANDLER_GPIO_H

#include "main.h"
#include "stm32h7xx_hal.h"


//设置为输出模式
//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_SetOutput(uint8_t gpio_index, uint8_t pull_mode);


//设置为开漏模式
//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_SetOpenDrain(uint8_t gpio_index, uint8_t pull_mode);


//普通的GPIO写入

HAL_StatusTypeDef Handler_GPIO_Write(uint8_t gpio_index, uint8_t write_value);

//扫描模式入GPIO写入
HAL_StatusTypeDef Handler_scan_GPIO_Write(uint8_t gpio_index, uint8_t write_value);



//设置为输入模式
//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_SetInput(uint8_t gpio_index, uint8_t pull_mode);



#endif // HANDLER_GPIO_H
