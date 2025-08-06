#ifndef TEST_USB_SEND_H
#define TEST_USB_SEND_H

#include <stdint.h>
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

/* 任务相关定义 */
extern TaskHandle_t UsbSendTaskHandle;

/* 函数声明 */

void IIC_interruption_Task(void *argument);

HAL_StatusTypeDef Test_I2C3_Slave_Init(void);

void Set_GPIO_Press_Flag(void);

#endif /* TEST_USB_SEND_H */