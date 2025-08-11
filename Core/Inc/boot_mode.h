#ifndef BOOT_MODE_H
#define BOOT_MODE_H

#include "stm32h7xx_hal.h"

/* 启动模式定义 */
#define BOOT_MODE_BOOTLOADER    0x5A5A5A5A  // Bootloader模式标识
#define BOOT_MODE_APPLICATION   0xA5A5A5A5  // 应用程序模式标识

/* 函数声明 */
void Boot_Mode_Init(void);
uint32_t Boot_Mode_Get(void);
void Boot_Mode_Set(uint32_t mode);

#endif /* BOOT_MODE_H */
