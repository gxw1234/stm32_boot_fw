#include "boot_mode.h"

/* 使用DTCMRAM的一个固定地址作为启动模式寄存器 */
#define BOOT_MODE_REG_ADDR  0x20000000  // DTCMRAM起始地址

/**
  * @brief  初始化启动模式
  * @retval None
  */
void Boot_Mode_Init(void)
{
    /* DTCMRAM在系统复位后会保持数据，无需特殊初始化 */
}

/**
  * @brief  获取启动模式
  * @retval 启动模式值
  */
uint32_t Boot_Mode_Get(void)
{
    return *(volatile uint32_t*)BOOT_MODE_REG_ADDR;
}

/**
  * @brief  设置启动模式
  * @param  mode: 启动模式值
  * @retval None
  */
void Boot_Mode_Set(uint32_t mode)
{
    *(volatile uint32_t*)BOOT_MODE_REG_ADDR = mode;
}
