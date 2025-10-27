#include "boot_mode.h"

/* 使用RTC备份寄存器存储启动模式，重启和掉电后都能保持 */
#define BOOT_MODE_RTC_BACKUP_REG_INDEX    0  // 使用备份寄存器0

/* RTC备份寄存器直接访问宏 */
#define RTC_BACKUP_REG(index)   (&(RTC->BKP0R) + (index))

/**
  * @brief  初始化启动模式
  * @retval None
  */
void Boot_Mode_Init(void)
{
    /* PWR模块在STM32H7中通常默认启用，无需手动启用时钟 */
    
    /* 启用对备份域的访问 */
    PWR->CR1 |= PWR_CR1_DBP;
    
    /* 启用RTC时钟 - 直接操作RCC寄存器 */
    RCC->BDCR |= RCC_BDCR_RTCEN;
    
    /* 等待备份域访问生效 */
    while ((PWR->CR1 & PWR_CR1_DBP) == 0);
}

/**
  * @brief  获取启动模式
  * @retval 启动模式值
  */
uint32_t Boot_Mode_Get(void)
{
    /* 确保备份域访问已启用 */
    if ((PWR->CR1 & PWR_CR1_DBP) == 0)
    {
        PWR->CR1 |= PWR_CR1_DBP;
        while ((PWR->CR1 & PWR_CR1_DBP) == 0);
    }
    
    /* 直接读取RTC备份寄存器 */
    return *((volatile uint32_t*)RTC_BACKUP_REG(BOOT_MODE_RTC_BACKUP_REG_INDEX));
}

/**
  * @brief  设置启动模式
  * @param  mode: 启动模式值
  * @retval None
  */
void Boot_Mode_Set(uint32_t mode)
{
    /* 确保备份域访问已启用 */
    if ((PWR->CR1 & PWR_CR1_DBP) == 0)
    {
        PWR->CR1 |= PWR_CR1_DBP;
        while ((PWR->CR1 & PWR_CR1_DBP) == 0);
    }
    
    /* 直接写入RTC备份寄存器 */
    *((volatile uint32_t*)RTC_BACKUP_REG(BOOT_MODE_RTC_BACKUP_REG_INDEX)) = mode;
}
