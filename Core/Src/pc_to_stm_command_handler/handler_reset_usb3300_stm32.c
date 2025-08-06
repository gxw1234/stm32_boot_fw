#include "main.h"
#include "stm32h7xx_hal.h"
#include "handler_reset_usb3300_stm32.h"
#include "FreeRTOS.h"
#include "task.h"

void STM32_SoftwareReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
    while(1);
}



void handler_reset_usb3300_stm32(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // 设置为低电平
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // 设置为高电平
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // 设置为低电平
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_Delay(100);
    STM32_SoftwareReset();
}