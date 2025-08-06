#include "init/uart_init.h"
#include <stdio.h>
#include <errno.h>
#include <sys/unistd.h>

UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void)
{
    
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1000000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    
}

/* 用于printf重定向的系统调用 */
int _write(int file, char *ptr, int len)
{
    HAL_StatusTypeDef hstatus;

    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        hstatus = HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    errno = EBADF;
    return -1;
}


void UART_Init(void)
{
    
    MX_USART1_UART_Init();
    setvbuf(stdout, NULL, _IONBF, 0);
}


UART_HandleTypeDef* UART_GetHandle(void)
{
    return &huart1;
}
