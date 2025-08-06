/**
 * @file handler_gpio.c
 * @brief GPIO处理相关函数实现
 */
#include "handler_gpio.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "tasks/test_iic_send.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_PIN_MAP;

static const GPIO_PIN_MAP gpio_map[] = {
    {NULL, 0},         // 0: 未用
    {GPIOH, GPIO_PIN_4}, // 1: H4
    {GPIOH, GPIO_PIN_5}, // 2: H5
    {GPIOF, GPIO_PIN_11},// 3: F11
    {GPIOH, GPIO_PIN_6}, // 4: H6
    {GPIOH, GPIO_PIN_7}, // 5: H7
    {GPIOH, GPIO_PIN_8}, // 6: H8
    {GPIOE, GPIO_PIN_10},// 7: E10
    {GPIOE, GPIO_PIN_9}, // 10: E9
    {GPIOE, GPIO_PIN_8}, //11: E8
    {GPIOE, GPIO_PIN_7}, //12: E7
    {GPIOB, GPIO_PIN_2}, //13: B2
    {GPIOC, GPIO_PIN_4}, //14: C4
    {GPIOF, GPIO_PIN_15}, //17: F15
    {GPIOF, GPIO_PIN_14}, //18: F14
    {GPIOD, GPIO_PIN_8}, //19: D8
    {GPIOD, GPIO_PIN_9}, //20: D9
    {GPIOC, GPIO_PIN_11}, //21: C11
    {GPIOC, GPIO_PIN_8}, //22: C8
    {GPIOC, GPIO_PIN_9}, //23: C9
    {GPIOA, GPIO_PIN_15}, //24: A15
    {GPIOC, GPIO_PIN_12}, //25: C12
    {GPIOC, GPIO_PIN_10}, //26: C10
    {GPIOD, GPIO_PIN_2}, //27: D2
    {GPIOC, GPIO_PIN_7}, //29: C7
    {GPIOA, GPIO_PIN_8}, //30: A8
    {GPIOA, GPIO_PIN_9}, //31: A9
    {GPIOA, GPIO_PIN_10}, //32: A10
    {GPIOA, GPIO_PIN_11}, //33: A11
    {GPIOA, GPIO_PIN_12}, //34: A12
    {GPIOA, GPIO_PIN_13}, //35: A13
    {GPIOA, GPIO_PIN_14}, //36: A14
    {GPIOB, GPIO_PIN_1},//57:B1
    {GPIOB, GPIO_PIN_4},//162:B4
    {GPIOC, GPIO_PIN_1},//33:C1
    {GPIOB, GPIO_PIN_15},//95:B15
    {GPIOB, GPIO_PIN_3},//161:B3
    {GPIOA, GPIO_PIN_0},//40:A0
    {GPIOB, GPIO_PIN_14},//94:B14
    {GPIOE, GPIO_PIN_0},//169:E0
    {GPIOD, GPIO_PIN_7},//151:D7
    {GPIOG, GPIO_PIN_10},//153:G10
    {GPIOC, GPIO_PIN_4},//54:C4
    {GPIOF, GPIO_PIN_0},//16:F0
    {GPIOF, GPIO_PIN_1},//17:F1
    {GPIOG, GPIO_PIN_9},//152:G9
    {GPIOG, GPIO_PIN_11},//154:G11
    {GPIOB, GPIO_PIN_9},//168:B9
    {GPIOB, GPIO_PIN_8},//167:B8

    //39
    //40
};





//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_SetOutput(uint8_t gpio_index, uint8_t pull_mode) {
    if (gpio_index >= sizeof(gpio_map)/sizeof(gpio_map[0]) || gpio_map[gpio_index].port == NULL) {
        return HAL_ERROR;
    }
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpio_map[gpio_index].pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // 根据pull_mode设置上拉下拉模式
    switch (pull_mode) {
        case 0:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
        case 1:
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            break;
        case 2:
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            break;
        default:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
    }
    HAL_GPIO_Init(gpio_map[gpio_index].port, &GPIO_InitStruct);

    printf("--GPIO set GPIO_MODE_OUTPUT_PP: index=%d, pin=0x%04X, pull_mode=%d\r\n", gpio_index, gpio_map[gpio_index].pin, pull_mode);
    return HAL_OK;
}


//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_SetOpenDrain(uint8_t gpio_index, uint8_t pull_mode) {
    if (gpio_index >= sizeof(gpio_map)/sizeof(gpio_map[0]) || gpio_map[gpio_index].port == NULL) {
        return HAL_ERROR;
    }
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpio_map[gpio_index].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // 根据pull_mode设置上拉下拉模式
    switch (pull_mode) {
        case 0:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
        case 1:
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            break;
        case 2:
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            break;
        default:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
    }
    HAL_GPIO_Init(gpio_map[gpio_index].port, &GPIO_InitStruct);
    printf("--GPIO set GPIO_MODE_OUTPUT_OD: index=%d, pin=0x%04X, pull_mode=%d\r\n", gpio_index, gpio_map[gpio_index].pin, pull_mode);
    return HAL_OK;
}




//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_GPIO_Write(uint8_t gpio_index, uint8_t write_value) {
    if (gpio_index >= sizeof(gpio_map)/sizeof(gpio_map[0]) || gpio_map[gpio_index].port == NULL) {
        return HAL_ERROR;
    }
    GPIO_PinState pin_state = (write_value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(gpio_map[gpio_index].port, gpio_map[gpio_index].pin, pin_state);
    printf("---GPIO write: index=%d, pin=0x%04X, value=0x%02X, state=%d\r\n", gpio_index, gpio_map[gpio_index].pin, write_value, pin_state);
    return HAL_OK;
}

//@param pull_mode 上拉下拉模式：0=无上拉下拉，1=上拉，2=下拉
HAL_StatusTypeDef Handler_scan_GPIO_Write(uint8_t gpio_index, uint8_t write_value) {
    if (gpio_index >= sizeof(gpio_map)/sizeof(gpio_map[0]) || gpio_map[gpio_index].port == NULL) {
        return HAL_ERROR;
    }
    GPIO_PinState pin_state = (write_value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(gpio_map[gpio_index].port, gpio_map[gpio_index].pin, pin_state);
    
    // 如果是下压操作（写入0），设置标志位
    if (write_value == 0) {
        Set_GPIO_Press_Flag();
    }

    
    
    // printf("Handler_scan_GPIO_Write: index=%d, pin=0x%04X, value=0x%02X, state=%d\r\n", gpio_index, gpio_map[gpio_index].pin, write_value, pin_state);
    return HAL_OK;
}







HAL_StatusTypeDef Handler_GPIO_SetInput(uint8_t gpio_index, uint8_t pull_mode) {
    if (gpio_index >= sizeof(gpio_map)/sizeof(gpio_map[0]) || gpio_map[gpio_index].port == NULL) {
        return HAL_ERROR;
    }
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpio_map[gpio_index].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // 根据pull_mode设置上拉下拉模式
    switch (pull_mode) {
        case 0:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
        case 1:
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            break;
        case 2:
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            break;
        default:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            break;
    }
    
    HAL_GPIO_Init(gpio_map[gpio_index].port, &GPIO_InitStruct);
    printf("GPIO set Handler_GPIO_SetInput: index=%d, pin=0x%04X, pull_mode=%d\r\n", gpio_index, gpio_map[gpio_index].pin, pull_mode);
    return HAL_OK;
}
