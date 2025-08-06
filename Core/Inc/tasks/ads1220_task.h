#ifndef ADS1220_TASK_H
#define ADS1220_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"

extern SPI_HandleTypeDef hspi4;

void ADS1220_Task(void *argument);
void ADS1220_Init(void);
uint8_t SPI_TransmitReceive(uint8_t data);
void SPI_Transmit(uint8_t data);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


void Enable_Current_Data_Sending(void);  
void Disable_Current_Data_Sending(void); 


static float Convert_ADC_To_Voltage(uint32_t adc_value);
#endif /* ADS1220_TASK_H */