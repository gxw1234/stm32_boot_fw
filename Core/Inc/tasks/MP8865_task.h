#ifndef MP8865_TASK_H
#define MP8865_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

void MP8865_Task(void *pvParameters);

void MP8865_Init(void);

HAL_StatusTypeDef MP8865_SetVoltage(uint8_t voltage_value);
HAL_StatusTypeDef MP8865_SetVoltageV(float voltage_v);
uint8_t MP8865_VoltageToRegValue(float voltage_v);

#endif /* MP8865_TASK_H */