/**
  ******************************************************************************
  * @file           : gpio_init.h
  * @brief          : GPIO initialization function declarations
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_INIT_H__
#define __GPIO_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Function prototypes -------------------------------------------------------*/
void MX_GPIO_Init(void);
void MX_QUADSPI_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __GPIO_INIT_H__ */
