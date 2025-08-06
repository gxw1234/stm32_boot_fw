#ifndef __HANDLER_RESET_USB3300_STM32_H
#define __HANDLER_RESET_USB3300_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"


void STM32_SoftwareReset(void);


void handler_reset_usb3300_stm32(void);

#ifdef __cplusplus
}
#endif

#endif /* __HANDLER_RESET_USB3300_STM32_H */