cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_PWR_LDO_SUPPLY 
	USE_HAL_DRIVER 
	STM32H750xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Core/Inc/config
    ../../Core/Inc/pc_to_stm_command_handler
    ../../Core/Inc/Font_library
    ../../Core/Inc/tasks
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Drivers/STM32H7xx_HAL_Driver/Inc
    ../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/main.c
    ../../Core/Src/tasks/lcd_task.c
    ../../Core/Src/tasks/ads1220_task.c
    ../../Core/Src/tasks/MP8865_task.c
    ../../Core/Src/tasks/test_iic_send.c
    ../../Core/Src/tasks/usb_command_pc_to_st_task.c
    ../../Core/Src/init/uart_init.c
    ../../Core/Src/init/gpio_init.c
    ../../Core/Src/init/gd25qxx.c
    ../../Core/Src/init/qspi_init.c
    ../../Core/Src/tasks/debug_task.c
    ../../Core/Src/tasks/image_queue_task.c
    ../../Core/Src/Font_library/fonts.c
    ../../Core/Src/config/freertos.c
    ../../Core/Src/config/stm32h7xx_it.c
    ../../Core/Src/config/stm32h7xx_hal_msp.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/App/usbd_cdc_if.c
    ../../USB_DEVICE/Target/usbd_conf.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    ../../Core/Src/config/system_stm32h7xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/config/sysmem.c
    ../../Core/Src/config/syscalls.c
    ../../startup_stm32h750xx.s
    ../../Core/Src/pc_to_stm_command_handler/command_handler.c
    ../../Core/Src/pc_to_stm_command_handler/handler_spi.c
    ../../Core/Src/pc_to_stm_command_handler/handler_iic.c
    ../../Core/Src/pc_to_stm_command_handler/handler_reset_usb3300_stm32.c
    ../../Core/Src/pc_to_stm_command_handler/handler_gpio.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_qspi.c
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


