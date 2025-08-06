#include "tasks/debug_task.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "pc_to_stm_command_handler/command_handler.h"
#include <string.h>

void debug_task(void *argument)
{
    // 定义SPI协议响应数据包
    typedef struct {
        GENERIC_CMD_HEADER header;
        uint8_t spi_data[20];  // SPI数据部分
    } SPI_Response_Packet;
    
    SPI_Response_Packet response;
    uint8_t ret = 0;

    // 设置协议头
    response.header.protocol_type = PROTOCOL_SPI;        // SPI协议
    response.header.cmd_id = CMD_READ;                   // 读数据命令
    response.header.device_index = 1;                    // SPI1_CS0
    response.header.param_count = 0;                     // 无参数
    response.header.data_len = 20;                       // 数据长度20字节
    response.header.total_packets = sizeof(SPI_Response_Packet);  // 总包大小

    for (int i = 0; i < 20; i++) {
        response.spi_data[i] = i + 1;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1)
    {
        ret = USB_Sender((uint8_t*)&response, sizeof(response));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}   