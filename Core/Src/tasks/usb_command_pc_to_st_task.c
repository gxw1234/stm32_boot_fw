#include "tasks/usb_command_pc_to_st_task.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "pc_to_stm_command_handler/command_handler.h"
#include "FreeRTOS.h"  


#define CMD_END_MARKER      0xA5A5A5A5
#define FRAME_START_MARKER  0x5A5A5A5A 



// 定义大缓冲区大小 - 优化为1KB
#define BIG_BUFFER_SIZE (1024)  
// 将大缓冲区放到主RAM区域，避免DTCMRAM溢出
static uint8_t big_buffer[BIG_BUFFER_SIZE] ;

typedef enum {
    WAITING_FOR_HEADER,     // 等待帧头
    RECEIVING_DATA          // 正在接收数据
} frame_state_t;

// 帧处理变量
static frame_state_t frame_state = WAITING_FOR_HEADER;
static uint32_t expected_total_length = 0;  // 期望的总数据长度
static uint32_t received_length = 0;        // 已接收的数据长度
static uint32_t frame_type = 0;             // 帧类型
static uint32_t cmd_id = 0;                 // 命令ID
static uint8_t is_queue_write_cmd = 0;      // 是否为CMD_QUEUE_WRITE命令
static int8_t image_buffer_index = -1;      // 图像缓冲区索引
QueueHandle_t usbMessageQueueHandle = NULL;


static void send_queue_write_response(uint8_t status)
{
    typedef struct {
        GENERIC_CMD_HEADER header;
        uint8_t status;  // 状态数据
    } Queue_Write_Response;
    Queue_Write_Response response;
    response.header.protocol_type = PROTOCOL_SPI;
    response.header.cmd_id = CMD_QUEUE_WRITE;
    response.header.device_index = 0;
    response.header.param_count = 0;
    response.header.data_len = sizeof(uint8_t);
    response.header.total_packets = sizeof(Queue_Write_Response);
    response.status = status;
    USB_Sender((uint8_t*)&response, sizeof(response));
}


void usb_command_pc_to_st_task(void *argument)
{
    memset(big_buffer, 0, BIG_BUFFER_SIZE);
    usbMessageQueueHandle = xQueueCreate(100, sizeof(USB_Data_TypeDef));
    if (usbMessageQueueHandle == NULL) {
        printf(" Queue create failed!\r\n");
        vTaskDelete(NULL);
        return;
    }
    USB_Data_TypeDef usbData;
    while (1) {
        if (xQueueReceive(usbMessageQueueHandle, &usbData, portMAX_DELAY) == pdPASS) {
            //判断是否找到帧头，如果有帧头就解析协议头，没有帧头就继续接收数据
            //接收数据的时候分两种情况，一种是CMD_QUEUE_WRITE，另一种是其他命令
            //CMD_QUEUE_WRITE: 的时候是零拷贝，直接提交到队列
            //其他命令： 放在big_buffer
            if (frame_state == WAITING_FOR_HEADER) {
                if (usbData.Length >= 4) {
                    uint32_t *header = (uint32_t*)usbData.Buf;
                    if (*header == FRAME_START_MARKER) {
                        if (usbData.Length >= 12) {  
                            // 帧头4 + 协议头8字节
                            //=================解析协议头========================
                            // 协议头格式：[protocol_type:1][cmd_id:1][reserved:2][param_count:1][reserved:1][total_packets:2]
                            //======================解析协议头===========================
                            // 协议类型    命令ID  总数据包长
                            uint8_t protocol_type = usbData.Buf[4];          
                            uint8_t cmd_id_byte = usbData.Buf[5];    
                            uint16_t total_packets;
                            memcpy(&total_packets, usbData.Buf + 10, 2);         
                            frame_type = protocol_type;
                            cmd_id = cmd_id_byte;
                            expected_total_length = 4 + total_packets + 4;       // 帧头 + 数据 + 帧尾
                            memcpy(big_buffer, usbData.Buf, usbData.Length);
                            received_length = usbData.Length;
                            if (received_length >= expected_total_length) {
                
                                uint32_t cmd_length = received_length - 8; 
                                Process_Command(big_buffer + 4, &cmd_length); 
                            } else {
                                frame_state = RECEIVING_DATA;
                            }
                        }
                    }
                }
            } 
            else if (frame_state == RECEIVING_DATA) {
                    if (is_queue_write_cmd) {
                    } else {
                        memcpy(big_buffer + received_length, usbData.Buf, usbData.Length);
                    }
                    received_length += usbData.Length;
                    if (received_length >= expected_total_length) {
                       
                        uint32_t cmd_length = received_length - 8; 
                        Process_Command(big_buffer + 4, &cmd_length); 
                        frame_state = WAITING_FOR_HEADER;
                        received_length = 0;
                        is_queue_write_cmd = 0;
                        image_buffer_index = -1;
                    }
            }
        }
    }
}
