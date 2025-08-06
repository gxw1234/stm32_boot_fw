#include "tasks/usb_command_pc_to_st_task.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "pc_to_stm_command_handler/command_handler.h"
#include "FreeRTOS.h"  
#include "tasks/image_queue_task.h"  

// 定义结束标记
#define CMD_END_MARKER      0xA5A5A5A5
// 定义帧头标识符
#define FRAME_START_MARKER  0x5A5A5A5A 



// 定义大缓冲区大小 - 优化为1KB
#define BIG_BUFFER_SIZE (1024*24*2)  
// 将大缓冲区放到主RAM区域，避免DTCMRAM溢出
static uint8_t big_buffer[BIG_BUFFER_SIZE] __attribute__((section(".ram_buffers")));

// 简单的帧处理状态
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


// 发送CMD_QUEUE_WRITE
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
    // 100*512=51200/96*240=2   可以存两张图
    usbMessageQueueHandle = xQueueCreate(100, sizeof(USB_Data_TypeDef));
    if (usbMessageQueueHandle == NULL) {
        printf("[USB] Queue create failed!\r\n");
        vTaskDelete(NULL);
        return;
    }
    USB_Data_TypeDef usbData;
    while (1) {
        if (xQueueReceive(usbMessageQueueHandle, &usbData, portMAX_DELAY) == pdPASS) {
            if (frame_state == WAITING_FOR_HEADER) {
                // 查找帧头  
                if (usbData.Length >= 4) {
                    uint32_t *header = (uint32_t*)usbData.Buf;
                    //找到帧头要处理协议
                    if (*header == FRAME_START_MARKER) {
                        // 解析协议头信息
                        if (usbData.Length >= 12) {  // 帧头4 + 协议头8字节

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
                            is_queue_write_cmd = (protocol_type == PROTOCOL_SPI && cmd_id_byte == CMD_QUEUE_WRITE);
                            if (is_queue_write_cmd) {
                                // CMD_QUEUE_WRITE: 分配图像缓冲区
                                
                                // 用零拷贝方式提交图像数据

                                //收到图像以后直接提交到队列
                                // 计算纯图像数据大小：总长度 - 帧头(4) - 协议头(sizeof(GENERIC_CMD_HEADER)) - 帧尾(4)
                                image_buffer_index = ImageQueue_AllocateBuffer();
                                if (image_buffer_index >= 0) {
                                    uint8_t* image_buffer = ImageQueue_GetBufferPtr(image_buffer_index);
                                    // 跳过帧头(4字节)和协议头(8字节)，只拷贝纯图像数据
                                    uint32_t data_offset = 4 + sizeof(GENERIC_CMD_HEADER);  // 12字节
                                    uint32_t copy_length = (usbData.Length > data_offset) ? (usbData.Length - data_offset) : 0;
                                    if (copy_length > 0) {
                                        memcpy(image_buffer, usbData.Buf + data_offset, copy_length);
                                    }
                                } else {
                                    frame_state = WAITING_FOR_HEADER;
                                    received_length = 0;
                                    continue;
                                }
                            } else {

                                memcpy(big_buffer, usbData.Buf, usbData.Length);
                            }
                            received_length = usbData.Length;
                            if (received_length >= expected_total_length) {
                                if (is_queue_write_cmd) {
                                    // CMD_QUEUE_WRITE: 只提交纯图像数据（去掉帧头和帧尾）
                                    if (image_buffer_index >= 0) {
                                        // 用零拷贝方式提交图像数据
                                        //收到图像以后直接提交到队列
                                        // 计算纯图像数据大小：总长度 - 帧头(4) - 协议头(sizeof(GENERIC_CMD_HEADER)) - 帧尾(4)
                                        uint32_t image_data_size = received_length - 4 - sizeof(GENERIC_CMD_HEADER) - 4;
                                        HAL_StatusTypeDef result = ImageQueue_CommitBuffer(image_buffer_index, image_data_size);
                                        if (result == HAL_OK) {
                                            send_queue_write_response(0x00);  // 0x00=数据完整且队列未满
                                        } else {
                                            send_queue_write_response((result == HAL_BUSY) ? 0x02 : 0xFF);  // 0x02=队列满, 0xFF=其他错误
                                        }
                                    }
                                } else {
                                    uint32_t cmd_length = received_length - 8; // 去掉帧头和帧尾
                                    Process_Command(big_buffer + 4, &cmd_length); // 跳过帧头
                                }
                                frame_state = WAITING_FOR_HEADER;
                                received_length = 0;
                                is_queue_write_cmd = 0;
                                image_buffer_index = -1;
                            } else {
                                frame_state = RECEIVING_DATA;
                            }
                        }
                    }
                }
            } 
            else if (frame_state == RECEIVING_DATA) {
                // 继续接收数据
                    if (is_queue_write_cmd) {
                        // CMD_QUEUE_WRITE: 继续写入图像缓冲区
                        if (image_buffer_index >= 0) {
                            uint8_t* image_buffer = ImageQueue_GetBufferPtr(image_buffer_index);
                            uint32_t data_offset = 4 + sizeof(GENERIC_CMD_HEADER);
                            uint32_t image_data_received = received_length - data_offset;
                            memcpy(image_buffer + image_data_received, usbData.Buf, usbData.Length);
                        }
                    } else {
                        // 其他命令: 继续写入big_buffer
                        memcpy(big_buffer + received_length, usbData.Buf, usbData.Length);
                    }
                    received_length += usbData.Length;
                    // 检查是否接收完成
                    if (received_length >= expected_total_length) {
                        
                        // 根据命令类型进行不同处理
                        if (is_queue_write_cmd) {
                            // CMD_QUEUE_WRITE: 只提交纯图像数据（去掉帧头和帧尾）
                            if (image_buffer_index >= 0) {
                                // 计算纯图像数据大小：总长度 - 帧头(4) - 协议头(sizeof(GENERIC_CMD_HEADER)) - 帧尾(4)
                                uint32_t image_data_size = received_length - 4 - sizeof(GENERIC_CMD_HEADER) - 4;
                                HAL_StatusTypeDef result = ImageQueue_CommitBuffer(image_buffer_index, image_data_size);
                                if (result == HAL_OK) {
                                    send_queue_write_response(0x00);  // 0x00=数据完整且队列未满
                                } else {
                                    send_queue_write_response((result == HAL_BUSY) ? 0x02 : 0xFF);  // 0x02=队列满, 0xFF=其他错误
                                }
                            }
                        } else {

                            uint32_t cmd_length = received_length - 8; 
                            Process_Command(big_buffer + 4, &cmd_length); 
                        }
                        frame_state = WAITING_FOR_HEADER;
                        received_length = 0;
                        is_queue_write_cmd = 0;
                        image_buffer_index = -1;
                    }
            }
        }
    }
}
