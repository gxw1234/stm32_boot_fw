#include "tasks/usb_command_pc_to_st_task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pc_to_stm_command_handler/command_handler.h"
#include <string.h>
#include "init/gd25qxx.h"
#include "boot_mode.h"
#include "pc_to_stm_command_handler/handler_reset_usb3300_stm32.h"

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
static uint8_t is_boot_write_cmd = 0;      // 是否为BOOTLOADER_WRITE_BYTES命令
static int8_t image_buffer_index = -1;      // 图像缓冲区索引

// QSPI Flash地址管理
#define QSPI_FIRMWARE_START_ADDR    0x020000    // 固件起始地址 (128KB偏移)
static uint32_t qspi_current_addr = QSPI_FIRMWARE_START_ADDR;  // 当前写入地址
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
    usbMessageQueueHandle = xQueueCreate(10, sizeof(USB_Data_TypeDef));
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
                            expected_total_length = 4 + total_packets + 4;      
                            
                            received_length = usbData.Length;
                            if (received_length >= expected_total_length) {
                                switch (protocol_type) {
                                    case PROTOCOL_BOOTLOADER_WRITE_BYTES:
                                        switch (cmd_id_byte) {
                                            case BOOTLOADER_START_WRITE:
                                                qspi_current_addr = QSPI_FIRMWARE_START_ADDR;
                                                printf("Bootloader start write, reset QSPI address to 0x%08lX\n", qspi_current_addr);
                                                break;
                                            case BOOTLOADER_WRITE_BYTES:
                                                qspi_current_addr = QSPI_FIRMWARE_START_ADDR; 
                                                uint32_t data_offset = 12;  
                                                uint32_t data_length = usbData.Length - data_offset;
                                                if (received_length >= expected_total_length) {
                                                    data_length -= 4;  
                                                }
                                                if (data_length > 0) {
                                                    GD25QXX_WriteBytes(qspi_current_addr, usbData.Buf + data_offset, data_length);
                                                    qspi_current_addr += data_length;  // 更新地址
                                                    send_queue_write_response(0);
                                                }
                                                is_boot_write_cmd = 1;
                                                break;
                                            case BOOTLOADER_SWITCH_RUN:
                                                // 切换到RUN模式
                                                Boot_Mode_Set(BOOT_MODE_APPLICATION);
                                                printf("Boot mode switched to APPLICATION\n");
                                                break;
                                            case BOOTLOADER_SWITCH_BOOT:
                                                // 切换到BOOT模式
                                                Boot_Mode_Set(BOOT_MODE_BOOTLOADER);
                                                printf("Boot mode switched to BOOTLOADER\n");
                                                break;
                                            case BOOTLOADER_RESET:
                                                // 复位命令
                                                printf("Received reset command, resetting system...\n");
                                                handler_reset_usb3300_stm32();
                                                break;
                                            default:
                                                memcpy(big_buffer, usbData.Buf, usbData.Length);
                                                uint32_t cmd_length = received_length - 8; 
                                                Process_Command(big_buffer + 4, &cmd_length);
                                                is_boot_write_cmd = 0;
                                                break;
                                        }
                                        break;
                                    default:
                                        memcpy(big_buffer, usbData.Buf, usbData.Length);
                                        uint32_t cmd_length = received_length - 8; 
                                        Process_Command(big_buffer + 4, &cmd_length);
                                        is_boot_write_cmd = 0;
                                        break;
                                }

                                frame_state = WAITING_FOR_HEADER;
                                received_length = 0;
                                is_boot_write_cmd = 0;
                            } else {
                                frame_state = RECEIVING_DATA;
                            }
                        }
                    }
                }
            } 
            else if (frame_state == RECEIVING_DATA) {
                    received_length += usbData.Length;
                    if (received_length >= expected_total_length) {
                        if (is_boot_write_cmd) {
                           
                            uint32_t data_length = usbData.Length - 4;  
                            if (data_length > 0) {
                                GD25QXX_WriteBytes(qspi_current_addr, usbData.Buf, data_length);
                                qspi_current_addr += data_length;  
                            }
                        }
                        else
                        {
                            uint32_t cmd_length = received_length - 8; 
                            Process_Command(big_buffer + 4, &cmd_length); 
                        }
                        frame_state = WAITING_FOR_HEADER;
                        received_length = 0;
                        is_boot_write_cmd = 0;
                        image_buffer_index = -1;
                    }
                    else
                    {
                        if (is_boot_write_cmd) {
                            // 中间包：追加写入所有数据
                            GD25QXX_WriteBytes(qspi_current_addr, usbData.Buf, usbData.Length);
                            qspi_current_addr += usbData.Length;  // 更新地址
                        } else {
                            memcpy(big_buffer + received_length, usbData.Buf, usbData.Length);
                        }


                        
                    }
            }
        }
    }
}
