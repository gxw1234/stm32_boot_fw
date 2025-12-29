#include "command_handler.h"
#include "main.h"
#include "pc_to_stm_command_handler/handler_iic.h" 
#include "handler_gpio.h" 
#include "pc_to_stm_command_handler/handler_reset_usb3300_stm32.h" 
#include "usbd_cdc_if.h"

int Get_Parameter(uint8_t* buffer, int pos, void* data, uint16_t max_len) {
    PARAM_HEADER header;
    memcpy(&header, buffer + pos, sizeof(PARAM_HEADER));
    pos += sizeof(PARAM_HEADER);
    if (header.param_len > max_len) {
        return -1;
    }
    memcpy(data, buffer + pos, header.param_len);
    pos += header.param_len;
    return pos;
}







int8_t Process_Command(uint8_t* Buf, uint32_t *Len) {
   
    if (*Len >= sizeof(GENERIC_CMD_HEADER)) {
        GENERIC_CMD_HEADER* header = (GENERIC_CMD_HEADER*)Buf;
        // printf("SPI Init Command: Device=%d, ParamCount=%d, ProtocolType=%d, CmdId=%d\r\n", 
        // header->device_index, header->param_count, header->protocol_type, header->cmd_id);
        switch (header->protocol_type) {
            case PROTOCOL_SPI: {
                switch (header->cmd_id) {
                    case CMD_INIT: {
                        break;
                    }
                    case CMD_TRANSFER:
                        {
                            printf("Unknown SPI command ID: 0x%02X\r\n", header->cmd_id);
                        }
                        break;
                    default: {
                        printf("Unknown SPI command ID: 0x%02X\r\n", header->cmd_id);
                        break;
                    }
                }
                break;
            }
            case PROTOCOL_BOOTLOADER_WRITE_BYTES: {
                switch (header->cmd_id) {
                    case BOOTLOADER_WRITE_BYTES: {
                        printf("Receive Command:BOOTLOADER_WRITE_BYTES,device_index:%d\r\n", header->device_index);
                        break;
                    }
                    case BOOTLOADER_SWITCH_RUN: {
                        printf("Receive Command:BOOTLOADER_SWITCH_RUN,device_index:%d\r\n", header->device_index);
                        break;
                    }
                    case BOOTLOADER_SWITCH_BOOT: {
                        printf("Receive Command:BOOTLOADER_SWITCH_BOOT,device_index:%d\r\n", header->device_index);
                        break;
                    }
                    
                    default: {
                        printf("Unknown BOOTLOADER_WRITE_BYTES command ID: 0x%02X\r\n", header->cmd_id);
                        break;
                    }
                }
                break;
            }   
            default: {
                printf("Unknown protocol type: 0x%02X\r\n", header->protocol_type);
                break;
            }
        }
    } else {
        // 数据过短，直接打印原始数据

        printf("Data too short, length: %d\r\n", *Len);
    }
    
    return 0;
}
