#include "command_handler.h"
#include "main.h"
#include "../tasks/MP8865_task.h"
#include "tasks/ads1220_task.h"  
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





static void Process_Power_SetVoltage(uint8_t channel, uint16_t voltage_mv) {
    // 将毫伏转换为伏特
    float voltage_v = voltage_mv / 1000.0f;
    
    // printf("Power Set Voltage: Channel=%d, Target=%.3fV (%dmV)\r\n", 
    //        channel, voltage_v, voltage_mv);
    
    HAL_StatusTypeDef status = MP8865_SetVoltageV(voltage_v);
    
    // if (status == HAL_OK) {
    //     printf("MP8865 Set voltage success: %.3fV\r\n", voltage_v);
    // } else {
    //     printf("MP8865 Set voltage failed, error code: %d\r\n", status);
    // }
}


/**
 * @brief 处理开始读取电流命令
 * 
 * @param device_index 设备索引
 */
static void Process_Power_StartCurrentReading(uint8_t device_index) {
    printf("Receive Command:POWER_CMD_START_CURRENT_READING,device_index:%d\r\n", device_index);
    Enable_Current_Data_Sending();
}

/**
 * @brief 处理停止读取电流命令
 * 
 * @param device_index 设备索引
 */
static void Process_Power_StopCurrentReading(uint8_t device_index) {
    printf("Receive Command:POWER_CMD_STOP_CURRENT_READING,device_index:%d\r\n", device_index);
    Disable_Current_Data_Sending();
}

/**
 * @brief 处理IIC初始化命令
 * 
 * @param iic_index IIC索引
 * @param pConfig IIC配置结构体指针
 */
static void Process_IIC_Init(uint8_t iic_index, PIIC_CONFIG pConfig) {

    printf("IIC init: %d, ClockSpeed=%lu, OwnAddr=0x%04X, Master=%d, AddrBits=%d, EnablePu=%d\r\n", 
            iic_index, pConfig->ClockSpeedHz, pConfig->OwnAddr, pConfig->Master, pConfig->AddrBits, pConfig->EnablePu);
    
    // 调用handler_iic中的初始化函数
    HAL_StatusTypeDef status = Handler_IIC_Init(iic_index, pConfig);
    
    if (status == HAL_OK) {

        printf("IIC init SUCCESSFUL: %d\r\n", iic_index);
    } else {

        printf("IIC init fail: %d, error: %d\r\n", iic_index, status);
    }
    


}


static void Process_IIC_SlaveWrite(uint8_t iic_index, uint8_t* pData, uint16_t data_len, uint32_t timeout) {

    HAL_StatusTypeDef status = Handler_IIC_SlaveWriteBytes(iic_index, pData, data_len, timeout);
    
}


static int Process_Power_ReadCurrentData(uint8_t channel, uint8_t* response_buf, int max_len) {

    

    float current_value = 123.456f;
    int response_len = 0;
    

    if (max_len >= sizeof(GENERIC_CMD_HEADER) + sizeof(float)) {
        GENERIC_CMD_HEADER* response_header = (GENERIC_CMD_HEADER*)response_buf;
        response_header->protocol_type = PROTOCOL_POWER;
        response_header->cmd_id = POWER_CMD_READ_CURRENT_DATA;
        response_header->device_index = channel;
        response_header->param_count = 0;
        response_header->data_len = sizeof(float); 
        memcpy(response_buf + sizeof(GENERIC_CMD_HEADER), &current_value, sizeof(float));
        response_len = sizeof(GENERIC_CMD_HEADER) + sizeof(float);
        printf("Current data: %.3f\r\n", current_value);
    }
    
    return response_len;
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
