#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// 协议类型定义
#define PROTOCOL_SPI        0x01    // SPI协议
#define PROTOCOL_IIC        0x02    // IIC协议
#define PROTOCOL_UART       0x03    // UART协议
#define PROTOCOL_GPIO       0x04    // GPIO协议
#define PROTOCOL_POWER      0x05    // 电源协议
#define PROTOCOL_RESETSTM32      0x06    // 复位STM32

// 通用命令ID定义
#define CMD_INIT            0x01    // 初始化命令
#define CMD_WRITE           0x02    // 写数据命令
#define CMD_READ            0x03    // 读数据命令
#define CMD_TRANSFER        0x04    // 读写数据命令



//----SPI------------
#define CMD_QUEUE_STATUS    0x05    // 队列状态查询命令
#define CMD_QUEUE_START    0x06    // 队列启动命令    
#define CMD_QUEUE_STOP    0x07    // 队列停止命令
#define CMD_QUEUE_WRITE           0x08    // 队列写数据命令

//----结束符------------
#define CMD_END_MARKER      0xA5A5A5A5 // 命令包结束符



//---------------GPIO------------------
#define GPIO_DIR_OUTPUT  0x01    // 输出模式
#define GPIO_DIR_OUTPUT_OD  0x02    // 输出开漏
#define GPIO_DIR_INPUT   0x00    // 输入模式
#define GPIO_DIR_WRITE   0x03    // 写入
#define GPIO_SCAN_DIR_WRITE   0x04    // 扫描写入
//---------------GPIO------------------

#define GPIO_SCAN_MODE_WRITE   0x04    // 扫描写入

// 电源命令ID定义
#define POWER_CMD_SET_VOLTAGE       0x01  // 设置电压命令
#define POWER_CMD_START_CURRENT_READING 0x02  // 开始读取电流命令
#define POWER_CMD_STOP_CURRENT_READING  0x03  // 停止读取电流命令
#define POWER_CMD_READ_CURRENT_DATA     0x04  // 读取电流数据命令

// 电源通道定义
#define POWER_CHANNEL_1         0x01  // 电源通道1
#define POWER_CHANNEL_UA        0x02  // 微安电流通道
#define POWER_CHANNEL_MA        0x03  // 毫安电流通道




// 通用命令包头结构
typedef struct _GENERIC_CMD_HEADER {
  uint8_t protocol_type;  // 协议类型：SPI/IIC/UART等
  uint8_t cmd_id;         // 命令ID：初始化/读/写等
  uint8_t device_index;   // 设备索引
  uint8_t param_count;    // 参数数量
  uint16_t data_len;      // 数据部分长度
  uint16_t total_packets; // 整个数据包的总大小，包括头部和数据
} GENERIC_CMD_HEADER, *PGENERIC_CMD_HEADER;

// 简化的参数头结构
typedef struct _PARAM_HEADER {
  uint16_t param_len;     // 参数长度
} PARAM_HEADER, *PPARAM_HEADER;

// SPI索引定义（保留向后兼容性）
#define SPI1_CS0    0
#define SPI1_CS1    1
#define SPI1_CS2    2
#define SPI2_CS0    3
#define SPI2_CS1    4
#define SPI2_CS2    5

// SPI配置结构体（保留向后兼容性）
typedef struct _SPI_CONFIG {
  char   Mode;            // SPI控制方式:0-硬件控制（全双工模式）,1-硬件控制（半双工模式），2-软件控制（半双工模式）,3-单总线模式，数据线输入输出都为MOSI,4-软件控制（全双工模式）  
  char   Master;          // 主从选择控制:0-从机，1-主机  
  char   CPOL;            // 时钟极性控制:0-SCK空闲时为低电平，1-SCK空闲时为高电平  
  char   CPHA;            // 时钟相位控制:0-第一个SCK时钟采样，1-第二个SCK时钟采样  
  char   LSBFirst;        // 数据移位方式:0-MSB在前，1-LSB在前  
  char   SelPolarity;     // 片选信号极性:0-低电平选中，1-高电平选中  
  unsigned int  ClockSpeedHz;    // SPI时钟频率:单位为HZ，硬件模式下最处50000000，最小390625，频率按2的倍数改变  
} SPI_CONFIG, *PSPI_CONFIG;




int Get_Parameter(uint8_t* buffer, int pos, void* data, uint16_t max_len);


int8_t Process_Command(uint8_t* Buf, uint32_t *Len);


static void Process_GPIO_Write(uint8_t gpio_index, uint8_t write_value);

static void Process_scan_GPIO_Write(uint8_t gpio_index, uint8_t write_value);






#endif /* COMMAND_HANDLER_H */
