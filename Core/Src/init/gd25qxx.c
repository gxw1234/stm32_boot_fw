#include "init/gd25qxx.h"
#include "stm32h7xx_hal_qspi.h"
extern QSPI_HandleTypeDef hqspi;

#define GD25QXX_CMD_WRITE_ENABLE  0x06
#define GD25QXX_CMD_PAGE_PROGRAM  0x02
#define GD25QXX_CMD_READ_DATA     0x03
#define GD25QXX_CMD_READ_STATUS1  0x05
#define GD25QXX_CMD_SECTOR_ERASE  0x20
#define GD25QXX_CMD_JEDEC_ID      0x9F

static void GD25QXX_WriteEnable(void) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction = GD25QXX_CMD_WRITE_ENABLE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.NbData = 0;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

static uint8_t GD25QXX_WaitBusy(void) {
    QSPI_CommandTypeDef cmd = {0};
    uint8_t status = 0;
    cmd.Instruction = GD25QXX_CMD_READ_STATUS1;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 1;
    do {
        HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
        HAL_QSPI_Receive(&hqspi, &status, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    } while (status & 0x01);
    return status;
}

// 单页写入（不跨页，最大256字节）
static void GD25QXX_WritePage(uint32_t addr, const uint8_t *data, uint32_t len) {
    GD25QXX_WriteEnable();
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction = GD25QXX_CMD_PAGE_PROGRAM;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = len;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_QSPI_Transmit(&hqspi, (uint8_t*)data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    GD25QXX_WaitBusy();
}

void GD25QXX_WriteBytes(uint32_t addr, const uint8_t *data, uint32_t datalen) {
    // 1. 计算起始扇区地址
    uint32_t sector_size = 4096;
    uint32_t sector_addr = addr & ~(sector_size - 1);
    GD25QXX_EraseSector(sector_addr);
    // 2. 分页写入（每页最多256字节）
    uint32_t page_size = 256;
    uint32_t remain = datalen;
    uint32_t cur_addr = addr;
    const uint8_t *cur_data = data;
    while (remain > 0) {
        uint32_t page_offset = cur_addr % page_size;
        uint32_t write_len = page_size - page_offset;
        if (write_len > remain) write_len = remain;
        GD25QXX_WritePage(cur_addr, cur_data, write_len);
        cur_addr += write_len;
        cur_data += write_len;
        remain -= write_len;
    }
}

// 修改单字节写为调用多字节写
void GD25QXX_WriteByte(uint32_t addr, uint8_t data) {
    GD25QXX_WriteBytes(addr, &data, 1);
}

void GD25QXX_ReadBytes(uint32_t addr, uint8_t *data, uint32_t datalen) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction = GD25QXX_CMD_READ_DATA;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = datalen;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_QSPI_Receive(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

uint8_t GD25QXX_ReadByte(uint32_t addr) {
    QSPI_CommandTypeDef cmd = {0};
    uint8_t data = 0;
    cmd.Instruction = GD25QXX_CMD_READ_DATA;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 1;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_QSPI_Receive(&hqspi, &data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    return data;
}

void GD25QXX_EraseSector(uint32_t addr) {
    GD25QXX_WriteEnable();
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction = GD25QXX_CMD_SECTOR_ERASE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Address = addr;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.NbData = 0;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    GD25QXX_WaitBusy();
} 