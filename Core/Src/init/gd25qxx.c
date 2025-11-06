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
    // 注意：扇区擦除已在BOOTLOADER_START_WRITE命令中完成，这里只需要分页写入
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

/**
 * @brief  读取Flash芯片ID
 * @param  id: 存储ID的缓冲区（至少3字节）
 * @retval HAL状态
 */
HAL_StatusTypeDef GD25QXX_ReadID(uint8_t *id) {
    QSPI_CommandTypeDef cmd = {0};
    
    cmd.Instruction = GD25QXX_CMD_JEDEC_ID;  // 0x9F
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 3;  // 读取3字节ID
    
    HAL_StatusTypeDef status = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    if (status == HAL_OK) {
        status = HAL_QSPI_Receive(&hqspi, id, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    }
    
    return status;
}

static HAL_StatusTypeDef GD25QXX_ReadSR2(uint8_t *sr2) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.Instruction = 0x35;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 1;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Receive(&hqspi, sr2, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

static HAL_StatusTypeDef GD25QXX_WriteSR2(uint8_t sr2) {
    QSPI_CommandTypeDef cmd = {0};
    GD25QXX_WriteEnable();
    cmd.Instruction = 0x31;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 0;
    cmd.NbData = 1;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Transmit(&hqspi, &sr2, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    GD25QXX_WaitBusy();
    return HAL_OK;
}

static HAL_StatusTypeDef GD25QXX_EnableQuadMode_Winbond(void) {
    uint8_t sr2 = 0;
    if (GD25QXX_ReadSR2(&sr2) != HAL_OK) return HAL_ERROR;
    if (sr2 & 0x02) return HAL_OK;
    sr2 |= 0x02;
    if (GD25QXX_WriteSR2(sr2) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

/**
 * @brief  启用XIP内存映射模式
 * @retval HAL状态
 */
HAL_StatusTypeDef GD25QXX_EnableXIP(void) {
    extern int printf(const char *format, ...);
    QSPI_CommandTypeDef cmd = {0};
    QSPI_MemoryMappedTypeDef memMappedCfg = {0};
    
    printf("Configuring XIP memory mapped mode...\r\n");
    
    /* 首先确保QSPI处于就绪状态 */
    if (hqspi.State != HAL_QSPI_STATE_READY) {
        printf("QSPI not ready, aborting current operation...\r\n");
        HAL_QSPI_Abort(&hqspi);
        HAL_Delay(10);
    }
    
    /* 检测Flash厂商并使用对应配置 */
    uint8_t flash_id[3] = {0};
    if (GD25QXX_ReadID(flash_id) == HAL_OK) {
        printf("Flash ID: 0x%02X 0x%02X 0x%02X\r\n", flash_id[0], flash_id[1], flash_id[2]);
        
        if (flash_id[0] == 0xEF && flash_id[1] == 0x40 && flash_id[2] == 0x18) {
            printf("Detected Winbond W25Q128 - using Winbond Quad XIP config\r\n");
            if (GD25QXX_EnableQuadMode_Winbond() != HAL_OK) {
                return HAL_ERROR;
            }
            cmd.Instruction = 0x6B;
            cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
            cmd.AddressSize = QSPI_ADDRESS_24_BITS;
            cmd.AddressMode = QSPI_ADDRESS_1_LINE;
            cmd.DataMode = QSPI_DATA_4_LINES;
            cmd.DummyCycles = 8;
            cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
            cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
            cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
            cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
            
        } else if (flash_id[0] == 0xC8) {
            printf("Detected GigaDevice Flash - using GD XIP config\r\n");
            
            /* 原GD25QXX配置 */
            cmd.Instruction = GD25QXX_CMD_READ_DATA;  // 0x03 标准读取命令
            cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
            cmd.AddressSize = QSPI_ADDRESS_24_BITS;
            cmd.AddressMode = QSPI_ADDRESS_1_LINE;
            cmd.DataMode = QSPI_DATA_1_LINE;
            cmd.DummyCycles = 0;  // 标准读取命令无需虚拟周期
            cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
            cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
            cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
            cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
            
        } else {
            printf("Unknown Flash - using conservative XIP config\r\n");
            
            /* 保守配置：使用标准读取命令 */
            cmd.Instruction = 0x03;  // 标准读取命令
            cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
            cmd.AddressSize = QSPI_ADDRESS_24_BITS;
            cmd.AddressMode = QSPI_ADDRESS_1_LINE;
            cmd.DataMode = QSPI_DATA_1_LINE;
            cmd.DummyCycles = 0;
            cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
            cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
            cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
            cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
        }
    } else {
        printf("Flash ID read failed - using default config\r\n");
        return HAL_ERROR;
    }
    
    printf("Enabling memory mapped mode...\r\n");
    
    /* 配置内存映射参数 */
    memMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_ENABLE;
    memMappedCfg.TimeOutPeriod = 0x20;  // 设置超时周期
    
    /* 启用内存映射模式 */
    HAL_StatusTypeDef status = HAL_QSPI_MemoryMapped(&hqspi, &cmd, &memMappedCfg);
    
    if (status == HAL_OK) {
        printf("Memory mapped mode enabled successfully\r\n");
    } else {
        printf("Failed to enable memory mapped mode, status: %d\r\n", status);
    }
    
    return status;
}

/**
 * @brief  禁用XIP内存映射模式
 * @retval HAL状态
 */
HAL_StatusTypeDef GD25QXX_DisableXIP(void) {
    /* 中止内存映射模式 */
    return HAL_QSPI_Abort(&hqspi);
}

/**
 * @brief  检查XIP状态
 * @retval 1: XIP已启用, 0: XIP未启用
 */
uint8_t GD25QXX_CheckXIPStatus(void) {
    /* 检查QSPI状态寄存器中的内存映射模式标志 */
    return (hqspi.State == HAL_QSPI_STATE_BUSY_MEM_MAPPED) ? 1 : 0;
}

/**
 * @brief  安全读取XIP内存 - 使用字节访问避免总线错误
 * @param  addr: 要读取的地址
 * @param  data: 存储读取数据的指针
 * @retval 1: 成功, 0: 失败
 */
static uint8_t GD25QXX_SafeXIPRead(uint32_t addr, uint32_t *data) {
    extern int printf(const char *format, ...);
    
    /* 检查地址是否在XIP范围内 */
    if (addr < 0x90000000 || addr >= 0x91000000) {
        printf("Address 0x%08lX out of XIP range\r\n", addr);
        return 0;
    }
    
    /* 检查XIP状态 */
    if (!GD25QXX_CheckXIPStatus()) {
        printf("XIP not enabled\r\n");
        return 0;
    }
    
    printf("Attempting byte-by-byte read at 0x%08lX...\r\n", addr);
    
    /* 尝试逐字节读取以避免总线错误 */
    __disable_irq();
    
    /* 设置默认值 */
    *data = 0xDEADBEEF;
    
    /* 尝试读取第一个字节 */
    volatile uint8_t byte0, byte1, byte2, byte3;
    
    printf("Reading byte 0...\r\n");
    byte0 = *(volatile uint8_t*)(addr + 0);
    printf("Byte 0: 0x%02X\r\n", byte0);
    
    printf("Reading byte 1...\r\n");
    byte1 = *(volatile uint8_t*)(addr + 1);
    printf("Byte 1: 0x%02X\r\n", byte1);
    
    printf("Reading byte 2...\r\n");
    byte2 = *(volatile uint8_t*)(addr + 2);
    printf("Byte 2: 0x%02X\r\n", byte2);
    
    printf("Reading byte 3...\r\n");
    byte3 = *(volatile uint8_t*)(addr + 3);
    printf("Byte 3: 0x%02X\r\n", byte3);
    
    /* 组合成32位数据 */
    *data = (uint32_t)byte0 | ((uint32_t)byte1 << 8) | ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 24);
    
    __enable_irq();
    
    printf("Combined data: 0x%08lX\r\n", *data);
    return 1;
}

/**
 * @brief  测试XIP功能
 * @retval None
 */
void GD25QXX_TestXIP(void) {
    extern int printf(const char *format, ...);
    
    printf("=== XIP Test Start ===\r\n");
    printf("QSPI State: %d (130=MEM_MAPPED)\r\n", hqspi.State);
    
    /* 注意：在内存映射模式下，无法使用命令模式读取Flash ID */
    printf("Note: Flash ID cannot be read in memory-mapped mode\r\n");
    printf("XIP mode is active, testing direct memory access...\r\n");
    
    /* 检查D-Cache状态 */
    printf("D-Cache enabled: %s\r\n", (SCB->CCR & SCB_CCR_DC_Msk) ? "YES" : "NO");
    
    /* 测试不同的XIP地址读取 */
    uint32_t test_addresses[] = {
        0x90000000,  // XIP起始地址
        0x90000004,  // XIP起始地址+4
        0x90020000,  // 应用程序地址
        0x90020004   // 应用程序地址+4
    };
    
    for (int i = 0; i < 4; i++) {
        uint32_t addr = test_addresses[i];
        uint32_t data = 0;
        
        printf("Testing XIP read at 0x%08lX: ", addr);
        
        /* 第一次尝试：正常访问 */
        if (GD25QXX_SafeXIPRead(addr, &data)) {
            printf("0x%08lX\r\n", data);
        } else {
            printf("FAILED - trying with D-Cache disabled...\r\n");
            
            /* 第二次尝试：禁用D-Cache */
            printf("Disabling D-Cache for XIP test...\r\n");
            SCB_DisableDCache();
            
            if (GD25QXX_SafeXIPRead(addr, &data)) {
                printf("SUCCESS with D-Cache disabled: 0x%08lX\r\n", data);
            } else {
                printf("STILL FAILED even with D-Cache disabled\r\n");
            }
            
            /* 重新启用D-Cache */
            SCB_EnableDCache();
            printf("D-Cache re-enabled\r\n");
        }
        
        HAL_Delay(100);  // 增加延时
    }
    
    printf("=== XIP Test End ===\r\n");
}

/**
 * @brief  测试不同的XIP内存映射地址
 * @retval None
 */
void GD25QXX_TestAlternativeXIPAddresses(void) {
    extern int printf(const char *format, ...);
    
    printf("=== Testing Alternative XIP Addresses ===\r\n");
    
    /* STM32H750可能的QSPI内存映射地址 */
    uint32_t possible_addresses[] = {
        0x90000000,  // 标准QSPI Bank1地址
        0x70000000,  // 备用QSPI Bank1地址
        0x60000000,  // 另一个可能的映射地址
        0xA0000000   // 高地址空间映射
    };
    
    const char* address_names[] = {
        "Standard QSPI Bank1 (0x90000000)",
        "Alternative QSPI Bank1 (0x70000000)", 
        "Alternative mapping (0x60000000)",
        "High address space (0xA0000000)"
    };
    
    for (int i = 0; i < 4; i++) {
        uint32_t base_addr = possible_addresses[i];
        printf("\nTesting %s:\r\n", address_names[i]);
        
        /* 临时禁用D-Cache以避免Cache问题 */
        SCB_DisableDCache();
        
        /* 尝试读取前4个字节 */
        __disable_irq();
        
        printf("Attempting to read first 4 bytes...\r\n");
        
        volatile uint8_t byte0, byte1, byte2, byte3;
        uint8_t success = 1;
        
        /* 逐字节读取以精确定位问题 */
        printf("Reading byte at 0x%08lX: ", base_addr);
        byte0 = *(volatile uint8_t*)(base_addr);
        printf("0x%02X ", byte0);
        
        printf("0x%08lX: ", base_addr + 1);
        byte1 = *(volatile uint8_t*)(base_addr + 1);
        printf("0x%02X ", byte1);
        
        printf("0x%08lX: ", base_addr + 2);
        byte2 = *(volatile uint8_t*)(base_addr + 2);
        printf("0x%02X ", byte2);
        
        printf("0x%08lX: ", base_addr + 3);
        byte3 = *(volatile uint8_t*)(base_addr + 3);
        printf("0x%02X\r\n", byte3);
        
        uint32_t combined = (uint32_t)byte0 | ((uint32_t)byte1 << 8) | 
                           ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 24);
        
        printf("Combined 32-bit value: 0x%08lX\r\n", combined);
        
        __enable_irq();
        
        /* 重新启用D-Cache */
        SCB_EnableDCache();
        
        printf("Address 0x%08lX test completed successfully\r\n", base_addr);
        HAL_Delay(100);
    }
    
    printf("=== Alternative XIP Address Test Complete ===\r\n");
}