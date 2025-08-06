#include "handler_spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "string.h"

static SPI_HandleTypeDef hspi5;


#define SPI_RX_BUFFER_SIZE 512
static uint8_t spi_rx_buffer[SPI_RX_BUFFER_SIZE];


typedef struct {
    GENERIC_CMD_HEADER header;
    uint8_t data[SPI_RX_BUFFER_SIZE];  
} SPI_Data_Packet;

/**Sindex 1:SPI5   对应的引脚是PH6,PH7,PF11,H8
CS          对应 P6  (H8)
MOSI        对应 P3  （F11)
SCK         对应 P4   (H6)
MISO        对应 P5   (H7)

{H6: P4,H7: P5,F11: P3}
*/
static HAL_StatusTypeDef SPI_GPIO_Init(uint8_t spi_index)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (spi_index == SPI_INDEX_1) { 
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI5;
        PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            return HAL_ERROR;
        }
        __HAL_RCC_SPI5_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        /**SPI5 GPIO Configuration
        PF11    ------> SPI5_MOSI
        PH6     ------> SPI5_SCK
        PH7     ------> SPI5_MISO
        */
        // 配置PH6和PH7
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* CS0引脚配置 */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
        /* CS0默认高电平 */
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET);

        return HAL_OK;
    }
    return HAL_ERROR; 
}


HAL_StatusTypeDef Handler_SPI_Init(uint8_t spi_index, PSPI_CONFIG pConfig)
{
    HAL_StatusTypeDef status;
    if (spi_index == SPI_INDEX_1) { // SPI5
        status = SPI_GPIO_Init(spi_index);
        if (status != HAL_OK) {
            return status;
        }
        __HAL_RCC_SPI5_CLK_ENABLE();
        hspi5.Instance = SPI5;
        hspi5.Init.Mode = pConfig->Master ? SPI_MODE_MASTER : SPI_MODE_SLAVE;
        hspi5.Init.Direction = SPI_DIRECTION_2LINES;
        hspi5.Init.DataSize = SPI_DATASIZE_8BIT; // 默认8位数据宽度
        hspi5.Init.CLKPolarity = pConfig->CPOL ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        hspi5.Init.CLKPhase = pConfig->CPHA ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
        hspi5.Init.NSS = SPI_NSS_SOFT; // 软件NSS管理
        uint32_t spi_clock = HAL_RCC_GetPCLK2Freq(); // 获取PCLK2频率
        uint32_t prescaler = SPI_BAUDRATEPRESCALER_256; // 默认最低速度
        if (pConfig->ClockSpeedHz >= spi_clock / 2)
            prescaler = SPI_BAUDRATEPRESCALER_2;
        else if (pConfig->ClockSpeedHz >= spi_clock / 4)
            prescaler = SPI_BAUDRATEPRESCALER_4;
        else if (pConfig->ClockSpeedHz >= spi_clock / 8)
            prescaler = SPI_BAUDRATEPRESCALER_8;
        else if (pConfig->ClockSpeedHz >= spi_clock / 16)
            prescaler = SPI_BAUDRATEPRESCALER_16;
        else if (pConfig->ClockSpeedHz >= spi_clock / 32)
            prescaler = SPI_BAUDRATEPRESCALER_32;
        else if (pConfig->ClockSpeedHz >= spi_clock / 64)
            prescaler = SPI_BAUDRATEPRESCALER_64;
        else if (pConfig->ClockSpeedHz >= spi_clock / 128)
            prescaler = SPI_BAUDRATEPRESCALER_128;
        prescaler = SPI_BAUDRATEPRESCALER_4;
        hspi5.Init.BaudRatePrescaler = prescaler;
        hspi5.Init.FirstBit = pConfig->LSBFirst ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
        hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi5.Init.CRCPolynomial = 7;
        status = HAL_SPI_Init(&hspi5);
        if (status == HAL_OK && !pConfig->Master) {
            HAL_NVIC_SetPriority(SPI5_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(SPI5_IRQn);
            HAL_SPI_Receive_IT(&hspi5, spi_rx_buffer, SPI_RX_BUFFER_SIZE);
        }
        return status;
    }
    return HAL_ERROR; 
}

HAL_StatusTypeDef Handler_SPI_Transmit(uint8_t spi_index, uint8_t *pTxData, uint8_t *pRxData, uint16_t DataSize, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    if (spi_index == SPI_INDEX_1) { // SPI5
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_RESET);
        if (pRxData != NULL) {
            status = HAL_SPI_TransmitReceive(&hspi5, pTxData, pRxData, DataSize, Timeout);
            return status;
        } else {
            status = HAL_SPI_Transmit(&hspi5, pTxData, DataSize, Timeout);
        }
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET);

        // printf("Handler_SPI_Transmit: spi_index=%d, status=%d\r\n", spi_index, status);
       return status;

    }
    return HAL_ERROR; 
}

void SPI5_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi5);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi5) {
        SPI_Data_Packet packet;
        packet.header.protocol_type = PROTOCOL_SPI;    // SPI协议
        packet.header.cmd_id = CMD_READ;              // 读数据命令
        packet.header.device_index = SPI_INDEX_1;     // SPI设备索引
        packet.header.param_count = 0;                // 无参数
        packet.header.data_len = SPI_RX_BUFFER_SIZE;  // 数据长度
        packet.header.total_packets = sizeof(SPI_Data_Packet); // 总包大小
        memcpy(packet.data, spi_rx_buffer, SPI_RX_BUFFER_SIZE);
        USB_Sender((uint8_t*)&packet, sizeof(packet));
        HAL_SPI_Receive_IT(&hspi5, spi_rx_buffer, SPI_RX_BUFFER_SIZE);
    }
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi5) {
        printf("SPI5 error occurred, ErrorCode: 0x%lX\r\n", hspi->ErrorCode);
        HAL_SPI_Receive_IT(&hspi5, spi_rx_buffer, SPI_RX_BUFFER_SIZE);
    }
}