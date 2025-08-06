
#include "tasks/test_iic_send.h"
#include "stm32h7xx_hal_def.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include "pc_to_stm_command_handler/command_handler.h"

struct rx_tx {
  uint8_t rx;
  uint8_t tx;
};
__IO uint32_t Transfer_Direction = 0;
__IO uint32_t Xfer_Complete = 0;

struct rx_tx rx_tx_data[] = {
  {0xfc, 0x39},
  {0xfd, 0x01},
  {0x18, 48},
  {0x1a, 120},
  {0x09, 0x01},
};

/* Data buffer */
uint8_t aTxBuffer[4] = {0x39, 0x01, 0, 0};  // Transmit buffer
uint8_t aRxBuffer[4] = {0};  // Receive buffer

TaskHandle_t UsbSendTaskHandle;
I2C_HandleTypeDef hi2c3_test_; 

#define DATA_SIZE 64
static uint8_t myData[DATA_SIZE];

// GPIO下压标志位
static uint8_t gpio_pressed_flag = 0;
// USB发送标志位
static uint8_t usb_send_flag = 0;


/**
 * @brief Initialize I2C3 as slave mode
 * @retval HAL status
 */
HAL_StatusTypeDef Test_I2C3_Slave_Init(void)
{
    hi2c3_test_.Instance = I2C3;
    hi2c3_test_.Init.Timing = 0x00300F38;
    hi2c3_test_.Init.OwnAddress1 = 0xdc;
    hi2c3_test_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3_test_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3_test_.Init.OwnAddress2 = 0;
    hi2c3_test_.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3_test_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3_test_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_StatusTypeDef status = HAL_I2C_Init(&hi2c3_test_);
    if (status != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3_test_, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler();
    }
  
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3_test_, 0) != HAL_OK)
    {
      Error_Handler();
    }
    
    return status;
}

/**
 * @brief USB send test task
 * @param argument Not used
 */
void IIC_interruption_Task(void *argument)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    /* Prevent compiler warning for unused argument */
    (void)argument;
    // vTaskDelay(pdMS_TO_TICKS(1000));

    if (Test_I2C3_Slave_Init() == HAL_OK) {
        printf("I2C3 slave mode init success, address: 0x%02X\r\n", 0x6e);
        status = HAL_I2C_EnableListen_IT(&hi2c3_test_);
        if (status != HAL_OK) {
            printf("Failed to enable I2C3 listen mode, error code: %d\r\n", status);
        } else {
            printf("I2C3 listen mode enabled\r\n");
        }
    } else {
        printf("I2C3 slave mode init failed, error code: %d\r\n", status);
    }

    while (1)
    {
        if (Xfer_Complete == 1) {
            while (HAL_I2C_GetState(&hi2c3_test_) != HAL_I2C_STATE_READY){}
            HAL_I2C_EnableListen_IT(&hi2c3_test_);
            Xfer_Complete = 0;
        }
        

        if (usb_send_flag == 1) {
           
            typedef struct {
                GENERIC_CMD_HEADER header;
                uint8_t status;  // 状态数据
            } Scan_gpio_Response;
            
            Scan_gpio_Response response;
            response.header.protocol_type = PROTOCOL_SPI;
            response.header.cmd_id = GPIO_SCAN_MODE_WRITE;
            response.header.device_index = 1;
            response.header.param_count = 0;
            response.header.data_len = sizeof(uint8_t);
            response.header.total_packets = sizeof(Scan_gpio_Response);
            response.status = 1;
            USB_Sender((uint8_t*)&response, sizeof(response));

            usb_send_flag = 0; // 复位标志位
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    Xfer_Complete = 1;
    HAL_I2C_Slave_Seq_Transmit_IT(&hi2c3_test_, (uint8_t *)&aTxBuffer[1], 1, I2C_NEXT_FRAME);
    // printf("I2C slave TX complete callback\r\n");
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    Xfer_Complete = 1;
    HAL_I2C_Slave_Seq_Receive_IT(&hi2c3_test_, &aRxBuffer[1], 1, I2C_NEXT_FRAME);
    // printf("I2C slave RX complete callback, received: 0x%02X\r\n", aRxBuffer[0]);
}


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    Transfer_Direction = TransferDirection;
    
    if (Transfer_Direction != I2C_DIRECTION_TRANSMIT) {
        // Master read request, slave sending data
        for (int i = 0; i < sizeof(rx_tx_data) / sizeof(rx_tx_data[0]); i++) {
            if (aRxBuffer[0] == rx_tx_data[i].rx) {
                aTxBuffer[0] = rx_tx_data[i].tx;
                if (aRxBuffer[0] == 0x09) {
                    if (gpio_pressed_flag) {
                        usb_send_flag = 1;
                        gpio_pressed_flag = 0; 
                    } else {
                        printf("N");
                    }
                }
            }
        }
        
        // printf("Address match callback: Master READ, sending data: 0x%02X\r\n", aTxBuffer[0]);
        
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c3_test_, (uint8_t *)&aTxBuffer[0], 1, I2C_NEXT_FRAME) != HAL_OK) {
            printf("Failed to set slave transmit\r\n");
        }
    }
    else {
        // Master write request, slave receiving data
        // printf("Address match callback: Master WRITE, preparing to receive data\r\n");
        
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c3_test_, (uint8_t *)&aRxBuffer[0], 1, I2C_NEXT_FRAME) != HAL_OK) {
            printf("Failed to set slave receive\r\n");
        }
    }
}


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    // printf("I2C listen complete callback\r\n");
}


void Set_GPIO_Press_Flag(void) {
    gpio_pressed_flag = 1;
}

