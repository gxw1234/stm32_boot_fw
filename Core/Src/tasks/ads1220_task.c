#include "tasks/ads1220_task.h"
#include "tasks/lcd_task.h"  /* 包含 LCD 函数声明 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h> /* 添加 memcpy 函数的头文件 */
#include "usb_device.h"
#include "usbd_cdc_if.h"


#define SPI_CS1_LOW()       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define SPI_CS1_HIGH()      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define SPI_CS2_LOW()       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)
#define SPI_CS2_HIGH()      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)



SPI_HandleTypeDef hspi4;


static uint32_t sample_count = 0;
static float voltage_sum = 0;


static uint32_t sample_count_2 = 0;
static uint8_t sample_count_threshold_reached = 0; 


float current_uA =0;
float current_mA =0;

float current_send_temp =0;

#define DATA_BUFFER_SIZE 20000  // 足够存储1000个浮点值的字符串和换行符
static uint8_t data_buffer[DATA_BUFFER_SIZE];  // 大的数据缓冲区
static uint8_t sending_enabled = 0;  // 是否启用数据发送
static uint32_t buffer_pos = 0;  // 数据缓冲区当前位置
#define VALUE_X_POS 90  
#define CURRENT_MA_Y_POS 44  
#define WAVE_X 10                 // 波形图左上角X坐标
#define WAVE_Y 120                 // 波形图左上角Y坐标
#define WAVE_WIDTH (LCD_WIDTH-20) // 波形图宽度
#define WAVE_HEIGHT 100           // 波形图高度


/**
 * @brief 启用电压数据发送
 */
void Enable_Current_Data_Sending(void) {
    buffer_pos = 0;  // 重置缓冲区位置
    sending_enabled = 1;  // 启用发送开关
}

/**
 * @brief 禁用电压数据发送
 */
void Disable_Current_Data_Sending(void) {
    sending_enabled = 0;  // 关闭发送开关
}
#define VREF              2.048f    // 参考电压2.048V
#define ADC_FSR           8388608.0f // 2^23, ADC满量程范围

static float Convert_ADC_To_Voltage(uint32_t adc_value)
{
    int32_t signed_value;
    float voltage;
    uint32_t temp = 0;
    if(adc_value & 0x800000) {
        signed_value = (int32_t)(adc_value| 0xFF000000);
    } else {
        signed_value = (int32_t)adc_value;
    }
    voltage = ((float)signed_value * VREF) / ADC_FSR;
    return voltage;
}

void ADS1220_Task(void *argument)
{


    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ADS1220_Init();
    vTaskDelay(pdMS_TO_TICKS(1));
    SPI_CS1_LOW();
    SPI_Transmit(0x06);  // RESET命令
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x43);  // WREG命令，写寄存器3
    SPI_Transmit(0x00);  // 配置数据 - 寄存器0：PGA=1, AIN0/AIN1
    SPI_Transmit(0xD4);  // 配置数据 - 寄存器1：DR=20SPS, 连续转换模式
    SPI_Transmit(0x17);  // 配置数据 - 寄存器2：IDAC关闭
    SPI_Transmit(0xA0);  // 配置数据 - 寄存器3：默认设置
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x23);
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x08);  
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_CS1_HIGH();
    vTaskDelay(pdMS_TO_TICKS(1)); 
    printf("--------System Start!\r\n");

    SPI_CS2_LOW();
    SPI_Transmit(0x06);  // RESET命令
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x43);  // WREG命令，写寄存器3
    SPI_Transmit(0x00);  // 配置数据 - 寄存器0：PGA=1, AIN0/AIN1
    SPI_Transmit(0xD4);  // 配置数据 - 寄存器1：DR=20SPS, 连续转换模式
    SPI_Transmit(0x17);  // 配置数据 - 寄存器2：IDAC关闭
    SPI_Transmit(0xA0);  // 配置数据 - 寄存器3：默认设置
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x23);
    vTaskDelay(pdMS_TO_TICKS(1)); 
    SPI_Transmit(0x08);  
    vTaskDelay(pdMS_TO_TICKS(1));
    SPI_CS2_HIGH();
    vTaskDelay(pdMS_TO_TICKS(1));

    /* PH9  微安中断  */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    /* PH10 毫安中断 */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    LCD_Show_String(5, CURRENT_MA_Y_POS, "Current:", COLOR_WHITE, COLOR_BLACK, FONT_1608);
    LCD_Draw_Current_Wave(WAVE_X, WAVE_Y, WAVE_WIDTH, WAVE_HEIGHT);
    while(1) {



        // SPI_CS3_LOW();

        if(sample_count_threshold_reached) {
            // 当达到阈值且数据发送功能启用时，发送所有收集的数据
            if (sending_enabled && buffer_pos > 0) {
                USB_Sender(data_buffer, buffer_pos);
                buffer_pos = 0; // 重置缓冲区位置
            }
            float avg_current_mA = current_mA / 1000;  


            // printf("current_mA : %.6f mA\r\n", avg_current_mA);
            char current_str[30] = {0};
            sprintf(current_str, "%.3f mA", avg_current_mA);
            LCD_Show_String(VALUE_X_POS, CURRENT_MA_Y_POS, current_str, COLOR_CYAN, COLOR_BLACK, FONT_1608);
            LCD_Add_Current_Point(avg_current_mA);
            LCD_Draw_Current_Wave(WAVE_X, WAVE_Y, WAVE_WIDTH, WAVE_HEIGHT);
            current_mA = 0;  
            sample_count_threshold_reached = 0; 
            sample_count_2 = 0; // 重置计数器
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // SPI_CS3_HIGH();

    }
}

void ADS1220_Init(void)
{
        hspi4.Instance = SPI4;
        hspi4.Init.Mode = SPI_MODE_MASTER;
        hspi4.Init.Direction = SPI_DIRECTION_2LINES;
        hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;    // CPOL = 0
        hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;        // CPHA = 1
        hspi4.Init.NSS = SPI_NSS_SOFT;
        hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  // 确保SCLK周期>150ns
        hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi4.Init.CRCPolynomial = 7;
        hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
        hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
      if (HAL_SPI_Init(&hspi4) != HAL_OK)
      {
        Error_Handler();
      }
    //   GPIO_InitTypeDef GPIO_InitStruct = {0};
    //   GPIO_InitStruct.Pin = GPIO_PIN_11;
    //   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //   GPIO_InitStruct.Pull = GPIO_PULLUP;
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    //   /* CS1默认高电平 */
    //   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
      
    //   /* CS2引脚配置 */
    //   GPIO_InitStruct.Pin = GPIO_PIN_15;
    //   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //   GPIO_InitStruct.Pull = GPIO_PULLUP;
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    //   /* CS2默认高电平 */
    //   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);







      
    //   __HAL_RCC_GPIOH_CLK_ENABLE();
    //   __HAL_RCC_GPIOE_CLK_ENABLE();
 
    //   GPIO_InitStruct.Pin = GPIO_PIN_7;
    //   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //   GPIO_InitStruct.Pull = GPIO_PULLUP;
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    //   /* CS3默认高电平 */
    //   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
      



    //   /* CS4引脚配置 */
    //   GPIO_InitStruct.Pin = GPIO_PIN_8;
    //   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //   GPIO_InitStruct.Pull = GPIO_PULLUP;
    //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //   HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    //   /* CS4默认高电平 */
    //   HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET);

      /* 所有初始化完成后，启用EXTI中断 */
      printf("ADS1220 initialization completed, enabling EXTI interrupts\r\n");
      
}

void SPI_Transmit(uint8_t data) {
    uint8_t txData[] = {data};  
    uint8_t rxData[1];          

    if(HAL_SPI_TransmitReceive(&hspi4, txData, rxData, sizeof(txData), 100) != HAL_OK)
    {
        Error_Handler();
    }

    printf("SPI: TX=0x%02X, RX=0x%02X\r\n", txData[0], rxData[0]);
}

uint8_t SPI_TransmitReceive(uint8_t data) {
    uint8_t txData[] = {data}; 
    uint8_t rxData[1];         
    if(HAL_SPI_TransmitReceive(&hspi4, txData, rxData, sizeof(txData), 100) != HAL_OK)
    {
        Error_Handler();
    }
    return rxData[0];
}

/**
  * @brief GPIO EXTI callback function
  * @param GPIO_Pin: Pin connected to EXTI line
  * @retval None
  */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_10)
    //微安档
    {
    //    SPI_CS2_LOW();
    //    uint32_t adc_value = 0;
    //    uint8_t msb = SPI_TransmitReceive(0xFF);  // 读取高8位
    //    uint8_t mid = SPI_TransmitReceive(0xFF);  // 读取中8位
    //    uint8_t lsb = SPI_TransmitReceive(0xFF);  // 读取低8位
    //    SPI_CS2_HIGH();
    //    adc_value = (uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb;
    //     float voltage = Convert_ADC_To_Voltage(adc_value);  
    //     current_uA = (2008*voltage +  0.27) /1000 ;

    //     current_mA += current_uA;

     }
     else if(GPIO_Pin == GPIO_PIN_9)
     {
        //毫安档
       SPI_CS1_LOW();
       uint32_t adc_value = 0;
       uint8_t msb = SPI_TransmitReceive(0xFF);  // 读取高8位
       uint8_t mid = SPI_TransmitReceive(0xFF);  // 读取中8位
       uint8_t lsb = SPI_TransmitReceive(0xFF);  // 读取低8位
       SPI_CS1_HIGH();
       adc_value = (uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb;
       float voltage = Convert_ADC_To_Voltage(adc_value);  
       float current_temp = voltage * 1170.5 + 0.222;


       current_mA += current_temp;


    //    uint8_t data_type = 0; 
    //    if (current_temp < 1)
    //    {
    //        if(current_uA != 0)
    //        {
    //             current_send_temp = current_uA;
    //             current_mA += current_uA;
    //             data_type = 1; // 微安数据
    //        }
    //    } else
    //    {
    //        current_mA += current_temp;
    //        current_send_temp = current_temp;
    //        data_type = 0; // 毫安数据
    //    }
    //    if (sending_enabled && buffer_pos < DATA_BUFFER_SIZE - sizeof(float) - 1) { 
    //         data_buffer[buffer_pos++] = data_type;
    //         memcpy(&data_buffer[buffer_pos], &current_send_temp, sizeof(float));
    //         buffer_pos += sizeof(float);
    //         }
       sample_count_2++;
       if(sample_count_2 >= 1000)
       {
           sample_count_threshold_reached = 1; 
       }
     }
   

   

}