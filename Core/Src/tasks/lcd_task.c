#include "tasks/lcd_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // 用于abs()函数
#include "Font_library/fonts.h"

/* 中文字库 - 汉字 (16x16点阵) */
const uint8_t GB2312_HanZi[5][32] = { 
    /* 最 (0xD7EE) 16x16 */
    {0x08,0x00,0x3F,0xF8,0x20,0x08,0xFF,0xFE,0x20,0x08,0x2F,0xE8,0x2A,0xA8,0x2A,0xA8,
     0x2F,0xE8,0x2A,0xA8,0x2A,0xA8,0x2F,0xE8,0x20,0x08,0x20,0x08,0x3F,0xF8,0x00,0x00},
    /* 大 (0xB4F3) 16x16 */
    {0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0xFF,0xFE,
     0x01,0x00,0x01,0x00,0x01,0x00,0x02,0x80,0x04,0x40,0x18,0x30,0x60,0x0C,0x00,0x00},
    /* 小 (0xD0A1) 16x16 */
    {0x00,0x00,0x01,0x00,0x02,0x80,0x04,0x40,0x08,0x20,0x10,0x10,0x3F,0xF8,0x01,0x00,
     0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00},
    /* 时 (0xCAB1) 16x16 */
    {0x10,0x40,0x10,0x40,0x10,0x40,0x3F,0xFC,0x22,0x44,0x22,0x44,0x22,0x44,0x22,0x44,
     0x3F,0xFC,0x22,0x44,0x22,0x44,0x42,0x42,0x82,0x41,0x02,0x40,0x04,0x20,0x18,0x00},
    /* 间 (0xBCE4) 16x16 */
    {0x01,0x00,0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x11,0x10,0x11,0x10,
     0x11,0x10,0x11,0x10,0xFF,0xFE,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00}
};

// /* ST7789 LCD引脚定义 */
// #define LCD_MOSI_PORT   GPIOG
// #define LCD_MOSI_PIN    GPIO_PIN_14
// #define LCD_CLK_PORT    GPIOG
// #define LCD_CLK_PIN     GPIO_PIN_13
// #define LCD_CS_PORT     GPIOG
// #define LCD_CS_PIN      GPIO_PIN_8
// #define LCD_RST_PORT    GPIOG
// #define LCD_RST_PIN     GPIO_PIN_12
// #define LCD_DC_PORT     GPIOG
// #define LCD_DC_PIN      GPIO_PIN_15
// #define LCD_BLK_PORT    GPIOI
// #define LCD_BLK_PIN     GPIO_PIN_6

/* ST7789命令定义 */
#define ST7789_NOP       0x00
#define ST7789_SWRESET   0x01
#define ST7789_SLPIN     0x10
#define ST7789_SLPOUT    0x11
#define ST7789_NORON     0x13
#define ST7789_INVOFF    0x20
#define ST7789_INVON     0x21
#define ST7789_DISPOFF   0x28
#define ST7789_DISPON    0x29
#define ST7789_CASET     0x2A
#define ST7789_RASET     0x2B
#define ST7789_RAMWR     0x2C
#define ST7789_MADCTL    0x36
#define ST7789_COLMOD    0x3A

/* 屏幕参数 */
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

/* 颜色定义 */
#define COLOR_BLACK   0x0000
#define COLOR_BLUE    0x001F
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW  0xFFE0
#define COLOR_WHITE   0xFFFF

/* SPI句柄定义 */
SPI_HandleTypeDef hspi6; // 用于LCD的SPI句柄

/* 引脚控制宏定义 - 仅保留需要单独控制的引脚 */
#define LCD_CS_HIGH   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET)
#define LCD_CS_LOW    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET)
#define LCD_RST_HIGH  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET)
#define LCD_RST_LOW   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET)
#define LCD_DC_HIGH   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET)
#define LCD_DC_LOW    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET)
#define LCD_BLK_ON    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET)  // 低电平激活
#define LCD_BLK_OFF   HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET)

/* 延时函数 */
void LCD_Delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief 初始化LCD使用的SPI接口
 */
void LCD_SPI_Init(void)
{
    // SPI6初始化 - 用于LCD
    hspi6.Instance = SPI6;
    hspi6.Init.Mode = SPI_MODE_MASTER;          // 主模式
    hspi6.Init.Direction = SPI_DIRECTION_1LINE;  // 单线模式，只发送
    hspi6.Init.DataSize = SPI_DATASIZE_8BIT;    // 8位数据
    hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;  // 时钟极性CPOL=0
    hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;      // 时钟相位CPHA=0
    hspi6.Init.NSS = SPI_NSS_SOFT;              // 软件片选
    hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 分频系数
    hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;     // 高位在前
    hspi6.Init.TIMode = SPI_TIMODE_DISABLE;     // 禁用TI模式
    hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // 禁用CRC
    hspi6.Init.CRCPolynomial = 0;
    
    if (HAL_SPI_Init(&hspi6) != HAL_OK)
    {
        // 初始化错误处理
        printf("LCD SPI init failed\r\n");
        return;
    }
    

    
    // 使能SPI
    __HAL_SPI_ENABLE(&hspi6);
}

/* 发送一个字节数据 - 使用硬件SPI */
void LCD_Write_Byte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi6, &data, 1, HAL_MAX_DELAY);
}

/* 发送命令 */
void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_DC_LOW;  // 命令模式
    LCD_CS_LOW;  // 选中LCD
    LCD_Write_Byte(cmd);
    LCD_CS_HIGH; // 取消选中
}

/* 发送数据 */
void LCD_Write_Data(uint8_t data)
{
    LCD_DC_HIGH; // 数据模式
    LCD_CS_LOW;  // 选中LCD
    LCD_Write_Byte(data);
    LCD_CS_HIGH; // 取消选中
}

/* 发送16位数据 */
void LCD_Write_Data_16Bit(uint16_t data)
{
    LCD_DC_HIGH; // 数据模式
    LCD_CS_LOW;  // 选中LCD
    LCD_Write_Byte(data >> 8);
    LCD_Write_Byte(data);
    LCD_CS_HIGH; // 取消选中
}

/* 复位LCD */
void LCD_Reset(void)
{
    LCD_RST_HIGH;
    LCD_Delay(10);
    LCD_RST_LOW;
    LCD_Delay(10);
    LCD_RST_HIGH;
    LCD_Delay(120);
}

/* 设置显示区域 */
void LCD_Set_Window(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    // 列地址设置
    LCD_Write_Cmd(ST7789_CASET);
    LCD_Write_Data(x_start >> 8);
    LCD_Write_Data(x_start & 0xFF);
    LCD_Write_Data(x_end >> 8);
    LCD_Write_Data(x_end & 0xFF);
    
    // 行地址设置
    LCD_Write_Cmd(ST7789_RASET);
    LCD_Write_Data(y_start >> 8);
    LCD_Write_Data(y_start & 0xFF);
    LCD_Write_Data(y_end >> 8);
    LCD_Write_Data(y_end & 0xFF);
    
    // 开始写入GRAM
    LCD_Write_Cmd(ST7789_RAMWR);
}

/* 清屏函数 */
void LCD_Clear(uint16_t color)
{
    uint16_t i, j;
    LCD_Set_Window(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
    
    for(i = 0; i < LCD_WIDTH; i++)
    {
        for(j = 0; j < LCD_HEIGHT; j++)
        {
            LCD_Write_Data_16Bit(color);
        }
    }
}

/* 画点函数 */
void LCD_Draw_Point(uint16_t x, uint16_t y, uint16_t color)
{
    if(x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    
    LCD_Set_Window(x, y, x, y);
    LCD_Write_Data_16Bit(color);
}

/* 画线函数 */
void LCD_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t sx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int16_t sy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    dx = (dx > 0) ? dx : -dx;
    dy = (dy > 0) ? dy : -dy;
    
    int16_t err = dx - dy;
    int16_t e2;
    
    while(1)
    {
        LCD_Draw_Point(x1, y1, color);
        if(x1 == x2 && y1 == y2) break;
        
        e2 = 2 * err;
        if(e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        
        if(e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

/* 画一个填充的矩形 */
void LCD_Fill_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_Set_Window(x1, y1, x2, y2);
    
    for(uint16_t i = 0; i < (x2-x1+1)*(y2-y1+1); i++)
    {
        LCD_Write_Data_16Bit(color);
    }
}

/* 初始化ST7789 LCD */
void LCD_Init(void)
{

    
    // 初始化SPI接口
    LCD_SPI_Init();

    
    // 打开背光 - 先打开背光可以提前看到屏幕
    LCD_BLK_ON;

    LCD_Delay(10);
    
    // 复位LCD

    LCD_Reset();
    
    // 软件复位命令
    LCD_Write_Cmd(ST7789_SWRESET);
    LCD_Delay(150);  // 增加复位后的延时
    
    // 退出睡眠模式

    LCD_Write_Cmd(ST7789_SLPOUT);
    LCD_Delay(150);  // 增加退出睡眠的延时
    
    // 设置像素格式，16位RGB565

    LCD_Write_Cmd(ST7789_COLMOD);
    LCD_Write_Data(0x55);  // 尝试不同的像素格式，0x55对应16位/像素
    LCD_Delay(10);
    
    // 设置显示方向 - 尝试多种方向
  
    LCD_Write_Cmd(ST7789_MADCTL);
    LCD_Write_Data(0x08);  // 修改为0x08，这是标准的上下正确的方向值
    LCD_Delay(10);
    
    // 开启显示反转

    LCD_Write_Cmd(ST7789_INVON);  // 显示反转ON
    LCD_Delay(10);
    
    // 开启显示

    LCD_Write_Cmd(ST7789_NORON);
    LCD_Delay(10);
    LCD_Write_Cmd(ST7789_DISPON);
    LCD_Delay(120);  // 等待显示稳定
    

    
    // 屏幕初始化，清为红色(便于调试，更容易看出屏幕是否工作)
    LCD_Clear(COLOR_RED);
}

/* 显示ASCII字符 */
void LCD_Show_Char(uint16_t x, uint16_t y, uint8_t ch, uint16_t color, uint16_t bg_color, uint8_t size)
{
    // 根据字体大小确定宽度
    uint16_t char_width;
    uint16_t char_height = size;
    
    if(size == FONT_0806) {
        char_width = 6;
    } else if(size == FONT_1206) {
        char_width = 6;
    } else if(size == FONT_1608) {
        char_width = 8;
    } else if(size == FONT_2010) {
        char_width = 10;
    } else if(size == FONT_2412) {
        char_width = 12;
    } else {
        // 默认使用12x6字体
        char_width = 6;
        char_height = 12;
    }
    
    // 检查坐标是否超出屏幕范围
    if(x > LCD_WIDTH-char_width || y > LCD_HEIGHT-char_height)
        return;
    
    ch = ch - ' '; // 得到相对于空格的字符偏移量
    
    // 根据字体大小选择相应的显示方式
    if(size == FONT_0806)
    {
        for(uint8_t t = 0; t < 8; t++) // 扫描8行
        {
            uint8_t temp = ascii_0806[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 6; i++) // 一行6个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                else // 没有数据的点，显示背景色
                    LCD_Draw_Point(x + i, y + t, bg_color);
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_1206)
    {
        for(uint8_t t = 0; t < 12; t++) // 扫描12行
        {
            uint8_t temp = ascii_1206[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 6; i++) // 一行6个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                else // 没有数据的点，显示背景色
                    LCD_Draw_Point(x + i, y + t, bg_color);
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_1608)
    {
        for(uint8_t t = 0; t < 16; t++) // 扫描16行
        {
            uint8_t temp = ascii_1608[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 8; i++) // 一行8个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                else // 没有数据的点，显示背景色
                    LCD_Draw_Point(x + i, y + t, bg_color);
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_2412)
    {
        for(uint8_t t = 0; t < 24; t++) // 扫描24行
        {
            uint8_t temp1 = ascii_2412[ch][t*2];     // 获取字符数据的高字节
            uint8_t temp2 = ascii_2412[ch][t*2+1];   // 获取字符数据的低字节
            
            // 处理第一个字节的8个点
            for(uint8_t i = 0; i < 8; i++)
            {
                if(temp1 & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                else // 没有数据的点，显示背景色
                    LCD_Draw_Point(x + i, y + t, bg_color);
                
                temp1 <<= 1; // 数据左移1位
            }
            
            // 处理第二个字节的4个点（注意：前面4位是有效的）
            for(uint8_t i = 0; i < 4; i++)
            {
                if(temp2 & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + 8 + i, y + t, color);
                else // 没有数据的点，显示背景色
                    LCD_Draw_Point(x + 8 + i, y + t, bg_color);
                
                temp2 <<= 1; // 数据左移1位
            }
        }
    }
    else
    {
        // 未实现的字体大小，用矩形框内部显示字符编码
        // 填充背景色
        LCD_Fill_Rect(x, y, x + char_width - 1, y + char_height - 1, bg_color);
        
        // 绘制矩形边框
        LCD_Draw_Line(x, y, x + char_width - 1, y, color);
        LCD_Draw_Line(x, y, x, y + char_height - 1, color);
        LCD_Draw_Line(x, y + char_height - 1, x + char_width - 1, y + char_height - 1, color);
        LCD_Draw_Line(x + char_width - 1, y, x + char_width - 1, y + char_height - 1, color);
        
        // 在矩形中间位置显示字符识别码
        ch += ' '; // 恢复原始字符
        if(ch >= '0' && ch <= '9') {
            // 数字字符，绘制对角线以识别
            LCD_Draw_Line(x, y, x + char_width - 1, y + char_height - 1, color);
        } else if(ch >= 'A' && ch <= 'Z') {
            // 大写字母，绘制水平线识别
            LCD_Draw_Line(x, y + char_height/2, x + char_width - 1, y + char_height/2, color);
        } else if(ch >= 'a' && ch <= 'z') {
            // 小写字母，绘制垂直线识别
            LCD_Draw_Line(x + char_width/2, y, x + char_width/2, y + char_height - 1, color);
        }
    }
}

/* 显示ASCII字符串 */
void LCD_Show_String(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t size)
{
    // 根据字体大小确定宽度
    uint16_t char_width;
    uint16_t char_height = size;
    
    if(size == FONT_0806) {
        char_width = 6;
    } else if(size == FONT_1206) {
        char_width = 6;
    } else if(size == FONT_1608) {
        char_width = 8;
    } else if(size == FONT_2010) {
        char_width = 10;
    } else if(size == FONT_2412) {
        char_width = 12;
    } else {
        // 默认使用12x6字体
        char_width = 6;
        char_height = 12;
    }
    
    while(*str != '\0')
    {
        LCD_Show_Char(x, y, *str, color, bg_color, size);
        x += char_width; // 水平方向移动字符宽度
        if(x > LCD_WIDTH-char_width) // 自动换行
        {
            x = 0;
            y += char_height;
        }
        str++;
    }
}

/* 显示ASCII字符串(无闪烁版) */
void LCD_Show_String_NoFlicker(uint16_t x, uint16_t y, const char *old_str, const char *new_str, uint16_t color, uint16_t bg_color, uint8_t size)
{
    // 根据字体大小确定宽度
    uint16_t char_width;
    uint16_t char_height = size;
    
    // 确定字符宽度
    if(size == FONT_0806) {
        char_width = 6;
    } else if(size == FONT_1206) {
        char_width = 6;
    } else if(size == FONT_1608) {
        char_width = 8;
    } else if(size == FONT_2010) {
        char_width = 10;
    } else if(size == FONT_2412) {
        char_width = 12;
    } else {
        // 默认使用12x6字体
        char_width = 6;
        char_height = 12;
    }
    
    uint16_t pos_x = x; // 起始位置
    
    // 处理较长的字符串
    size_t old_len = old_str ? strlen(old_str) : 0;
    size_t new_len = new_str ? strlen(new_str) : 0;
    
    // 先显示共同部分，逐字符对比更新
    size_t min_len = (old_len < new_len) ? old_len : new_len;
    
    for (size_t i = 0; i < min_len; i++) {
        if (old_str[i] != new_str[i]) {
            // 字符变化，更新这一字符
            LCD_Show_Char(pos_x, y, new_str[i], color, bg_color, size);
        }
        pos_x += char_width; // 移动到下一字符位置
    }
    
    // 如果新字符串更长，显示额外的部分
    if (new_len > old_len) {
        for (size_t i = min_len; i < new_len; i++) {
            LCD_Show_Char(pos_x, y, new_str[i], color, bg_color, size);
            pos_x += char_width;
        }
    }
    // 如果旧字符串更长，用背景色擦除多余的部分
    else if (old_len > new_len) {
        // 使用背景色填充剩余区域
        LCD_Fill_Rect(pos_x, y, pos_x + (old_len - new_len) * char_width - 1, y + char_height - 1, bg_color);
    }
}

/* 显示数值（无闪烁优化版） */
void LCD_Show_Value(uint16_t x, uint16_t y, const char *old_value, const char *new_value, 
                    uint16_t color, uint16_t bg_color, uint8_t size)
{
    LCD_Show_String_NoFlicker(x, y, old_value, new_value, color, bg_color, size);
}

/* 显示ASCII字符(无背景版) - 只绘制字符前景，不绘制背景 */
void LCD_Show_Char_NoBG(uint16_t x, uint16_t y, uint8_t ch, uint16_t color, uint8_t size)
{
    // 根据字体大小确定宽度
    uint16_t char_width;
    uint16_t char_height = size;
    
    if(size == FONT_0806) {
        char_width = 6;
    } else if(size == FONT_1206) {
        char_width = 6;
    } else if(size == FONT_1608) {
        char_width = 8;
    } else if(size == FONT_2010) {
        char_width = 10;
    } else if(size == FONT_2412) {
        char_width = 12;
    } else {
        // 默认使用12x6字体
        char_width = 6;
        char_height = 12;
    }
    
    // 检查坐标是否超出屏幕范围
    if(x > LCD_WIDTH-char_width || y > LCD_HEIGHT-char_height)
        return;
    
    ch = ch - ' '; // 得到相对于空格的字符偏移量
    
    // 根据字体大小选择相应的显示方式
    if(size == FONT_0806)
    {
        for(uint8_t t = 0; t < 8; t++) // 扫描8行
        {
            uint8_t temp = ascii_0806[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 6; i++) // 一行6个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                // 无背景版只绘制前景，忽略背景
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_1206)
    {
        for(uint8_t t = 0; t < 12; t++) // 扫描12行
        {
            uint8_t temp = ascii_1206[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 6; i++) // 一行6个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                // 无背景版只绘制前景，忽略背景
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_1608)
    {
        for(uint8_t t = 0; t < 16; t++) // 扫描16行
        {
            uint8_t temp = ascii_1608[ch][t]; // 获取字符的数据
            for(uint8_t i = 0; i < 8; i++) // 一行8个点
            {
                if(temp & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                // 无背景版只绘制前景，忽略背景
                
                temp <<= 1; // 数据左移1位
            }
        }
    }
    else if(size == FONT_2412)
    {
        for(uint8_t t = 0; t < 24; t++) // 扫描24行
        {
            uint8_t temp1 = ascii_2412[ch][t*2];     // 获取字符数据的高字节
            uint8_t temp2 = ascii_2412[ch][t*2+1];   // 获取字符数据的低字节
            
            // 处理第一个字节的8个点
            for(uint8_t i = 0; i < 8; i++)
            {
                if(temp1 & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + i, y + t, color);
                // 无背景版只绘制前景，忽略背景
                
                temp1 <<= 1; // 数据左移1位
            }
            
            // 处理第二个字节的4个点（注意：前面4位是有效的）
            for(uint8_t i = 0; i < 4; i++)
            {
                if(temp2 & 0x80) // 有数据的点，显示字符颜色
                    LCD_Draw_Point(x + 8 + i, y + t, color);
                // 无背景版只绘制前景，忽略背景
                
                temp2 <<= 1; // 数据左移1位
            }
        }
    }
    // 其他字体大小直接忽略
}

/* 显示ASCII字符串(无背景版) */
void LCD_Show_String_NoBG(uint16_t x, uint16_t y, const char *str, uint16_t color, uint8_t size)
{
    // 根据字体大小确定宽度
    uint16_t char_width;
    uint16_t char_height = size;
    
    if(size == FONT_0806) {
        char_width = 6;
    } else if(size == FONT_1206) {
        char_width = 6;
    } else if(size == FONT_1608) {
        char_width = 8;
    } else if(size == FONT_2010) {
        char_width = 10;
    } else if(size == FONT_2412) {
        char_width = 12;
    } else {
        // 默认使用12x6字体
        char_width = 6;
        char_height = 12;
    }
    
    while(*str != '\0')
    {
        LCD_Show_Char_NoBG(x, y, *str, color, size);
        x += char_width; // 水平方向移动字符宽度
        if(x > LCD_WIDTH-char_width) // 自动换行
        {
            x = 0;
            y += char_height;
        }
        str++;
    }
}

/* 显示时间样例 */
void LCD_Show_Time(uint16_t x, uint16_t y, uint8_t hour, uint8_t min, uint8_t sec, uint16_t color, uint16_t bg_color, uint8_t size)
{
    char time_str[9];
    sprintf(time_str, "%02d:%02d:%02d", hour, min, sec);
    LCD_Show_String(x, y, time_str, color, bg_color, size);
}

/* 波形图结构体及相关变量 */
#define WAVE_MAX_POINTS 220   // 波形图最大点数

typedef struct {
    float min_value;                     // 最小值
    float max_value;                     // 最大值
    float data[WAVE_MAX_POINTS];         // 数据点缓冲
    uint16_t data_count;                 // 当前数据点数量
    uint16_t display_start;              // 当前显示的起始位置
    uint8_t first_draw;                  // 标记是否是第一次绘制
} WaveformData;

// 全局波形图数据
static WaveformData waveform = {0};

/**
 * @brief 初始化波形图数据
 */
void LCD_Init_Waveform(void)
{
    memset(&waveform, 0, sizeof(WaveformData));
    // 初始化最大最小值为一个无效范围，确保第一个数据点会更新这些值
    waveform.min_value = 99999.0f;
    waveform.max_value = -99999.0f;
    waveform.first_draw = 1;  // 标记为第一次绘制
    waveform.data_count = 0;
    waveform.display_start = 0;
}

/**
 * @brief 添加新的电流值到波形图中
 * @param current_mA 新的电流值(mA)
 */
void LCD_Add_Current_Point(float current_mA)
{
    // 如果波形图还没有初始化，先初始化
    if (waveform.min_value > waveform.max_value) {
        LCD_Init_Waveform();
    }
    
    // 更新最大最小值
    if (current_mA < waveform.min_value) {
        waveform.min_value = current_mA;
    }
    
    if (current_mA > waveform.max_value) {
        waveform.max_value = current_mA;
    }
    
    // 添加新数据点
    if (waveform.data_count < WAVE_MAX_POINTS) {
        // 如果缓冲区没满，直接添加
        waveform.data[waveform.data_count] = current_mA;
        waveform.data_count++;
    } else {
        // 缓冲区已满，移动所有数据并添加新点
        for (int i = 0; i < WAVE_MAX_POINTS - 1; i++) {
            waveform.data[i] = waveform.data[i + 1];
        }
        waveform.data[WAVE_MAX_POINTS - 1] = current_mA;
        
        // 调整显示起始位置以保持滚动效果
        if (waveform.display_start > 0) {
            waveform.display_start--;
        }
    }
}

/**
 * @brief 绘制电流波形图，像示波器一样从左到右绘制，并实现连续滚动
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 波形图宽度
 * @param height 波形图高度
 */
/* 显示中文字符 (16x16点阵) */
void LCD_Show_Chinese(uint16_t x, uint16_t y, uint8_t index, uint16_t color, uint16_t bg_color)
{
    uint8_t temp, i, j;
    uint16_t x0 = x; // 保存初始x坐标
    
    // 检查坐标是否超出屏幕范围
    if(x > LCD_WIDTH-16 || y > LCD_HEIGHT-16)
        return;
    
    // 设置显示窗口
    LCD_Set_Window(x, y, x+15, y+15);
    
    // 发送RAM写入命令
    LCD_Write_Cmd(ST7789_RAMWR);
    
    uint16_t y0 = y;
    
    // 绘制16x16点阵
    for(j=0; j<32; j++)
    {
        temp = GB2312_HanZi[index][j];
        for(i=0; i<8; i++)
        {
            if(temp & 0x80) // 有数据的点，显示字符颜色
                LCD_Draw_Point(x, y, color);
            else // 没有数据的点，显示背景色
                LCD_Draw_Point(x, y, bg_color);
            
            temp <<= 1;
            x++;
        }
        
        // 每扫描8个点后，调整坐标
        if(j % 2 == 1) // 每两行完成一个16像素宽的点阵行
        {
            x = x0;     // 恢复到最左侧
            y++;         // 下一行
        }
        else
        {
            x = x0 + 8; // 移动到右半部分
        }
    }
    
    // 不需要恢复坐标，因为函数结束时x、y都是局部变量
}

void LCD_Draw_Current_Wave(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    // 如果没有数据点，不绘制
    if (waveform.data_count == 0) {
        return;
    }
    
    // 计算最大值和最小值，加上空白边距
    float min_current = waveform.min_value * 0.9f;  // 添加10%的空白
    float max_current = waveform.max_value * 1.1f;  // 添加10%的空白
    
    // 确保有一个最小范围
    if (max_current - min_current < 0.1f) {
        float center = (max_current + min_current) / 2.0f;
        max_current = center + 0.05f;
        min_current = center - 0.05f;
    }
    
    // 如果最小值是负数，调整为0
    if (min_current < 0) {
        min_current = 0;
    }
    
    // 如果是第一次绘制或者电流范围变化较大，重绘背景和坐标轴
    static float last_min_current = 0;
    static float last_max_current = 0;
    
    if (waveform.first_draw || 
        fabs(last_min_current - min_current) > 0.1f || 
        fabs(last_max_current - max_current) > 0.1f) {
        
        // 保存这次的范围值
        last_min_current = min_current;
        last_max_current = max_current;
        
        // 清除波形图区域
        LCD_Fill_Rect(x, y, x + width - 1, y + height - 1, COLOR_BLACK);
        
        // 绘制坐标轴
        LCD_Draw_Line(x, y, x, y + height - 1, COLOR_WHITE);         // Y轴（电流）
        LCD_Draw_Line(x, y + height - 1, x + width - 1, y + height - 1, COLOR_WHITE); // X轴（时间）
        
        // 绘制Y轴刻度和水平网格线
        float current_step = (max_current - min_current) / 5.0f;
        char scale_str[10];
        
        for (int i = 0; i <= 5; i++) {
            uint16_t scale_y = y + height - 1 - (i * (height - 1) / 5);
            
            // 绘制水平线（Y轴刻度线）
            LCD_Draw_Line(x, scale_y, x + 3, scale_y, COLOR_WHITE);
            
            // 绘制水平虚线
            if (i > 0 && i < 5) {
                for (int j = 0; j < width - 1; j += 2) {
                    LCD_Draw_Point(x + j, scale_y, COLOR_CYAN);  // 绘制虚线
                }
            }
            
            // 绘制电流值（Y轴刻度值）
            float scale_value = min_current + i * current_step;
            sprintf(scale_str, "%.1f", scale_value);
            LCD_Show_String(x - 20, scale_y - 4, scale_str, COLOR_WHITE, COLOR_BLACK, FONT_0806);
        }
        
        // 绘制X轴时间刻度（10秒）
        for (int i = 0; i <= 5; i++) {
            uint16_t scale_x = x + (width * i) / 5;
            // 绘制时间刻度线
            LCD_Draw_Line(scale_x, y + height - 3, scale_x, y + height, COLOR_WHITE);
            
            // 绘制时间刻度值 - 时间从左到右为0-10秒
            if (i <= 5) {
                char time_str[10];
                sprintf(time_str, "%ds", i * 2); // 0, 2, 4, 6, 8, 10秒
                LCD_Show_String(scale_x - 8, y + height + 2, time_str, COLOR_WHITE, COLOR_BLACK, FONT_0806);
            }
        }
        
        // 添加坐标轴标签
        LCD_Show_String(x - 25, y, "mA", COLOR_WHITE, COLOR_BLACK, FONT_0806);       // Y轴标签        
        waveform.first_draw = 0;  // 设置为非第一次绘制
    }
    
    // 存储网格线的Y坐标位置
    static uint16_t grid_lines[5]; // 最多5条水平网格线
    static uint8_t grid_count = 0;
    static uint8_t last_wave_drawn = 0;  // 上次波形的像素点记录
    static uint16_t last_wave_points[240][2];  // [x][0]=y坐标, [x][1]=1表示有效
    
    // 第一次运行或缩放变化时更新网格线位置
    if (grid_count == 0 || waveform.first_draw) {
        // 记录网格线位置(只需要y坐标)
        grid_count = 0;
        for (int i = 1; i < 5; i++) {
            grid_lines[grid_count++] = y + height - 1 - (i * (height - 1) / 5);
        }
        
        // 初始化时完全清除波形区域
        LCD_Fill_Rect(x + 1, y + 1, x + width - 2, y + height - 2, COLOR_BLACK);
        
        // 绘制水平网格线
        for (int i = 0; i < grid_count; i++) {
            for (int j = x + 1; j < x + width - 1; j += 2) {
                LCD_Draw_Point(j, grid_lines[i], COLOR_CYAN);
            }
        }
        
        // 清除波形记录
        memset(last_wave_points, 0, sizeof(last_wave_points));
        last_wave_drawn = 0;
    } else {
        // 仅擦除上次波形绘制的点，保留网格
        if (last_wave_drawn) {
            for (uint16_t i = 0; i < width; i++) {
                if (last_wave_points[i][1]) { // 如果这个位置有波形线
                    uint16_t px = x + 1 + i;
                    uint16_t py = last_wave_points[i][0];
                    
                    // 检查这个点是否在网格线上
                    uint8_t is_grid = 0;
                    for (int g = 0; g < grid_count; g++) {
                        if (py == grid_lines[g]) {
                            is_grid = 1;
                            break;
                        }
                    }
                    
                    // 如果在网格线上，重绘网格点，否则清除
                    if (is_grid && (i % 2 == 0)) {
                        LCD_Draw_Point(px, py, COLOR_CYAN);
                    } else {
                        LCD_Draw_Point(px, py, COLOR_BLACK);
                    }
                }
            }
        }
    }
    
    // 重置波形记录
    memset(last_wave_points, 0, sizeof(last_wave_points));
    last_wave_drawn = 1;
    
    // 计算每个像素代表的数据点数量
    float points_per_pixel = (float)waveform.data_count / (float)(width - 2);
    if (points_per_pixel < 1.0f) points_per_pixel = 1.0f;
    
    // 绘制波形
    uint16_t prev_x = 0, prev_y = 0;
    uint8_t first_point = 1;
    
    for (uint16_t i = 0; i < width - 2; i++) {
        uint16_t data_idx = (uint16_t)(i * points_per_pixel);
        if (data_idx >= waveform.data_count) break;
        
        // 计算Y坐标
        float current_val = waveform.data[data_idx];
        uint16_t current_y = y + height - 1 - (uint16_t)((current_val - min_current) * (height - 1) / (max_current - min_current));
        
        // 限制Y坐标在有效范围内
        if (current_y < y + 1) current_y = y + 1;
        if (current_y >= y + height - 1) current_y = y + height - 2;
        
        // 绘制点或线
        uint16_t current_x = x + 1 + i;
        
        if (!first_point) {
            // 连接当前点和上一个点
            // 使用布莱森汇线算法绘制线段并记录每个点
            int dx = abs(current_x - prev_x);
            int dy = abs(current_y - prev_y);
            int sx = prev_x < current_x ? 1 : -1;
            int sy = prev_y < current_y ? 1 : -1;
            int err = dx - dy;
            int e2;
            
            uint16_t temp_x = prev_x;
            uint16_t temp_y = prev_y;
            
            while (1) {
                // 绘制当前点
                LCD_Draw_Point(temp_x, temp_y, COLOR_YELLOW);
                
                // 记录这个波形点位置
                if (temp_x >= x + 1 && temp_x < x + width - 1) {
                    uint16_t index = temp_x - (x + 1);
                    if (index < width) {
                        last_wave_points[index][0] = temp_y; // 记录y坐标
                        last_wave_points[index][1] = 1;    // 标记为有效
                    }
                }
                
                // 到达终点时退出
                if (temp_x == current_x && temp_y == current_y) break;
                
                e2 = 2 * err;
                if (e2 > -dy) { err -= dy; temp_x += sx; }
                if (e2 < dx) { err += dx; temp_y += sy; }
            }
        } else {
            // 第一个点
            first_point = 0;
            LCD_Draw_Point(current_x, current_y, COLOR_YELLOW);
            
            // 记录这个点
            uint16_t index = current_x - (x + 1);
            if (index < width) {
                last_wave_points[index][0] = current_y;
                last_wave_points[index][1] = 1;
            }
        }
        
        // 记录当前点作为下一个线段的起点
        prev_x = current_x;
        prev_y = current_y;
    }
}

/* LCD任务主函数 */
void LCD_Task(void *pvParameters)
{
    printf("LCD start run...\r\n");
    
    // 初始化LCD
    LCD_Init();
    
    // 清屏，黑色背景
    LCD_Clear(COLOR_BLACK);
    
    // 计算中心位置 - 以显示英文
    uint16_t text_x = (LCD_WIDTH - 12*7)/2;  // 居中显示 "ST7789" 7个字符
    uint16_t text_y = (LCD_HEIGHT - 12)/2 - 50; // 在屏幕中心偏上一点
    
    // 显示中文字符 "最大", 注意LCD高度为240，调整y坐标到200以内
    LCD_Show_Chinese(10, 260, 0, COLOR_WHITE, COLOR_BLACK); // 显示"最"
    LCD_Show_Chinese(28, 260, 1, COLOR_WHITE, COLOR_BLACK); // 显示"大"
    LCD_Show_Chinese(80, 260, 0, COLOR_WHITE, COLOR_BLACK); // 显示"最"
    LCD_Show_Chinese(98, 260, 2, COLOR_WHITE, COLOR_BLACK); // 显示"小"
    LCD_Show_Chinese(150, 260, 3, COLOR_WHITE, COLOR_BLACK); // 显示"时"
    LCD_Show_Chinese(168, 260, 4, COLOR_WHITE, COLOR_BLACK); // 显示"间"
    
    // // 显示多种字体大小的标题
    // LCD_Show_String((LCD_WIDTH - 16*2)/2, text_y - 20, "Current ", COLOR_WHITE, COLOR_BLACK, FONT_1608);
    // LCD_Show_String((LCD_WIDTH - 16*2)/2, text_y - 40, "Voltage ", COLOR_WHITE, COLOR_BLACK, FONT_1608);


    
    // 显示各种字体大小示例
    LCD_Draw_Line(0, text_y -30, LCD_WIDTH-1, text_y -30, COLOR_WHITE); // 水平分隔线
    
    // LCD_Show_String(5, text_y + 100, "0806:", COLOR_RED, COLOR_BLACK, FONT_0806);
    // LCD_Show_String(50, text_y + 100, "ABCDEF123", COLOR_RED, COLOR_BLACK, FONT_0806);
    
    // LCD_Show_String(5, text_y + 115, "1206:", COLOR_YELLOW, COLOR_BLACK, FONT_1206);
    // LCD_Show_String(50, text_y + 115, "ABCDEF123", COLOR_YELLOW, COLOR_BLACK, FONT_1206);
    
    // LCD_Show_String(5, text_y + 135, "1608:", COLOR_GREEN, COLOR_BLACK, FONT_1608);
    // LCD_Show_String(50, text_y + 135, "ABCDEF", COLOR_GREEN, COLOR_BLACK, FONT_1608);
    
    // LCD_Show_String(5, text_y + 160, "2412:", COLOR_CYAN, COLOR_BLACK, FONT_2412);
    // LCD_Show_String(50, text_y + 160, "ABCD", COLOR_CYAN, COLOR_BLACK, FONT_2412);

    // 初始化时间显示变量
    uint8_t hour = 15; // 初始化为当前时间 15点
    uint8_t min = 23;
    uint8_t sec = 42;
    
    // 要清除的时间区域的左上角坐标
    uint16_t time_x = LCD_WIDTH - 70; // 右上角时间显示的X坐标
    uint16_t time_y = 5;              // 右上角时间显示的Y坐标
    
    // 黑色矩形清除时间区域（防止自刷时文字重叠）
    LCD_Fill_Rect(time_x - 5, time_y - 2, LCD_WIDTH - 2, time_y + 14, COLOR_BLACK);
    
    // 显示初始时间
    LCD_Show_Time(time_x, time_y, hour, min, sec, COLOR_WHITE, COLOR_BLACK, FONT_1206);
    
    // 输出调试信息
    printf("LCD show  end\r\n");
    
    while(1)
    {
        // 更新时间
        sec++;
        if(sec >= 60) { sec = 0; min++; }
        if(min >= 60) { min = 0; hour++; }
        if(hour >= 24) { hour = 0; }
        // // 清除上一次显示的时间
        // LCD_Fill_Rect(time_x - 5, time_y - 2, LCD_WIDTH - 2, time_y + 14, COLOR_BLACK);
        
        // // 显示新的时间
        // LCD_Show_Time(time_x, time_y, hour, min, sec, COLOR_WHITE, COLOR_BLACK, FONT_1206);
        
        // 每秒更新一次
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}