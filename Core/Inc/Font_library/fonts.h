/**
 * @file fonts.h
 * @brief LCD字体库头文件
 */

#ifndef __FONTS_H
#define __FONTS_H

#include "main.h"

/* 字体参数定义 */
#define FONT_0806      8 /* 8x6 像素字体 */
#define FONT_1206     12 /* 12x6 像素字体 */
#define FONT_1608     16 /* 16x8 像素字体 */
#define FONT_2010     20 /* 20x10 像素字体 */
#define FONT_2412     24 /* 24x12 像素字体 */

/* 字体数据声明 */
extern const uint8_t ascii_0806[95][8];    /* ASCII 字体 8x6 */
extern const uint8_t ascii_1206[95][12];   /* ASCII 字体 12x6 */
extern const uint8_t ascii_1608[95][16];   /* ASCII 字体 16x8 */
extern const uint8_t ascii_2412[95][48];   /* ASCII 字体 24x12 */

#endif /* __FONTS_H */
