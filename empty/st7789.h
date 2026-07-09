#ifndef ST7789_H_
#define ST7789_H_

#include "ti_msp_dl_config.h"

// --- 自动对接 SysConfig 生成的宏 ---
#define LCD_CS_CLR()   DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_CS_PIN)
#define LCD_CS_SET()   DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_CS_PIN)

#define LCD_DC_CLR()   DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_DC_PIN) 
#define LCD_DC_SET()   DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_DC_PIN) 

#define LCD_RES_CLR()  DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_RES_PIN)
#define LCD_RES_SET()  DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_RES_PIN)

#define LCD_BLK_CLR()  DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_BLK_PIN)
#define LCD_BLK_SET()  DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_BLK_PIN)

// --- 屏幕分辨率 ---
#define LCD_WIDTH   240
#define LCD_HEIGHT  320

// --- 常规 RGB565 颜色 ---
#define WHITE       0xFFFF
#define BLACK       0x0000
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F

// --- 函数声明 ---
void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Write_Cmd(uint8_t cmd);
void LCD_Write_Data(uint8_t data);
void LCD_Write_Data16(uint16_t data);

#endif /* ST7789_H_ */