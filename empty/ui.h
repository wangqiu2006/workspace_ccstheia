#ifndef UI_H_
#define UI_H_

#include "st7789.h"
#include <stdint.h>
#include <stdbool.h>

// ========== 扩展颜色定义 ==========
#define YELLOW      0xFFE0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define ORANGE      0xFD20
#define PURPLE      0x780F
#define GRAY        0x8410
#define DARKGRAY    0x4208
#define LIGHTGRAY   0xC618
#define BROWN       0xA145
#define PINK        0xF81F

// ========== 字体参数 ==========
#define FONT_WIDTH  6
#define FONT_HEIGHT 8

// ========== 基础绘图 API ==========
// 像素操作
void UI_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

// 线条绘制
void UI_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void UI_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void UI_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

// 矩形绘制
void UI_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void UI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void UI_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void UI_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);

// 圆形绘制
void UI_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);
void UI_FillCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);

// 三角形绘制
void UI_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void UI_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

// ========== 文字渲染 API ==========
// 单字符绘制（支持缩放 1x/2x/3x）
void UI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);

// 字符串绘制（支持缩放）
void UI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);

// 格式化输出（类似 printf）
void UI_Printf(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, uint8_t scale, const char *fmt, ...);

// ========== UI 组件 API ==========
// 按钮（带边框、背景、文字居中）
void UI_DrawButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const char *label, uint16_t bg, uint16_t fg, uint16_t border, bool pressed);

// 进度条（百分比 0-100）
void UI_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                        uint8_t percent, uint16_t fg, uint16_t bg, uint16_t border);

// 窗口/面板（带标题栏）
void UI_DrawWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const char *title, uint16_t title_bg, uint16_t title_fg, uint16_t body_bg);

// 滑动条（当前值/最大值）
void UI_DrawSlider(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   uint16_t value, uint16_t max_value, uint16_t fg, uint16_t bg, uint16_t handle);

// 复选框（勾选状态）
void UI_DrawCheckbox(uint16_t x, uint16_t y, uint16_t size, bool checked, uint16_t fg, uint16_t bg);

// 单选按钮（选中状态）
void UI_DrawRadio(uint16_t x, uint16_t y, uint16_t size, bool selected, uint16_t fg, uint16_t bg);

// ========== 高级绘图 API ==========
// 位图绘制（RGB565 格式）
void UI_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *bitmap);

// 单色位图（1 bit per pixel，1=前景色，0=背景色）
void UI_DrawMonoBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       const uint8_t *bitmap, uint16_t fg, uint16_t bg);

#endif /* UI_H_ */
