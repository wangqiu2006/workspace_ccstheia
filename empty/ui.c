#include "ui.h"
#include "ui_font.h"
#include <stdarg.h>
#include <stdio.h>

// =============================================
// 超时保护：防止 SPI 未初始化时死循环
// =============================================
#define SPI_TIMEOUT_MAX  100000  // 约 3ms @ 32MHz

// =============================================
// 内部辅助：交换两个 uint16_t
// =============================================
static inline void _swap(uint16_t *a, uint16_t *b) {
    uint16_t t = *a; *a = *b; *b = t;
}

// =============================================
// 像素操作
// =============================================
void UI_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    LCD_Address_Set(x, y, x, y);
    LCD_Write_Data16(color);
}

// =============================================
// 水平线（极速版，直接调用 LCD_Address_Set）
// =============================================
void UI_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    LCD_Address_Set(x, y, x + w - 1, y);
    LCD_CS_CLR();
    LCD_DC_SET();

    while (w--) {
        volatile uint32_t timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;  // 超时退出，避免死锁
        DL_SPI_transmitData8(SPI_LCD_INST, hi);

        timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, lo);
    }

    volatile uint32_t timeout = SPI_TIMEOUT_MAX;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --timeout);
    LCD_CS_SET();
}

// =============================================
// 垂直线（极速版）
// =============================================
void UI_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    LCD_Address_Set(x, y, x, y + h - 1);
    LCD_CS_CLR();
    LCD_DC_SET();

    while (h--) {
        volatile uint32_t timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, hi);

        timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, lo);
    }

    volatile uint32_t timeout = SPI_TIMEOUT_MAX;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --timeout);
    LCD_CS_SET();
}

// =============================================
// 任意直线（Bresenham 算法）
// =============================================
void UI_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    // 快速路径：纯水平/垂直线
    if (y0 == y1) { UI_DrawFastHLine(x0 < x1 ? x0 : x1, y0, (x1 > x0 ? x1 - x0 : x0 - x1) + 1, color); return; }
    if (x0 == x1) { UI_DrawFastVLine(x0, y0 < y1 ? y0 : y1, (y1 > y0 ? y1 - y0 : y0 - y1) + 1, color); return; }

    int16_t dx  =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy  = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sx  = (x0 < x1) ? 1 : -1;
    int16_t sy  = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy;

    while (1) {
        UI_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// =============================================
// 矩形（空心）
// =============================================
void UI_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (w == 0 || h == 0) return;
    UI_DrawFastHLine(x, y, w, color);           // 上边
    UI_DrawFastHLine(x, y + h - 1, w, color);   // 下边
    UI_DrawFastVLine(x, y, h, color);           // 左边
    UI_DrawFastVLine(x + w - 1, y, h, color);   // 右边
}

// =============================================
// 矩形（实心）—— 极速版
// =============================================
void UI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    uint32_t total = (uint32_t)w * h;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    LCD_Address_Set(x, y, x + w - 1, y + h - 1);
    LCD_CS_CLR();
    LCD_DC_SET();

    while (total--) {
        volatile uint32_t timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, hi);

        timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, lo);
    }

    volatile uint32_t timeout = SPI_TIMEOUT_MAX;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --timeout);
    LCD_CS_SET();
}

// =============================================
// 圆角矩形辅助：绘制圆角的一个四分之一圆弧
// =============================================
static void _drawCircleHelper(int16_t cx, int16_t cy, int16_t r, uint8_t corner, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (corner & 0x4) { UI_DrawPixel(cx + x, cy + y, color); UI_DrawPixel(cx + y, cy + x, color); }
        if (corner & 0x2) { UI_DrawPixel(cx + x, cy - y, color); UI_DrawPixel(cx + y, cy - x, color); }
        if (corner & 0x8) { UI_DrawPixel(cx - y, cy + x, color); UI_DrawPixel(cx - x, cy + y, color); }
        if (corner & 0x1) { UI_DrawPixel(cx - y, cy - x, color); UI_DrawPixel(cx - x, cy - y, color); }
    }
}

// =============================================
// 圆角矩形（空心）
// =============================================
void UI_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
    if (r == 0) { UI_DrawRect(x, y, w, h, color); return; }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    // 四条边（中间部分）
    UI_DrawFastHLine(x + r, y, w - 2 * r, color);           // 上
    UI_DrawFastHLine(x + r, y + h - 1, w - 2 * r, color);   // 下
    UI_DrawFastVLine(x, y + r, h - 2 * r, color);           // 左
    UI_DrawFastVLine(x + w - 1, y + r, h - 2 * r, color);   // 右

    // 四个圆角
    _drawCircleHelper(x + r, y + r, r, 1, color);               // 左上
    _drawCircleHelper(x + w - r - 1, y + r, r, 2, color);       // 右上
    _drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color); // 右下
    _drawCircleHelper(x + r, y + h - r - 1, r, 8, color);       // 左下
}

// =============================================
// 圆角矩形（实心）
// =============================================
static void _fillCircleHelper(int16_t cx, int16_t cy, int16_t r, uint8_t corner, int16_t delta, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (corner & 0x1) {
            UI_DrawFastVLine(cx + x, cy - y, 2 * y + 1 + delta, color);
            UI_DrawFastVLine(cx + y, cy - x, 2 * x + 1 + delta, color);
        }
        if (corner & 0x2) {
            UI_DrawFastVLine(cx - x, cy - y, 2 * y + 1 + delta, color);
            UI_DrawFastVLine(cx - y, cy - x, 2 * x + 1 + delta, color);
        }
    }
}

void UI_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
    if (r == 0) { UI_FillRect(x, y, w, h, color); return; }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    // 中间矩形
    UI_FillRect(x + r, y, w - 2 * r, h, color);

    // 填充四个圆角
    _fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    _fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

// =============================================
// 圆形（空心）—— 中点圆算法
// =============================================
void UI_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    UI_DrawPixel(cx, cy + r, color);
    UI_DrawPixel(cx, cy - r, color);
    UI_DrawPixel(cx + r, cy, color);
    UI_DrawPixel(cx - r, cy, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        UI_DrawPixel(cx + x, cy + y, color);
        UI_DrawPixel(cx - x, cy + y, color);
        UI_DrawPixel(cx + x, cy - y, color);
        UI_DrawPixel(cx - x, cy - y, color);
        UI_DrawPixel(cx + y, cy + x, color);
        UI_DrawPixel(cx - y, cy + x, color);
        UI_DrawPixel(cx + y, cy - x, color);
        UI_DrawPixel(cx - y, cy - x, color);
    }
}

// =============================================
// 圆形（实心）—— 通过水平线填充
// =============================================
void UI_FillCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color) {
    UI_DrawFastVLine(cx, cy - r, 2 * r + 1, color);

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        UI_DrawFastVLine(cx + x, cy - y, 2 * y + 1, color);
        UI_DrawFastVLine(cx - x, cy - y, 2 * y + 1, color);
        UI_DrawFastVLine(cx + y, cy - x, 2 * x + 1, color);
        UI_DrawFastVLine(cx - y, cy - x, 2 * x + 1, color);
    }
}

// =============================================
// 三角形（空心）
// =============================================
void UI_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    UI_DrawLine(x0, y0, x1, y1, color);
    UI_DrawLine(x1, y1, x2, y2, color);
    UI_DrawLine(x2, y2, x0, y0, color);
}

// =============================================
// 三角形（实心）—— 扫描线填充
// =============================================
void UI_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    int16_t a, b, y, last;

    // 按 y 坐标排序（冒泡排序）
    if (y0 > y1) { _swap(&y0, &y1); _swap(&x0, &x1); }
    if (y1 > y2) { _swap(&y1, &y2); _swap(&x1, &x2); }
    if (y0 > y1) { _swap(&y0, &y1); _swap(&x0, &x1); }

    if (y0 == y2) { // 退化成一条线
        a = b = x0;
        if (x1 < a)      a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a)      a = x2;
        else if (x2 > b) b = x2;
        UI_DrawFastHLine(a, y0, b - a + 1, color);
        return;
    }

    int16_t dx01 = x1 - x0, dy01 = y1 - y0;
    int16_t dx02 = x2 - x0, dy02 = y2 - y0;
    int16_t dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    // 上半部分（y0 到 y1）
    if (y1 == y0) last = y1;
    else          last = y1 - 1;

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy02;
        b = x0 + sb / dy01;
        sa += dx02;
        sb += dx01;
        if (a > b) _swap((uint16_t*)&a, (uint16_t*)&b);
        UI_DrawFastHLine(a, y, b - a + 1, color);
    }

    // 下半部分（y1 到 y2）
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) _swap((uint16_t*)&a, (uint16_t*)&b);
        UI_DrawFastHLine(a, y, b - a + 1, color);
    }
}

// =============================================
// 单字符绘制（支持缩放）
// =============================================
void UI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (c < 0x20 || c > 0x7E) c = ' '; // 超出范围显示空格
    const uint8_t *glyph = font6x8[c - 0x20];

    if (scale == 1) {
        // 无缩放快速路径
        for (uint8_t col = 0; col < FONT_WIDTH; col++) {
            uint8_t line = glyph[col];
            for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
                uint16_t color = (line & 0x01) ? fg : bg;
                UI_DrawPixel(x + col, y + row, color);
                line >>= 1;
            }
        }
    } else {
        // 缩放版本
        for (uint8_t col = 0; col < FONT_WIDTH; col++) {
            uint8_t line = glyph[col];
            for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
                uint16_t color = (line & 0x01) ? fg : bg;
                UI_FillRect(x + col * scale, y + row * scale, scale, scale, color);
                line >>= 1;
            }
        }
    }
}

// =============================================
// 字符串绘制
// =============================================
void UI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale) {
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;
    uint16_t char_width = FONT_WIDTH * scale;
    uint16_t char_height = FONT_HEIGHT * scale;

    while (*str) {
        if (*str == '\n') {
            cursor_x = x;
            cursor_y += char_height;
        } else if (*str == '\r') {
            cursor_x = x;
        } else {
            // 自动换行
            if (cursor_x + char_width > LCD_WIDTH) {
                cursor_x = x;
                cursor_y += char_height;
            }
            UI_DrawChar(cursor_x, cursor_y, *str, fg, bg, scale);
            cursor_x += char_width;
        }
        str++;
    }
}

// =============================================
// 格式化输出（类似 printf）
// =============================================
void UI_Printf(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg, uint8_t scale, const char *fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UI_DrawString(x, y, buffer, fg, bg, scale);
}

// =============================================
// 按钮（带边框 + 居中文字）
// =============================================
void UI_DrawButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const char *label, uint16_t bg, uint16_t fg, uint16_t border, bool pressed) {
    // 按下时背景加深
    uint16_t fill_color = pressed ? (bg & 0xE79C) : bg; // 简单暗化
    UI_FillRoundRect(x, y, w, h, 4, fill_color);
    UI_DrawRoundRect(x, y, w, h, 4, border);

    // 按下时偏移 1 像素，模拟按压效果
    uint8_t off = pressed ? 1 : 0;

    // 计算文字居中位置
    uint16_t str_len = 0;
    const char *p = label;
    while (*p++) str_len++;

    uint16_t text_w = str_len * FONT_WIDTH;
    uint16_t text_h = FONT_HEIGHT;
    uint16_t tx = x + (w - text_w) / 2 + off;
    uint16_t ty = y + (h - text_h) / 2 + off;
    UI_DrawString(tx, ty, label, fg, fill_color, 1);
}

// =============================================
// 进度条（0-100%）
// =============================================
void UI_DrawProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                        uint8_t percent, uint16_t fg, uint16_t bg, uint16_t border) {
    if (percent > 100) percent = 100;

    // 背景
    UI_FillRect(x, y, w, h, bg);
    // 边框
    UI_DrawRect(x, y, w, h, border);

    // 填充（内缩 1 像素）
    if (percent > 0) {
        uint16_t fill_w = (uint16_t)((w - 2) * percent / 100);
        if (fill_w > 0)
            UI_FillRect(x + 1, y + 1, fill_w, h - 2, fg);
    }

    // 百分比文字居中
    char buf[5];
    uint8_t d2 = percent / 100;
    uint8_t d1 = (percent % 100) / 10;
    uint8_t d0 = percent % 10;
    uint8_t idx = 0;
    if (d2) buf[idx++] = '0' + d2;
    if (d1 || d2) buf[idx++] = '0' + d1;
    buf[idx++] = '0' + d0;
    buf[idx++] = '%';
    buf[idx] = '\0';

    uint16_t tx = x + (w - idx * FONT_WIDTH) / 2;
    uint16_t ty = y + (h - FONT_HEIGHT) / 2;
    UI_DrawString(tx, ty, buf, WHITE, 0x0000, 1);
}

// =============================================
// 窗口/面板（带标题栏）
// =============================================
void UI_DrawWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   const char *title, uint16_t title_bg, uint16_t title_fg, uint16_t body_bg) {
    uint16_t title_h = FONT_HEIGHT + 4; // 标题栏高度

    // 标题栏
    UI_FillRect(x, y, w, title_h, title_bg);

    // 主体
    UI_FillRect(x, y + title_h, w, h - title_h, body_bg);

    // 外边框
    UI_DrawRect(x, y, w, h, GRAY);

    // 标题文字（左对齐，内边距 4px）
    UI_DrawString(x + 4, y + 2, title, title_fg, title_bg, 1);

    // 标题栏底部分隔线
    UI_DrawFastHLine(x, y + title_h - 1, w, GRAY);
}

// =============================================
// 滑动条
// =============================================
void UI_DrawSlider(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   uint16_t value, uint16_t max_value, uint16_t fg, uint16_t bg, uint16_t handle) {
    uint16_t track_y = y + h / 2 - 2;
    uint16_t track_h = 4;
    uint16_t knob_r  = h / 2;

    // 轨道背景
    UI_FillRoundRect(x, track_y, w, track_h, 2, bg);
    UI_DrawRoundRect(x, track_y, w, track_h, 2, GRAY);

    // 已走过部分（高亮色）
    if (max_value > 0 && value > 0) {
        uint16_t filled = (uint16_t)((uint32_t)(w - 2) * value / max_value);
        if (filled > 0)
            UI_FillRoundRect(x + 1, track_y + 1, filled, track_h - 2, 2, fg);
    }

    // 滑块旋钮
    uint16_t knob_x = (max_value > 0)
        ? (uint16_t)(x + (uint32_t)(w - 1) * value / max_value)
        : x;
    UI_FillCircle(knob_x, y + h / 2, knob_r, handle);
    UI_DrawCircle(knob_x, y + h / 2, knob_r, GRAY);
}

// =============================================
// 复选框
// =============================================
void UI_DrawCheckbox(uint16_t x, uint16_t y, uint16_t size, bool checked, uint16_t fg, uint16_t bg) {
    // 外框
    UI_FillRect(x, y, size, size, bg);
    UI_DrawRect(x, y, size, size, fg);

    if (checked) {
        // 勾号：从左下到中间，再到右上
        uint16_t mx = x + size / 3;
        uint16_t my = y + size * 2 / 3;
        uint16_t rx = x + size - 2;
        uint16_t ry = y + 2;
        uint16_t cx = x + size / 2 - 1;
        uint16_t cy = y + size - 3;
        UI_DrawLine(x + 2, my, cx, cy, fg);
        UI_DrawLine(cx, cy, rx, ry, fg);
        // 加粗勾号（向上偏移一像素再画一次）
        UI_DrawLine(x + 2, my - 1, cx, cy - 1, fg);
        UI_DrawLine(cx, cy - 1, rx, ry - 1, fg);
    }
}

// =============================================
// 单选按钮
// =============================================
void UI_DrawRadio(uint16_t x, uint16_t y, uint16_t size, bool selected, uint16_t fg, uint16_t bg) {
    uint16_t r = size / 2;
    uint16_t cx = x + r;
    uint16_t cy = y + r;

    // 外圆
    UI_FillCircle(cx, cy, r, bg);
    UI_DrawCircle(cx, cy, r, fg);

    // 选中时填充内圆
    if (selected && r > 2) {
        UI_FillCircle(cx, cy, r - 3, fg);
    }
}

// =============================================
// RGB565 位图绘制
// =============================================
void UI_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *bitmap) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    LCD_Address_Set(x, y, x + w - 1, y + h - 1);
    LCD_CS_CLR();
    LCD_DC_SET();

    uint32_t total = (uint32_t)w * h;
    for (uint32_t i = 0; i < total; i++) {
        uint16_t color = bitmap[i];
        uint8_t hi = color >> 8;
        uint8_t lo = color & 0xFF;

        volatile uint32_t timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, hi);

        timeout = SPI_TIMEOUT_MAX;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --timeout);
        if (!timeout) break;
        DL_SPI_transmitData8(SPI_LCD_INST, lo);
    }

    volatile uint32_t timeout = SPI_TIMEOUT_MAX;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --timeout);
    LCD_CS_SET();
}

// =============================================
// 单色位图绘制（1 bit per pixel）
// =============================================
void UI_DrawMonoBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       const uint8_t *bitmap, uint16_t fg, uint16_t bg) {
    uint16_t byte_width = (w + 7) / 8; // 每行占用的字节数

    for (uint16_t row = 0; row < h; row++) {
        for (uint16_t col = 0; col < w; col++) {
            uint16_t byte_idx = row * byte_width + col / 8;
            uint8_t bit_mask = 0x80 >> (col % 8);
            uint16_t color = (bitmap[byte_idx] & bit_mask) ? fg : bg;
            UI_DrawPixel(x + col, y + row, color);
        }
    }
}

