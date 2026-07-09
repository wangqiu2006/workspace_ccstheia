/*
 * ST7789 UI 库使用示例
 * 演示如何使用像素级 UI 库绘制界面
 */

#include "ti_msp_dl_config.h"
#include "st7789.h"
#include "ui.h"

int main(void) {
    // 1. 初始化硬件
    SYSCFG_DL_init();

    // 2. 初始化屏幕
    LCD_Init();

    // 3. 绘制欢迎界面
    LCD_Clear(BLACK);

    // 顶部标题栏
    UI_FillRect(0, 0, 240, 40, BLUE);
    UI_DrawString(35, 12, "MSPM0G3507", WHITE, BLUE, 2);

    // 欢迎窗口
    UI_DrawWindow(10, 50, 220, 120, "Welcome", PURPLE, WHITE, DARKGRAY);
    UI_DrawString(20, 75, "UI Library v1.0", CYAN, DARKGRAY, 1);
    UI_DrawString(20, 95, "Resolution:", WHITE, DARKGRAY, 1);
    UI_Printf(20, 110, YELLOW, DARKGRAY, 1, "  240 x 320 px");
    UI_DrawString(20, 130, "Driver: ST7789", GREEN, DARKGRAY, 1);

    // 按钮
    UI_DrawButton(20, 185, 90, 40, "Start", GREEN, WHITE, GRAY, false);
    UI_DrawButton(130, 185, 90, 40, "Setup", BLUE, WHITE, GRAY, false);

    // 状态指示
    UI_DrawCheckbox(20, 245, 18, true, GREEN, BLACK);
    UI_DrawString(45, 245, "System Ready", WHITE, BLACK, 1);

    // 进度条
    UI_DrawString(20, 275, "Loading...", LIGHTGRAY, BLACK, 1);
    UI_DrawProgressBar(20, 290, 200, 20, 85, GREEN, DARKGRAY, GRAY);

    delay_cycles(CPUCLK_FREQ * 3); // 显示 3 秒

    // 4. 进入动态演示循环
    uint8_t progress = 0;
    uint16_t temp = 25;
    bool wifi_on = true;

    while (1) {
        // 绘制仪表盘
        LCD_Clear(BLACK);

        // 标题
        UI_FillRect(0, 0, 240, 35, BLUE);
        UI_DrawString(50, 10, "Dashboard", WHITE, BLUE, 2);

        // 温度显示
        UI_DrawWindow(10, 45, 105, 80, "Temp", CYAN, WHITE, BLACK);
        UI_Printf(20, 70, RED, BLACK, 3, "%dC", temp);

        // 状态面板
        UI_DrawWindow(125, 45, 105, 80, "Status", CYAN, WHITE, BLACK);
        UI_DrawCheckbox(135, 70, 15, wifi_on, GREEN, BLACK);
        UI_DrawString(155, 70, "WiFi", WHITE, BLACK, 1);
        UI_DrawRadio(135, 95, 15, true, BLUE, BLACK);
        UI_DrawString(155, 95, "Active", WHITE, BLACK, 1);

        // 进度条
        UI_DrawString(10, 140, "CPU Load:", WHITE, BLACK, 1);
        UI_DrawProgressBar(10, 160, 220, 25, progress,
                          progress > 75 ? RED : (progress > 50 ? YELLOW : GREEN),
                          DARKGRAY, GRAY);

        // 滑动条
        UI_DrawString(10, 200, "Brightness:", WHITE, BLACK, 1);
        UI_DrawSlider(10, 220, 220, 20, progress, 100, BLUE, DARKGRAY, CYAN);

        // 控制按钮
        UI_DrawButton(10, 260, 70, 35, "Refresh", GREEN, WHITE, GRAY, false);
        UI_DrawButton(90, 260, 70, 35, "Config", BLUE, WHITE, GRAY, false);
        UI_DrawButton(170, 260, 60, 35, "Info", ORANGE, WHITE, GRAY, false);

        // 底部信息栏
        UI_DrawFastHLine(0, 305, 240, GRAY);
        UI_Printf(10, 310, LIGHTGRAY, BLACK, 1, "Time: %02d:%02d",
                 (progress / 4) % 24, (progress * 15) % 60);

        // 更新变量
        progress = (progress + 5) % 101;
        temp = 25 + (progress / 10);
        if (progress == 0) wifi_on = !wifi_on;

        delay_cycles(CPUCLK_FREQ / 2); // 每 0.5 秒更新一次
    }
}
 