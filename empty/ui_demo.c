/*
 * UI 库演示示例
 * 展示 ST7789 像素级 UI 库的各种功能
 */

#include "ti_msp_dl_config.h"
#include "st7789.h"
#include "ui.h"
#include "ui_demo.h"

// 演示场景枚举
typedef enum {
    DEMO_SHAPES,      // 基础图形
    DEMO_TEXT,        // 文字渲染
    DEMO_BUTTONS,     // 按钮与控件
    DEMO_PROGRESS,    // 进度条与滑动条
    DEMO_DASHBOARD,   // 仪表盘界面
    DEMO_COUNT
} DemoScene;

static DemoScene current_scene = DEMO_SHAPES;

// =============================================
// 场景 1: 基础图形演示
// =============================================
void demo_shapes(void) {
    LCD_Clear(BLACK);

    // 标题
    UI_DrawString(60, 5, "SHAPES DEMO", WHITE, BLACK, 2);

    // 实心矩形
    UI_FillRect(10, 40, 60, 40, RED);
    UI_DrawString(15, 90, "Rect", WHITE, BLACK, 1);

    // 圆角矩形
    UI_FillRoundRect(80, 40, 60, 40, 8, GREEN);
    UI_DrawString(85, 90, "Round", WHITE, BLACK, 1);

    // 圆形
    UI_FillCircle(185, 60, 20, BLUE);
    UI_DrawString(170, 90, "Circle", WHITE, BLACK, 1);

    // 三角形
    UI_FillTriangle(30, 120, 10, 160, 50, 160, YELLOW);
    UI_DrawString(15, 170, "Triangle", WHITE, BLACK, 1);

    // 线条
    UI_DrawLine(80, 120, 140, 160, CYAN);
    UI_DrawLine(80, 160, 140, 120, MAGENTA);
    UI_DrawString(85, 170, "Lines", WHITE, BLACK, 1);

    // 空心图形
    UI_DrawRect(160, 120, 60, 40, ORANGE);
    UI_DrawCircle(190, 200, 25, PURPLE);

    UI_DrawString(50, 300, "Press to continue", LIGHTGRAY, BLACK, 1);
}

// =============================================
// 场景 2: 文字渲染演示
// =============================================
void demo_text(void) {
    LCD_Clear(BLACK);

    UI_DrawString(65, 10, "TEXT DEMO", WHITE, BLACK, 2);

    // 不同缩放
    UI_DrawString(10, 50, "Scale 1x: Hello!", GREEN, BLACK, 1);
    UI_DrawString(10, 70, "Scale 2x:", YELLOW, BLACK, 2);
    UI_DrawString(10, 100, "Big Text", CYAN, BLACK, 3);

    // 彩色文字
    UI_DrawString(10, 150, "RED", RED, BLACK, 2);
    UI_DrawString(60, 150, "GREEN", GREEN, BLACK, 2);
    UI_DrawString(140, 150, "BLUE", BLUE, BLACK, 2);

    // Printf 格式化输出
    UI_Printf(10, 190, WHITE, BLACK, 1, "CPU: MSPM0G3507");
    UI_Printf(10, 205, WHITE, BLACK, 1, "Speed: 32 MHz");
    UI_Printf(10, 220, WHITE, BLACK, 1, "Display: 240x320");

    // 数字显示
    UI_Printf(10, 250, ORANGE, BLACK, 2, "Temp: 25 C");
    UI_Printf(10, 280, MAGENTA, BLACK, 2, "Volt: 3.3V");
}

// =============================================
// 场景 3: 按钮演示
// =============================================
void demo_buttons(void) {
    LCD_Clear(BLACK);

    UI_DrawString(50, 10, "BUTTON DEMO", WHITE, BLACK, 2);

    // 普通按钮
    UI_DrawButton(20, 50, 90, 35, "OK", GREEN, WHITE, LIGHTGRAY, false);
    UI_DrawButton(130, 50, 90, 35, "Cancel", RED, WHITE, LIGHTGRAY, false);

    // 按下状态
    UI_DrawButton(20, 100, 90, 35, "Pressed", BLUE, WHITE, GRAY, true);
    UI_DrawButton(130, 100, 90, 35, "Normal", BLUE, WHITE, GRAY, false);

    // 复选框
    UI_DrawString(10, 155, "Checkbox:", WHITE, BLACK, 1);
    UI_DrawCheckbox(100, 150, 20, true, GREEN, BLACK);
    UI_DrawCheckbox(130, 150, 20, false, GREEN, BLACK);

    // 单选按钮
    UI_DrawString(10, 185, "Radio:", WHITE, BLACK, 1);
    UI_DrawRadio(100, 180, 20, true, BLUE, BLACK);
    UI_DrawRadio(130, 180, 20, false, BLUE, BLACK);
    UI_DrawRadio(160, 180, 20, false, BLUE, BLACK);

    // 窗口面板
    UI_DrawWindow(10, 220, 220, 90, "Settings", BLUE, WHITE, DARKGRAY);
    UI_DrawString(20, 242, "Volume: High", WHITE, DARKGRAY, 1);
    UI_DrawString(20, 257, "Brightness: 80%", WHITE, DARKGRAY, 1);
    UI_DrawString(20, 272, "Mode: Auto", WHITE, DARKGRAY, 1);
}

// =============================================
// 场景 4: 进度条与滑动条演示
// =============================================
void demo_progress(void) {
    LCD_Clear(BLACK);

    UI_DrawString(30, 10, "PROGRESS DEMO", WHITE, BLACK, 2);

    // 不同百分比的进度条
    UI_DrawString(10, 50, "Progress Bars:", CYAN, BLACK, 1);
    UI_DrawProgressBar(10, 70, 220, 25, 25, GREEN, DARKGRAY, GRAY);
    UI_DrawProgressBar(10, 105, 220, 25, 50, YELLOW, DARKGRAY, GRAY);
    UI_DrawProgressBar(10, 140, 220, 25, 75, ORANGE, DARKGRAY, GRAY);
    UI_DrawProgressBar(10, 175, 220, 25, 100, RED, DARKGRAY, GRAY);

    // 滑动条
    UI_DrawString(10, 215, "Sliders:", MAGENTA, BLACK, 1);
    UI_DrawSlider(10, 235, 220, 20, 30, 100, BLUE, DARKGRAY, CYAN);
    UI_DrawSlider(10, 265, 220, 20, 60, 100, GREEN, DARKGRAY, YELLOW);
    UI_DrawSlider(10, 295, 220, 20, 90, 100, RED, DARKGRAY, ORANGE);
}

// =============================================
// 场景 5: 仪表盘界面演示
// =============================================
void demo_dashboard(void) {
    LCD_Clear(DARKGRAY);

    // 状态栏
    UI_FillRect(0, 0, 240, 30, BLUE);
    UI_DrawString(10, 8, "DASHBOARD", WHITE, BLUE, 2);
    UI_DrawString(190, 12, "12:34", YELLOW, BLUE, 1);

    // 左侧信息面板
    UI_DrawWindow(5, 35, 110, 140, "System", PURPLE, WHITE, BLACK);
    UI_Printf(10, 57, GREEN, BLACK, 1, "CPU: 45%%");
    UI_Printf(10, 72, YELLOW, BLACK, 1, "RAM: 2.1GB");
    UI_Printf(10, 87, CYAN, BLACK, 1, "Temp: 42C");
    UI_DrawProgressBar(10, 105, 100, 15, 45, GREEN, GRAY, LIGHTGRAY);
    UI_DrawButton(10, 130, 100, 30, "Refresh", BLUE, WHITE, GRAY, false);

    // 右侧状态指示
    UI_DrawWindow(125, 35, 110, 140, "Status", PURPLE, WHITE, BLACK);
    UI_DrawCheckbox(135, 60, 15, true, GREEN, BLACK);
    UI_DrawString(155, 60, "WiFi", WHITE, BLACK, 1);
    UI_DrawCheckbox(135, 85, 15, false, RED, BLACK);
    UI_DrawString(155, 85, "BT", WHITE, BLACK, 1);
    UI_DrawRadio(135, 110, 15, true, BLUE, BLACK);
    UI_DrawString(155, 110, "Mode A", WHITE, BLACK, 1);
    UI_DrawRadio(135, 135, 15, false, BLUE, BLACK);
    UI_DrawString(155, 135, "Mode B", WHITE, BLACK, 1);

    // 底部数据显示
    UI_FillRect(5, 185, 230, 130, BLACK);
    UI_DrawRect(5, 185, 230, 130, GRAY);
    UI_DrawString(10, 195, "Data Monitor:", CYAN, BLACK, 1);
    UI_Printf(10, 215, WHITE, BLACK, 1, "Packets: 1,245");
    UI_Printf(10, 230, WHITE, BLACK, 1, "Errors:  3");
    UI_Printf(10, 245, WHITE, BLACK, 1, "Speed:   54 Mbps");

    UI_DrawSlider(10, 270, 220, 18, 75, 100, GREEN, DARKGRAY, CYAN);
    UI_DrawString(10, 295, "Bandwidth", LIGHTGRAY, BLACK, 1);
}

// =============================================
// 运行所有演示（供 empty.c 调用）
// =============================================
void UI_RunAllDemos(void) {

    current_scene = DEMO_SHAPES;

    while (1) {
        // 根据当前场景渲染不同界面
        switch (current_scene) {
            case DEMO_SHAPES:
                demo_shapes();
                break;
            case DEMO_TEXT:
                demo_text();
                break;
            case DEMO_BUTTONS:
                demo_buttons();
                break;
            case DEMO_PROGRESS:
                demo_progress();
                break;
            case DEMO_DASHBOARD:
                demo_dashboard();
                break;
            default:
                current_scene = DEMO_SHAPES;
                continue;
        }

        // 延时 3 秒后切换到下一个场景
        delay_cycles(CPUCLK_FREQ * 3);

        current_scene = (current_scene + 1) % DEMO_COUNT;
    }
}
