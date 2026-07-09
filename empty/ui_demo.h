/*
 * UI 库演示函数声明
 */

#ifndef UI_DEMO_H_
#define UI_DEMO_H_

#include <stdint.h>

// 单个演示场景
void demo_shapes(void);      // 基础图形演示
void demo_text(void);        // 文字渲染演示
void demo_buttons(void);     // 按钮与控件演示
void demo_progress(void);    // 进度条与滑动条演示
void demo_dashboard(void);   // 仪表盘界面演示

// 运行所有演示（自动循环切换）
void UI_RunAllDemos(void);

#endif /* UI_DEMO_H_ */
