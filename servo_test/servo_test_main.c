/**
 * @file servo_test_main.c
 * @brief 舵机功能测试主程序 - MSPM0G3507
 *
 * 测试内容:
 *   TEST 1: 三点定位测试  (0° / 90° / 180°)
 *   TEST 2: 全范围扫描测试 (0° -> 180° -> 0°)
 *   TEST 3: 指定角度序列   (30° / 60° / 90° / 120° / 150°)
 *   TEST 4: 快速抖动测试   (检测舵机响应速度)
 *
 * 接线:
 *   舵机信号线 -> PA26
 *   舵机红线   -> 5V
 *   舵机棕/黑线-> GND
 *
 * SysConfig 配置要点:
 *   Timer: PWM_SERVO (TIMG12)
 *   时钟分频: DIV_16
 *   周期(Load): 39999
 *   PWM_CH0: PA26
 */

#include "ti_msp_dl_config.h"
#include "servo.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 *  阻塞延时 (复用 xiaochexunji 项目的方式)
 * --------------------------------------------------------------------- */
static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000U; i++) {
        __asm("nop");
    }
}

/* -----------------------------------------------------------------------
 *  TEST 1: 三点定位测试
 *  验证舵机能准确到达 0° / 90° / 180°
 * --------------------------------------------------------------------- */
static void test1_three_point(void)
{
    /* 0° */
    Servo_SetAngle(0);
    delay_ms(1000);

    /* 90° */
    Servo_SetAngle(90);
    delay_ms(1000);

    /* 180° */
    Servo_SetAngle(180);
    delay_ms(1000);

    /* 回中 */
    Servo_SetAngle(90);
    delay_ms(500);
}

/* -----------------------------------------------------------------------
 *  TEST 2: 全范围扫描测试
 *  0° -> 180° -> 0°，每步约15ms
 * --------------------------------------------------------------------- */
static void test2_sweep(void)
{
    Servo_Sweep(15U);
    delay_ms(500);
}

/* -----------------------------------------------------------------------
 *  TEST 3: 指定角度序列
 *  依次定位到 30° / 60° / 90° / 120° / 150°
 * --------------------------------------------------------------------- */
static void test3_angle_sequence(void)
{
    const uint8_t angles[] = {30, 60, 90, 120, 150, 90};
    const uint8_t len = sizeof(angles) / sizeof(angles[0]);

    for (uint8_t i = 0; i < len; i++) {
        Servo_SetAngle(angles[i]);
        delay_ms(800);
    }
}

/* -----------------------------------------------------------------------
 *  TEST 4: 快速抖动测试
 *  在 80° ~ 100° 之间快速来回，检测响应速度
 * --------------------------------------------------------------------- */
static void test4_jitter(void)
{
    for (uint8_t i = 0; i < 10; i++) {
        Servo_SetAngle(80);
        delay_ms(100);
        Servo_SetAngle(100);
        delay_ms(100);
    }
    /* 回中稳定 */
    Servo_SetAngle(90);
    delay_ms(500);
}

/* -----------------------------------------------------------------------
 *  主函数
 * --------------------------------------------------------------------- */
int main(void)
{
    /* 1. 板级初始化（SysConfig 生成） */
    SYSCFG_DL_init();

    /* 2. 舵机初始化，置于90°中间位置 */
    Servo_Init();

    /* 上电等待稳定 */
    delay_ms(1000);

    /* 3. 循环执行所有测试 */
    while (1)
    {
        /* --- TEST 1 --- */
        test1_three_point();
        delay_ms(500);

        /* --- TEST 2 --- */
        test2_sweep();
        delay_ms(500);

        /* --- TEST 3 --- */
        test3_angle_sequence();
        delay_ms(500);

        /* --- TEST 4 --- */
        test4_jitter();

        /* 每轮测试结束后暂停2秒再重复 */
        delay_ms(2000);
    }
}
