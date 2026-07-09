/**
 * @file empty.c
 * @brief 八路灰度巡线 — MSPM0G3507 + A4950 + MG513XP28_12V
 *
 * 引脚映射:
 *   AIN1 = PB14 (TIMA0 CC0)  左轮PWM
 *   AIN2 = PA15 (GPIO)       左轮方向
 *   BIN1 = PA7  (TIMA0 CC1)  右轮PWM
 *   BIN2 = PA12 (GPIO)       右轮方向
 *   继电器 PB16 (GRP_RELAY)  拉高接通电机12V
 *
 *   灰度AD0=PA24 AD1=PA25 AD2=PA26 OUT=PA27
 *   左编码器: PA23(A) PA0(B)
 *   右编码器: PB24(A) PB25(B)
 *   串口: TX=PA10 RX=PA11 115200
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ================================================================
 *  参数配置
 * ================================================================ */
#define BASE_SPEED     1200U    /* 基础速度 [0,3200] */
#define KP             120      /* P系数 */
#define MOTOR_MAX      3200U
#define MOTOR_STOP     0U

/* ================================================================
 *  延时
 * ================================================================ */
static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000U; i++) __asm("nop");
}
static void delay_us(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 4U; i++) __asm("nop");
}

/* ================================================================
 *  UART
 * ================================================================ */
static void uart_puts(const char *s)
{
    while (*s) DL_UART_transmitDataBlocking(UART_JDY31_INST, (uint8_t)*s++);
}
static void uart_printf(const char *fmt, ...)
{
    char buf[128]; va_list a;
    va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    uart_puts(buf);
}

/* ================================================================
 *  电机控制（TIMA0 4通道）
 *  左电机: AIN1=CC0(PB14) AIN2=CC1(PA7)
 *  右电机: BIN1=CC2(PA15) BIN2=CC3(PA12)
 *  正转: IN1=duty IN2=0  反转: IN1=0 IN2=duty
 * ================================================================ */
static void motor_set(int32_t left, int32_t right)
{
    if (left  >  (int32_t)MOTOR_MAX) left  =  (int32_t)MOTOR_MAX;
    if (left  < -(int32_t)MOTOR_MAX) left  = -(int32_t)MOTOR_MAX;
    if (right >  (int32_t)MOTOR_MAX) right =  (int32_t)MOTOR_MAX;
    if (right < -(int32_t)MOTOR_MAX) right = -(int32_t)MOTOR_MAX;

    /* 左轮: AIN1(CC0)正转 AIN2(CC1)反转 */
    if (left >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)left, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,     DL_TIMER_CC_1_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,      DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-left), DL_TIMER_CC_1_INDEX);
    }

    /* 右轮: BIN1(CC2)正转 BIN2(CC3)反转 */
    if (right >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)right, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,      DL_TIMER_CC_3_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,       DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-right), DL_TIMER_CC_3_INDEX);
    }
}

static void motor_stop(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP, DL_TIMER_CC_3_INDEX);
}

/* ================================================================
 *  灰度传感器
 * ================================================================ */
static void mux_select(uint8_t ch)
{
    if (ch & 0x01) DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD0_PIN);
    else           DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD0_PIN);
    if (ch & 0x02) DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD1_PIN);
    else           DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD1_PIN);
    if (ch & 0x04) DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD2_PIN);
    else           DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD2_PIN);
}

static void gray_scan_all(uint8_t data[8])
{
    for (uint8_t ch = 0; ch < 8; ch++) {
        mux_select(ch);
        delay_us(50);
        data[ch] = DL_GPIO_readPins(GRP_GRAY_OUT_PORT, GRP_GRAY_OUT_OUT_PIN) ? 1U : 0U;
    }
}

/* ================================================================
 *  巡线算法（P控制）
 * ================================================================ */
static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static int32_t calc_error(const uint8_t data[8])
{
    int32_t sum = 0, count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (data[i]) { sum += weight[i]; count++; }
    }
    if (count == 0) return 127;
    return sum / count;
}

static void line_follow(const uint8_t data[8])
{
    int32_t err = calc_error(data);
    if (err == 127) { motor_stop(); return; }
    int32_t cor   = (err * KP) / 10;
    motor_set((int32_t)BASE_SPEED + cor, (int32_t)BASE_SPEED - cor);
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    delay_ms(200);

    /* 先输出欢迎信息，确认UART正常 */
    uart_puts("\r\n=== 八路灰度巡线 MSPM0G3507 ===\r\n");
    uart_printf("BASE=%u KP=%d\r\n", BASE_SPEED, KP);

    /* 启动 TIMA0 PWM */
    DL_TimerA_startCounter(PWM_DRIVE_INST);

    /* 闭合继电器，接通电机12V PB16 */
    DL_GPIO_setPins(GRP_RELAY_PORT, GRP_RELAY_MOTOR_PIN);

    delay_ms(500);

    uint8_t sensor[8];

    while (1)
    {
        /* 逐通道隔离测试：每次只开一路 PWM，其余全部为0 */

        /* CC0 = PB14 (AIN1) */
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 1200, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_3_INDEX);
        uart_puts("CC0 (PB14/AIN1) ON\r\n");
        delay_ms(2000);

        /* CC1 = PA7 (AIN2) */
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 1200, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_3_INDEX);
        uart_puts("CC1 (PA7/AIN2) ON\r\n");
        delay_ms(2000);

        /* CC2 = PA15 (BIN1) */
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 1200, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_3_INDEX);
        uart_puts("CC2 (PA15/BIN1) ON\r\n");
        delay_ms(2000);

        /* CC3 = PA12 (BIN2) */
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,    0, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 1200, DL_TIMER_CC_3_INDEX);
        uart_puts("CC3 (PA12/BIN2) ON\r\n");
        delay_ms(2000);

        /* 全停 */
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0, DL_TIMER_CC_3_INDEX);
        uart_puts("STOP\r\n");
        delay_ms(1000);
    }
}
