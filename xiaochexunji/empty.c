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
 *  编码器计数（双沿中断，volatile 保证中断和主循环间可见性）
 * ================================================================ */
static volatile int32_t enc_left  = 0;
static volatile int32_t enc_right = 0;

/* GROUP1 IRQ: 左编码器(GPIOA PA23/PA0) + 右编码器(GPIOB PB24/PB25) */
void GROUP1_IRQHandler(void)
{
    /* 先判断是 GPIOA 还是 GPIOB 触发 */
    if (DL_GPIO_getPendingInterrupt(GRP_ENC_LEFT_PORT)) {
        DL_GPIO_clearInterruptStatus(GRP_ENC_LEFT_PORT,
            GRP_ENC_LEFT_ENC_L_A_PIN | GRP_ENC_LEFT_ENC_L_B_PIN);
        enc_left++;
    }
    if (DL_GPIO_getPendingInterrupt(GRP_ENC_RIGHT_PORT)) {
        DL_GPIO_clearInterruptStatus(GRP_ENC_RIGHT_PORT,
            GRP_ENC_RIGHT_ENC_R_A_PIN | GRP_ENC_RIGHT_ENC_R_B_PIN);
        enc_right++;
    }
}

/* ================================================================
 *  参数配置
 * ================================================================ */
#define BASE_SPEED     1200U    /* 基础速度 [0,3200] */
#define KP             120     /* 位置P系数 */
#define KS             6       /* 速度平衡系数: 太大振荡 太小收敛慢 */
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
 *  电机控制（TIMA0 4通道 — 实测接线）
 *  左电机: 正转=CC0(PB14) 反转=CC2(PA15)
 *  右电机: 正转=CC1(PA7)  反转=CC3(PA12)
 *  正转: 正PWM=duty 反PWM=0   反转: 正PWM=0 反PWM=duty
 * ================================================================ */
static void motor_set(int32_t left, int32_t right)
{
    if (left  >  (int32_t)MOTOR_MAX) left  =  (int32_t)MOTOR_MAX;
    if (left  < -(int32_t)MOTOR_MAX) left  = -(int32_t)MOTOR_MAX;
    if (right >  (int32_t)MOTOR_MAX) right =  (int32_t)MOTOR_MAX;
    if (right < -(int32_t)MOTOR_MAX) right = -(int32_t)MOTOR_MAX;

    /* 左轮: 正转=CC0(PB14) 反转=CC2(PA15) */
    if (left >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)left, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,     DL_TIMER_CC_2_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,      DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-left), DL_TIMER_CC_2_INDEX);
    }

    /* 右轮: 正转=CC1(PA7) 反转=CC3(PA12) */
    if (right >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)right, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,      DL_TIMER_CC_3_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, MOTOR_STOP,       DL_TIMER_CC_1_INDEX);
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
    static int32_t prev_l = 0, prev_r = 0, bias = 0;

    int32_t err = calc_error(data);
    if (err == 127) { motor_stop(); bias = 0; return; }

    int32_t steer = (err * KP) / 10;

    /* 读编码器增量，关中断防撕裂 */
    int32_t cur_l, cur_r;
    __disable_irq();
    cur_l = enc_left;  cur_r = enc_right;
    __enable_irq();

    int32_t dl = cur_l - prev_l;
    int32_t dr = cur_r - prev_r;
    prev_l = cur_l;  prev_r = cur_r;

    /* 积分速度平衡: 左快 → bias 正增 → 右轮多补 */
    bias += (dl - dr) * KS / 10;
    if (bias >  500) bias =  500;
    if (bias < -500) bias = -500;

    motor_set((int32_t)BASE_SPEED + steer,
              (int32_t)BASE_SPEED - steer + bias);
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    delay_ms(200);

    /* 使能编码器中断 */
    NVIC_EnableIRQ(GRP_ENC_LEFT_INT_IRQN);
    NVIC_EnableIRQ(GRP_ENC_RIGHT_INT_IRQN);

    /* 欢迎信息 */
    uart_puts("\r\n=== 八路灰度巡线 MSPM0G3507 ===\r\n");
    uart_printf("BASE=%u KP=%d\r\n", BASE_SPEED, KP);

    /* 启动 TIMA0 PWM */
    DL_TimerA_startCounter(PWM_DRIVE_INST);

    /* 闭合继电器 PB16 */
    DL_GPIO_setPins(GRP_RELAY_PORT, GRP_RELAY_MOTOR_PIN);

    delay_ms(500);

    uint8_t sensor[8];
    int32_t snap_l, snap_r;

    /* ── 巡线主循环 ── */
    while (1)
    {
        gray_scan_all(sensor);
        line_follow(sensor);   /* P控制差速转向 */

        /* 调试输出（稳定后可注释掉提高控制频率）*/
        __disable_irq();
        snap_l = enc_left;
        snap_r = enc_right;
        __enable_irq();

        uart_printf("S:%d%d%d%d%d%d%d%d err=%d L=%d R=%d\r\n",
                    sensor[0], sensor[1], sensor[2], sensor[3],
                    sensor[4], sensor[5], sensor[6], sensor[7],
                    (int)calc_error(sensor),
                    (int)snap_l, (int)snap_r);

        delay_ms(10);   /* 100Hz 控制频率 */
    }
}
