/**
 * @file empty.c
 * @brief 八路灰度巡线 — 加权位置 + 比例差速（参考官方 app_irtracking_eight）
 *
 * 传感器排列: X1-X8 (左→右), ACTIVE_LEVEL=1 表示检测到黑线
 * 位置加权: [-30,-20,-15,-5, 5,15,20,30] → err 负=偏左, 正=偏右
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ================================================================
 *  可调参数
 * ================================================================ */
#define SPEED_MAX          800    /* 直道最高速度 */
#define SPEED_MIN          350    /* 弯道最低速度 */
#define TURN_MAX            500    /* 最大差速量 */
#define STEER_K             30    /* 转向比例系数 */
#define DEAD_ZONE             7    /* 死区 ±7 */
#define MOTOR_MAX         3200
#define ACTIVE_LEVEL        1     /* 1=黑线检测到, 0=白底 */

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
 *  传感器读取 — 8路MUX扫描
 * ================================================================ */
static uint8_t sensor[8];  /* X1..X8, 1=在线 */

static void mux_select(uint8_t ch)
{
    uint8_t c = (ch >> 2) & 1;
    uint8_t b = (ch >> 1) & 1;
    uint8_t a =  ch       & 1;

    (a) ? DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD0_PIN)
        : DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD0_PIN);
    (b) ? DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD1_PIN)
        : DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD1_PIN);
    (c) ? DL_GPIO_setPins(GRP_GRAY_PORT, GRP_GRAY_AD2_PIN)
        : DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD2_PIN);
}

static void read_all_sensors(void)
{
    for (uint8_t ch = 0; ch < 8; ch++) {
        mux_select(ch);
        delay_us(100);
        sensor[ch] = (DL_GPIO_readPins(GRP_GRAY_OUT_PORT, GRP_GRAY_OUT_OUT_PIN))
                     ? ACTIVE_LEVEL : !ACTIVE_LEVEL;
    }
}

/* ================================================================
 *  位置误差计算 — 加权平均 (参考官方 Line_Tracke)
 *  返回: err ∈ [-30, +30], 负=偏左需右转, 正=偏右需左转
 * ================================================================ */
static const int8_t weight[8] = {-30, -20, -15, -5, 5, 15, 20, 30};

static int8_t last_err = 0;

static int8_t calc_error(void)
{
    int16_t sum = 0;
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i] == ACTIVE_LEVEL) {
            sum += weight[i];
            cnt++;
        }
    }

    if (cnt == 0) {
        /* 丢线: 保持上次方向继续找, 不返回0 */
        return last_err;
    }
    last_err = (int8_t)(sum / cnt);
    return last_err;
}

/* ================================================================
 *  电机驱动 (同前)
 * ================================================================ */
static void motor_set(int32_t left, int32_t right)
{
    if (left  >  (int32_t)MOTOR_MAX) left  =  (int32_t)MOTOR_MAX;
    if (left  < -(int32_t)MOTOR_MAX) left  = -(int32_t)MOTOR_MAX;
    if (right >  (int32_t)MOTOR_MAX) right =  (int32_t)MOTOR_MAX;
    if (right < -(int32_t)MOTOR_MAX) right = -(int32_t)MOTOR_MAX;

    if (left >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)left,  DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0,               DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0,               DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-left), DL_TIMER_CC_0_INDEX);
    }
    if (right >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)right, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0,               DL_TIMER_CC_1_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0,               DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-right), DL_TIMER_CC_1_INDEX);
    }
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    delay_ms(200);

    DL_TimerA_startCounter(PWM_DRIVE_INST);
    DL_GPIO_setPins(GRP_SLEEP_PORT, GRP_SLEEP_SLEEP_PIN);

    static uint8_t edge = 0;  /* 0=正常, 1=左边缘转, 2=右边缘转 */

    while (1) {
        read_all_sensors();
        int8_t err = calc_error();

        int32_t l, r;

        /* ── 触发边缘标志 ── */
        if (!edge && sensor[0] == ACTIVE_LEVEL) edge = 1;
        if (!edge && sensor[7] == ACTIVE_LEVEL) edge = 2;

        /* ── 自适应速度: err越大越慢 ── */
        int32_t ab = err < 0 ? -err : err;
        int32_t spd = SPEED_MAX - (int32_t)ab * (SPEED_MAX - SPEED_MIN) / 30;

        /* ── 边缘转弯中: 原地甩 ── */
        if (edge == 1) {
            l =  1200; r = -1200;
            if (sensor[3] == ACTIVE_LEVEL || sensor[4] == ACTIVE_LEVEL) edge = 0;
        } else if (edge == 2) {
            l = -1200; r =  1200;
            if (sensor[3] == ACTIVE_LEVEL || sensor[4] == ACTIVE_LEVEL) edge = 0;
        }
        /* ── X2/X7 提前强转 ── */
        else if (sensor[1] == ACTIVE_LEVEL) {
            l = spd + 400; r = spd - 200;
        } else if (sensor[6] == ACTIVE_LEVEL) {
            l = spd - 200; r = spd + 400;
        }
        /* ── 死区直行 ── */
        else if (err >= -DEAD_ZONE && err <= DEAD_ZONE) {
            l = spd; r = spd;
        }
        /* ── 比例差速: 内轮保持前进, 靠快轮提速转向 ── */
        else {
            int32_t turn = (int32_t)err * STEER_K;
            if (turn >  TURN_MAX) turn =  TURN_MAX;
            if (turn < -TURN_MAX) turn = -TURN_MAX;
            if (turn > 0) {
                l = spd;           /* 内轮保持基准速 */
                r = spd + turn;    /* 外轮提速 */
            } else {
                l = spd - turn;    /* 外轮提速 */
                r = spd;           /* 内轮保持基准速 */
            }
        }

        motor_set(l, r);
        delay_ms(1);
    }
}
