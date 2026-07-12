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
#define SPEED_BASE        400     /* 直线基础速度 PWM */
#define STEER_K            40     /* 转向比例系数 (PWM / err) */
#define STEER_MIN          200    /* 最小差速量，防止小弯无力 */
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

    if (cnt == 0) return 0;   /* 丢线: 保持上次方向 */
    return (int8_t)(sum / cnt);
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

    while (1) {
        read_all_sensors();
        int8_t err = calc_error();

        /* err<0 → 偏左 → 右转 (右轮加速) */
        /* err>0 → 偏右 → 左转 (左轮加速) */
        int32_t abs_err = err < 0 ? -err : err;
        int32_t steer = (int32_t)err * STEER_K;

        /* 差速兜底：确保打滑时有足够的转向力 */
        if (abs_err >= 4 && steer > 0 && steer < STEER_MIN) steer =  STEER_MIN;
        if (abs_err >= 4 && steer < 0 && steer > -STEER_MIN) steer = -STEER_MIN;

        int32_t left_spd  = SPEED_BASE - steer;
        int32_t right_spd = SPEED_BASE + steer;

        motor_set(left_spd, right_spd);
        delay_ms(10);
    }
}
