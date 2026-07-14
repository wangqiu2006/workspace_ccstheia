/**
 * @file empty.c
 * @brief 八路灰度巡线 — 闭环比例差速 (慢版本内核 + 速度自适应)
 *
 * 速度策略:
 *   直道 (err小) → 全速 SPEED_MAX 快跑
 *   急弯 (err大) → 速度压到 TURN_SPEED, 差速钳到 STEER_MAX → 慢速掉头
 *   仍是闭环: 线回中自动停, 不依赖计时
 *
 * 圈数计数:
 *   X1(sensor[0]) 或 X8(sensor[7]) 触发 → 数一个弯
 *   触发后 CORNER_LOCK_MS(1秒) 内不再检测 → 防重复
 *   正方形一圈 4 个弯, TARGET_LAPS 圈 = TARGET_LAPS×4 个弯
 *   最后一个弯数到后继续跑完转弯, 回到直道停车
 *
 * 引脚:
 *   左电机: CC2=PA15(正) CC0=PB14(反)  [TIMA0]
 *   右电机: CC3=PA12(正) CC1=PA7(反)   [TIMA0]
 *   SLEEP:  PB16
 *   灰度:   AD0=PA24 AD1=PA25 AD2=PA26 OUT=PA27
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ================================================================
 *  SysTick — 1ms 自由运行计数器
 * ================================================================ */
volatile uint32_t g_millis = 0;

void SysTick_Handler(void)
{
    g_millis++;
}

/* ================================================================
 *  可调参数
 * ================================================================ */

/* ── 速度 ── */
#define SPEED_MAX         700    /* ★ 直道最高速                          */
#define TURN_SPEED        180    /* ★ 急弯时整体速度                      */
#define MOTOR_MAX        3200    /* PWM计数器上限 (=timerCount)            */

/* ── 转向 ── */
#define STEER_K            40    /* ★ 转向增益                            */
#define STEER_MIN         200    /* ★ 小弯差速兜底                        */
#define STEER_MAX         400    /* ★ 差速上限, 钳住转弯角速度             */

/* ── 传感器 ── */
#define ACTIVE_LEVEL        1    /* 1=检测到黑线输出高电平                */

/* ── 圈数控制 ── */
#define TARGET_LAPS         5    /* ★ 目标圈数                            */
#define CORNERS_PER_LAP     4    /* 正方形每圈4个弯                       */
#define CORNER_LOCK_MS   1000    /* ★ 数到弯后锁定时间(ms), 防重复触发    */
                                 /*   必须 > 整个转弯过程耗时              */
                                 /*   必须 < 转弯+一段直道总耗时           */

/* ================================================================
 *  全局变量
 * ================================================================ */
static uint8_t   sensor[8];
static int8_t    last_err = 0;
static uint8_t   scan_ch  = 0;

/* ── 圈数计数 ── */
static uint16_t  corner_count    = 0;  /* 已数到的弯数                   */
static uint32_t  corner_unlock   = 0;  /* 解锁时刻(ms绝对值), 初值0=立即可用 */
static uint8_t   finishing       = 0;  /* 1=最后一个弯已数到, 转完后停车 */
static uint8_t   car_stopped     = 0;  /* 1=已停车                       */

/* ================================================================
 *  MUX 选通 (74HC4051)
 * ================================================================ */
static void mux_set(uint8_t ch)
{
    uint8_t c = (ch >> 2) & 1;
    uint8_t b = (ch >> 1) & 1;
    uint8_t a =  ch        & 1;
    if (a) DL_GPIO_setPins(  GRP_GRAY_PORT, GRP_GRAY_AD0_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD0_PIN);
    if (b) DL_GPIO_setPins(  GRP_GRAY_PORT, GRP_GRAY_AD1_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD1_PIN);
    if (c) DL_GPIO_setPins(  GRP_GRAY_PORT, GRP_GRAY_AD2_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_PORT, GRP_GRAY_AD2_PIN);
}

/* ================================================================
 *  加权位置误差 [-30, +30]  负=偏左  正=偏右
 * ================================================================ */
static const int8_t weight[8] = {-30, -20, -15, -5, 5, 15, 20, 30};

static int8_t calc_err(void)
{
    int16_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i] == ACTIVE_LEVEL) { sum += weight[i]; cnt++; }
    }
    if (cnt == 0) return last_err;
    last_err = (int8_t)(sum / cnt);
    return last_err;
}

/* ================================================================
 *  电机输出  (A4950: motor(0,0) = 低侧制动)
 * ================================================================ */
static void motor(int32_t l, int32_t r)
{
    if (l >  MOTOR_MAX) l =  MOTOR_MAX;
    if (l < -MOTOR_MAX) l = -MOTOR_MAX;
    if (r >  MOTOR_MAX) r =  MOTOR_MAX;
    if (r < -MOTOR_MAX) r = -MOTOR_MAX;

    if (l >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  l, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, -l, DL_TIMER_CC_0_INDEX);
    }
    if (r >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  r, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_1_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, -r, DL_TIMER_CC_1_INDEX);
    }
}

/* ================================================================
 *  2kHz ISR — 非阻塞扫描 + 250Hz 闭环控制
 * ================================================================ */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:

        /* ── 读当前通道 → 预选下一通道 ── */
        sensor[scan_ch] = DL_GPIO_readPins(GRP_GRAY_OUT_PORT, GRP_GRAY_OUT_OUT_PIN)
                          ? ACTIVE_LEVEL : !ACTIVE_LEVEL;
        scan_ch = (scan_ch + 1) & 7;
        mux_set(scan_ch);
        if (scan_ch != 0) break;

        /* ── 每 4ms 执行一次控制 ── */

        uint32_t now = g_millis;

        /* ── 永久停车 ── */
        if (car_stopped) { motor(0, 0); break; }

        int8_t  err     = calc_err();
        int32_t abs_err = (err < 0) ? -err : err;

        /* ── 圈数计数 ──────────────────────────────────────────────
         *   X1(sensor[0]) 或 X8(sensor[7]) 触发 → 数一个弯
         *   触发后锁定 CORNER_LOCK_MS(1s), 期间不再检测
         *   最后一个弯: 不立即停, 继续跑, 1s 锁定到期后在直道上停
         * ─────────────────────────────────────────────────────── */
        {
            uint8_t outer = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

            if (finishing) {
                /* 最后一个弯已数到: 锁定期结束 = 转弯已完成, 直接停车 */
                if (now >= corner_unlock) {
                    car_stopped = 1;
                    motor(0, 0);
                    break;
                }
            } else if (now >= corner_unlock && outer) {
                /* 锁定期外 + 外侧触发 → 数一个弯 */
                corner_count++;
                corner_unlock = now + CORNER_LOCK_MS;   /* 锁定1秒 */

                if (corner_count >= (uint16_t)TARGET_LAPS * CORNERS_PER_LAP) {
                    finishing = 1;  /* 最后一个弯: 等1s锁定到期后停车 */
                }
            }
        }

        /* ── 闭环驾驶 (不动) ───────────────────────────────────── */
        int32_t speed = SPEED_MAX - abs_err * (SPEED_MAX - TURN_SPEED) / 30;
        int32_t steer = (int32_t)err * STEER_K;

        if (abs_err >= 4) {
            if (steer > 0 && steer <  STEER_MIN) steer =  STEER_MIN;
            if (steer < 0 && steer > -STEER_MIN) steer = -STEER_MIN;
        }
        if (steer >  STEER_MAX) steer =  STEER_MAX;
        if (steer < -STEER_MAX) steer = -STEER_MAX;

        motor(speed - steer, speed + steer);
        break;

    default:
        break;
    }
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();

    /* SysTick: 32MHz / 32000 = 1kHz → 1ms */
    SysTick->LOAD = 32000UL - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;

    mux_set(0);

    DL_TimerA_startCounter(PWM_DRIVE_INST);
    DL_GPIO_setPins(GRP_SLEEP_PORT, GRP_SLEEP_SLEEP_PIN);

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) { __WFI(); }
}
