/**
 * @file empty.c
 * @brief 八路灰度巡线 — 闭环比例差速 (慢版本内核 + 速度自适应)
 *
 * 设计思路 (结合两个版本优点):
 *   慢版本优点: 全程闭环, 一直朝线打方向直到线回中
 *               → 永远不会转过头, 永远不会冲出去, 不依赖计时
 *   快版本需求: 直道要快
 *
 *   做法: 保留慢版本的闭环比例差速内核, 叠加速度自适应
 *     直道 (err小) → 全速 SPEED_MAX 快跑
 *     缓弯 (err中) → 自动降速 + 比例差速
 *     急弯/直角 (err大) → 速度压到 SPEED_MIN, 差速大到内轮反转
 *                          → 天然原地掉头, 但仍是闭环, 线回中自动退出
 *
 *   ★ 没有状态机, 没有定时转弯, 没有开环 → 跑多少圈都不会累积漂移
 *
 * 传感器: X1~X8 (左→右), ACTIVE_LEVEL=1 表示检测到黑线
 * 位置加权: [-30,-20,-15,-5,5,15,20,30] → err 负=偏左, 正=偏右
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
 *  可调参数
 * ================================================================ */

/* ── 速度 ── */
#define SPEED_MAX         700    /* ★ 直道最高速 (要更快就加大)          */
#define SPEED_MIN         400    /* ★ 急弯速度 (=慢版本验证过的过弯速度) */
                                 /*   过弯不稳/冲出就减小这个值           */

/* ── 转向 ── */
#define STEER_K            40    /* ★ 转向增益 (慢版本验证值)            */
#define STEER_MIN         200    /* ★ 最小差速兜底, 防小弯打滑无力       */
#define STEER_MAX         400    /* ★ 最大差速上限 — 钳住转弯角速度      */
                                 /*   转弯太急就减小, 转不过弯就加大      */
#define TURN_SPEED        180    /* ★ 急弯时整体降速, 让掉头慢而稳       */
                                 /*   abs_err 越大越接近此值, 越慢越稳    */
#define MOTOR_MAX        3200    /* PWM计数器上限 (=timerCount)          */

/* ── 传感器 ── */
#define ACTIVE_LEVEL        1    /* 1=检测到黑线输出高电平               */

/* ── 圈数控制 ── */
#define TARGET_LAPS         1    /* ★ 目标圈数: 跑几圈后停车 (1/2/3/4…) */
#define CORNERS_PER_LAP     4    /* 正方形跑道每圈4个直角                */
#define CLEAR_CONFIRM      30    /* 回到直道确认×4ms=100ms, 才解锁       */
#define LOCKOUT_TICKS     125    /* ★ 数到弯后强制锁定×4ms=500ms         */
                                 /*   覆盖整个转弯过程, 期间绝不重复计数  */
                                 /*   数少了(漏弯)减小, 数多了(重复)加大  */

/* ================================================================
 *  全局变量
 * ================================================================ */
static uint8_t   sensor[8];        /* X1~X8 最新值               */
static int8_t    last_err = 0;     /* 上次误差 (丢线时保持方向)  */
static uint8_t   scan_ch  = 0;     /* 当前MUX扫描通道            */

/* ── 圈数计数 ── */
static uint16_t  corner_count = 0; /* 已数到的直角总数            */
static uint8_t   corner_armed = 0; /* 起跑=0锁定, 外侧先清空100ms才解锁 → 防起跑线误计 */
static uint8_t   clear_cnt    = 0; /* 回到直道连续确认计数        */
static uint16_t  lockout      = 0; /* 数弯后强制锁定倒计时(拍)    */
static uint8_t   finishing    = 0; /* 1=圈数已够, 正转最后一个弯, 回到直道即停 */
static uint8_t   car_stopped  = 0; /* 1=已完成圈数, 永久停车      */

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
 *  丢线 → 保持上次误差 (继续朝原方向打, 利于过急弯)
 * ================================================================ */
static const int8_t weight[8] = {-30, -20, -15, -5, 5, 15, 20, 30};

static int8_t calc_err(void)
{
    int16_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i] == ACTIVE_LEVEL) { sum += weight[i]; cnt++; }
    }
    if (cnt == 0) return last_err;      /* 丢线 → 保持方向 */
    last_err = (int8_t)(sum / cnt);
    return last_err;
}

/* ================================================================
 *  电机输出
 *    l>0 左轮前进  l<0 左轮后退
 *    r>0 右轮前进  r<0 右轮后退
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
 *  2kHz ISR — 非阻塞扫描 (每中断扫1路) + 250Hz 闭环控制
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
        if (scan_ch != 0) break;    /* 8路未扫完直接退出 */

        /* ======== 每 4ms 执行一次闭环控制 (250Hz) ======== */

        /* ── 圈数到达: 永久停车 (低侧制动) ── */
        if (car_stopped) {
            motor(0, 0);
            break;
        }

        int8_t  err     = calc_err();
        int32_t abs_err = (err < 0) ? -err : err;

        /* ── 数弯 + 圈数判断 (只观测, 不干预驾驶) ──
         *   正方形每圈 CORNERS_PER_LAP 个直角.
         *   最外侧传感器(X1/X8)亮 = 到达一个直角.
         *
         *   双重防重复计数:
         *     1) 强制锁定期 lockout: 数到一个弯后无条件锁死 LOCKOUT_TICKS
         *        (×4ms), 完整覆盖整个掉头过程. 期间既不计数也不解锁,
         *        彻底屏蔽转弯中拐角线在阵列下反复扫动造成的抖动.
         *     2) 回到直道确认: 锁定期结束后, 还需车真正回到新直道
         *        (中间在线 且 外侧全灭) 连续 CLEAR_CONFIRM 次才解锁. */
        {
            uint8_t outer = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

            if (lockout) {
                /* 强制锁定期: 倒计时, 期间完全不数弯不解锁 */
                lockout--;
            } else if (corner_armed) {
                if (outer) {
                    corner_count++;
                    corner_armed = 0;           /* 锁定, 防重复计数 */
                    clear_cnt    = 0;
                    lockout      = LOCKOUT_TICKS;/* 启动强制锁定期  */
                    if (corner_count >= (uint16_t)TARGET_LAPS * CORNERS_PER_LAP) {
                        finishing = 1;          /* 圈数已够: 不立即停,
                                                 * 正常转完这最后一个弯,
                                                 * 回到起点直道后才停车. */
                    }
                }
            } else {
                /* 解锁条件: 车必须真正回到"新直道"上, 而不只是外侧瞬间清空.
                 *   回到直道 = 中间(X4/X5)在线 且 两侧外侧(X1/X8)都灭.
                 *   需连续 CLEAR_CONFIRM 次确认. */
                uint8_t on_straight = ((sensor[3] == ACTIVE_LEVEL) ||
                                       (sensor[4] == ACTIVE_LEVEL)) && !outer;
                if (on_straight) {
                    if (++clear_cnt >= CLEAR_CONFIRM) {
                        if (finishing) {
                            car_stopped = 1;    /* 最后一个弯已转完并回正 → 停 */
                            motor(0, 0);
                            break;
                        }
                        corner_armed = 1;       /* 正常解锁, 继续数下一个弯 */
                    }
                } else {
                    clear_cnt = 0;
                }
            }
        }

        /* ── 速度自适应: 直道快, 弯道慢 ──
         * abs_err=0  → SPEED_MAX  (直道全速)
         * abs_err=30 → TURN_SPEED (急弯压到很低, 让掉头慢而稳)  */
        int32_t speed = SPEED_MAX - abs_err * (SPEED_MAX - TURN_SPEED) / 30;

        /* ── 比例转向 ── */
        int32_t steer = (int32_t)err * STEER_K;

        /* ── 差速兜底: 有偏差时保证足够转向力, 防打滑无力 ── */
        if (abs_err >= 4) {
            if (steer > 0 && steer <  STEER_MIN) steer =  STEER_MIN;
            if (steer < 0 && steer > -STEER_MIN) steer = -STEER_MIN;
        }

        /* ── 差速上限: 钳住转弯角速度, 防止掉头太急 ── */
        if (steer >  STEER_MAX) steer =  STEER_MAX;
        if (steer < -STEER_MAX) steer = -STEER_MAX;

        /* ── 差速输出 ──
         * 急弯时 speed 已降到 TURN_SPEED, steer 被钳到 STEER_MAX,
         *   内轮 = speed-steer 小幅反转 → 慢速掉头.
         *   仍是闭环: 线一回中 err 变小, 掉头自动停, 绝不转过头  */
        int32_t l = speed - steer;
        int32_t r = speed + steer;

        motor(l, r);
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

    mux_set(0);     /* 预选 ch0, 让第一个 ISR 读到有效值 */

    DL_TimerA_startCounter(PWM_DRIVE_INST);
    DL_GPIO_setPins(GRP_SLEEP_PORT, GRP_SLEEP_SLEEP_PIN);

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) { __WFI(); }
}
