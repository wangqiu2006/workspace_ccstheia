/**
 * @file empty.c
 * @brief 八路灰度巡线 — 正方形跑道 (A4950, SysTick计时)
 *
 * 计时方案: SysTick @ 32MHz → 1ms中断, g_millis 自由运行计数器
 *
 * 速度策略:
 *   直道 SPEED_MAX 设高 → 快
 *   转弯 CORNER_TURN_SPEED 设低 → 慢而稳
 *
 * 状态机 (3状态, 无ST_FIND):
 *   ST_FOLLOW → 比例差速巡线
 *   ST_BRAKE  → 检测到直角立即制动
 *   ST_TURN   → 原地旋转固定时间, 转完直接回ST_FOLLOW
 *
 *   转完后设 COOLDOWN_MS 冷却窗口, 窗口内屏蔽直角检测.
 *   比例差速会自动修正转弯后的小偏差, 不需要额外找线状态.
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

/* ── 直道 (要快) ── */
#define SPEED_MAX        1000    /* 直道最高速 PWM (MOTOR_MAX=3200=100%) */
#define SPEED_MIN         400    /* 急弯内轮最低速                        */
#define TURN_MAX          800    /* 差速上限                              */
#define STEER_K            25    /* 转向增益                              */
#define DEAD_ZONE           5    /* 误差死区 ±5                           */
#define MOTOR_MAX        3200    /* PWM计数器上限 (=timerCount)            */

/* ── 传感器 ── */
#define ACTIVE_LEVEL        1    /* 1=检测到黑线输出高电平                */

/* ── 直角检测 ── */
#define CORNER_DEBOUNCE     2    /* 连续N次(×4ms)确认才进弯               */

/* ── 直角过弯时间 (ms) ── */
#define BRAKE_MS           80    /* 制动时间                              */
#define TURN_MS           320    /* ★ 旋转时间: 转不到位加大, 转过头减小  */
#define COOLDOWN_MS      800    /* 过弯后冷却窗口 — 此期间不检测直角     */
                                 /*   改小可以更快响应下一个弯             */

/* ── 转弯速度 (要慢) ── */
#define CORNER_TURN_SPEED 500    /* ★ 原地转速 PWM                        */

/* ================================================================
 *  状态机
 * ================================================================ */
typedef enum {
    ST_FOLLOW = 0,
    ST_BRAKE,
    ST_TURN
} CarState;

/* ================================================================
 *  全局变量
 * ================================================================ */
static uint8_t   sensor[8];
static int8_t    last_err        = 0;
static uint8_t   scan_ch         = 0;

static CarState  state           = ST_FOLLOW;
static uint32_t  state_entry_ms  = 0;
static uint32_t  cooldown_until  = 0;   /* 冷却到期时刻 (绝对ms值)       */
static int8_t    turn_dir        = 1;   /* +1=右转  -1=左转              */
static uint8_t   corner_cnt      = 0;   /* 直角去抖计数                  */

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
 *  直角检测: 只看最外侧传感器
 *    X8=sensor[7] 亮 → 右侧直角
 *    X1=sensor[0] 亮 → 左侧直角
 * ================================================================ */
static uint8_t is_right_corner(void) { return sensor[7] == ACTIVE_LEVEL; }
static uint8_t is_left_corner(void)  { return sensor[0] == ACTIVE_LEVEL; }

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
 *  电机输出  (A4950: motor(0,0) = IN1=IN2=0 = 低侧制动)
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
 *  2kHz ISR — 非阻塞扫描 + 250Hz 控制
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
        int8_t   err = calc_err();
        int32_t  ab  = (err < 0) ? -err : err;
        int32_t  l, r;

        switch (state) {

        /* ============================================================
         *  正常巡线
         * ============================================================ */
        case ST_FOLLOW:

            /* 冷却窗口内: 直角检测全程屏蔽 */
            if (now >= cooldown_until) {
                uint8_t rc = is_right_corner();
                uint8_t lc = is_left_corner();

                if (rc || lc) {
                    if (++corner_cnt >= CORNER_DEBOUNCE) {
                        turn_dir       = rc ? -1 : 1;
                        state          = ST_BRAKE;
                        state_entry_ms = now;
                        corner_cnt     = 0;
                        motor(0, 0);
                        break;
                    }
                } else {
                    corner_cnt = 0;
                }
            }

            /* 比例差速巡线 */
            if (ab <= DEAD_ZONE) {
                l = SPEED_MAX;
                r = SPEED_MAX;
            } else {
                int32_t spd  = SPEED_MAX - ab * (SPEED_MAX - SPEED_MIN) / 30;
                int32_t turn = (int32_t)err * STEER_K;
                if (turn >  TURN_MAX) turn =  TURN_MAX;
                if (turn < -TURN_MAX) turn = -TURN_MAX;
                if (turn > 0) { l = spd;        r = spd + turn; }
                else          { l = spd - turn;  r = spd;        }
            }
            motor(l, r);
            break;

        /* ============================================================
         *  制动
         * ============================================================ */
        case ST_BRAKE:
            motor(0, 0);
            if ((now - state_entry_ms) >= BRAKE_MS) {
                state          = ST_TURN;
                state_entry_ms = now;
            }
            break;

        /* ============================================================
         *  原地旋转, 转完直接回ST_FOLLOW + 冷却
         *  不需要"找线"状态 — 比例差速会自动修正小偏差
         * ============================================================ */
        case ST_TURN:
            if (turn_dir > 0) {
                motor( CORNER_TURN_SPEED, -CORNER_TURN_SPEED);
            } else {
                motor(-CORNER_TURN_SPEED,  CORNER_TURN_SPEED);
            }
            if ((now - state_entry_ms) >= TURN_MS) {
                motor(0, 0);
                last_err       = 0;
                state          = ST_FOLLOW;
                state_entry_ms = now;
                /* 冷却窗口: 此后 COOLDOWN_MS 内不检测直角 */
                cooldown_until = now + COOLDOWN_MS;
            }
            break;

        default:
            state = ST_FOLLOW;
            motor(0, 0);
            break;
        }
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

    /* SysTick: 32MHz / 32000 = 1kHz → 1ms 中断 */
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
