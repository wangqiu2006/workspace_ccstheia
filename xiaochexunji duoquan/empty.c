/**
 * @file empty.c
 * @brief 八路灰度巡线 — 闭环比例差速 (慢版本内核 + 速度自适应)
 *
 * 速度策略:
 *   直道 (err小) → 全速 SPEED_MAX 快跑, 二次曲线衰减
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
#define SPEED_MAX         600    /* ★ 直道最高速                          */
#define TURN_SPEED        200   /* ★ 急弯时整体速度 (+20%)               */
#define MOTOR_MAX        3200    /* PWM计数器上限 (=timerCount)            */

/* ── 转向 ── */
#define STEER_K            25    /* ★ 转向增益 (降低→直道不抖)            */
#define STEER_MIN         120    /* ★ 小弯差速兜底 (降低→直道不过修)      */
#define STEER_MAX         400    /* ★ 差速上限, 钳住转弯角速度             */

/* ── 直角弯 pivot (防切内道) ── */
#define PIVOT_ADVANCE_MS  800    /* ★ 检测到直角后先直行的时间(ms):        */
                                 /*   把后轮轴推到拐角顶点再转, 防切内道   */
                                 /*   传感器越靠前/车越长 → 调大           */
#define PIVOT_ADV_SPEED   420    /* ★ 直行推进阶段的速度 (要够大才能克服静摩擦走起来) */
#define PIVOT_CREEP        45    /* ★ 直角弯时前进分量 (越小越接近原地转) */
#define PIVOT_STEER       400    /* ★ 直角弯时差速 (原地重转力度)         */
#define PIVOT_TURN_MS     450    /* ★ 原地转持续时间(ms), 确保转过90°      */

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
static uint32_t  corner_unlock   = 1000; /* 初值1000:上电1秒内屏蔽计数,防起跑误触发 */
static uint8_t   finishing       = 0;  /* 1=最后一个弯已数到, 转完后停车 */
static uint8_t   car_stopped     = 0;  /* 1=已停车                       */

/* ── 直角弯状态机 ── */
/*   0=正常循迹  1=检测到直角,直行推进中  2=原地转中 */
static uint8_t   pivot_state     = 0;
static uint32_t  pivot_t_end     = 0;  /* 当前阶段结束时刻(ms)           */
static int32_t   pivot_dir       = 1;  /* 转向方向: +1右 / -1左          */
static uint32_t  pivot_unlock    = 0;  /* pivot冷却: 转完后锁定,防重复触发 */

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
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  l, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_2_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, -l, DL_TIMER_CC_2_INDEX);
    }
    if (r >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  r, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_3_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST,  0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, -r, DL_TIMER_CC_3_INDEX);
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
         *   pivot执行期间(state1/2)暂停计数, 防直行推进时重复触发
         * ─────────────────────────────────────────────────────── */
        {
            uint8_t outer = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

            if (finishing) {
                uint8_t on_straight = !outer &&
                                      ((sensor[3] == ACTIVE_LEVEL) ||
                                       (sensor[4] == ACTIVE_LEVEL));
                if (now >= corner_unlock && on_straight) {
                    car_stopped = 1;
                    motor(0, 0);
                    break;
                }
            } else if (pivot_state == 0 && now >= corner_unlock && now >= pivot_unlock && outer) {
                /* pivot未激活 + 锁定期外 + 外侧触发 → 数一个弯 */
                corner_count++;
                corner_unlock = now + CORNER_LOCK_MS;

                if (corner_count >= (uint16_t)TARGET_LAPS * CORNERS_PER_LAP) {
                    finishing = 1;
                }
            }
        }

        /* ── 直角弯状态机: 先直行推进, 再原地转 (防切内道) ─────────
         *   传感器装车头, 领先后轮轴. 一看到直角就转 → 后轮轴还没到
         *   拐角顶点 → 抄内道. 所以: 先直行 ADVANCE_MS 把后轮轴推到
         *   顶点, 再原地转 90°.
         *   state 0: 正常循迹, 外侧(X1/X8)触发 → 记方向, 进 state1
         *   state 1: 全速直行, 到时后进 state2
         *   state 2: 原地重转, 中间(X4/X5)回中 → 回 state0
         * ─────────────────────────────────────────────────────── */
        uint8_t outer_now = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

        if (pivot_state == 1) {
            /* 直行推进: 定时直行, 到时转 state2 原地转 */
            if (now >= pivot_t_end) {
                pivot_state = 2;
                pivot_t_end = now + PIVOT_TURN_MS;  /* 原地转计时开始 */
            } else {
                motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED); /* 直行推进 */
                break;
            }
        }

        if (pivot_state == 2) {
            /* 原地重转: 计时到了 且 中间传感器回中 才结束
             * 防止计时到了但车还没对准新直道就切回循迹
             * 硬超时2秒: 防止S3/S4永远等不到时原地打转    */
            uint8_t centered = (sensor[3] == ACTIVE_LEVEL) ||
                               (sensor[4] == ACTIVE_LEVEL);
            if ((now >= pivot_t_end && centered) ||
                (now >= pivot_t_end + 2000)) {   /* 硬超时兜底 */
                pivot_state  = 0;
                pivot_unlock = now + 1000;
            } else {
                motor(PIVOT_CREEP + pivot_dir * PIVOT_STEER,
                      PIVOT_CREEP - pivot_dir * PIVOT_STEER);
                break;
            }
        }

        /* state 0: 外侧触发 或 偏差过大(弯道PID啃不动了) → 启动pivot */
        if (pivot_state == 0 && now >= pivot_unlock &&
            (outer_now || abs_err >= 20)) {
            if (outer_now) {
                pivot_dir = (sensor[7] == ACTIVE_LEVEL) ? 1 : -1; /* 右/左 */
            } else {
                pivot_dir = (err > 0) ? 1 : -1;  /* 偏差方向决定转向 */
            }
            pivot_state = 1;
            pivot_t_end = now + PIVOT_ADVANCE_MS;
            motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED);
            break;
        }

        /* ── 闭环驾驶 (直道 + 小弯) ──────────────────────────────
         *  速度二次衰减: err=0→600, err=10→545, err=20→382, err=30→108
         *  小偏差几乎不减速, 只有大弯才压速                         */
        int32_t speed = SPEED_MAX - (abs_err * abs_err) * (SPEED_MAX - TURN_SPEED) / 900;
        int32_t steer = (int32_t)err * STEER_K;

        if (abs_err >= 4) {
            if (steer > 0 && steer <  STEER_MIN) steer =  STEER_MIN;
            if (steer < 0 && steer > -STEER_MIN) steer = -STEER_MIN;
        }
        if (steer >  STEER_MAX) steer =  STEER_MAX;
        if (steer < -STEER_MAX) steer = -STEER_MAX;

        motor(speed + steer, speed - steer);
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
