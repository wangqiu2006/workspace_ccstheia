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
#include "hlk_as201.h"
#include <stdint.h>
#include <math.h>

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

/* ── 陀螺仪辅助 (HLK-AS201, UART_GYRO=UART1 PA8/PA9, 20Hz 主动上报) ──
 *   全部为"叠加增强": 陀螺仪没接/掉线时 gyro_ok=0, 下列逻辑自动跳过,
 *   行为完全退回原纯灰度循迹, 不影响现有效果。                          */
#define GYRO_ENABLE          1     /* 0=编译期完全禁用陀螺仪辅助            */
#define GYRO_DAMP_K          6     /* ★ 直道阻尼增益: steer -= K*gz(°/s)   */
                                   /*   越大越"稳"但反应钝, 0=关直道阻尼    */
#define GYRO_DAMP_SIGN     (+1)    /* ★ 阻尼符号: 若开阻尼后直道更抖, 改 -1 */
#define GYRO_DAMP_ERRLIM     6     /* 仅 |err|<=此值(近直道)才施加阻尼,     */
                                   /*   避免干扰正常过弯                    */
#define GYRO_DAMP_MAX      150     /* 阻尼项绝对上限, 防陀螺尖峰拉爆差速     */
#define TURN_TARGET_DEG    82.0f   /* ★ 转弯判定角度(°): 累计转过此角+回中→结束 */
                                   /*   略小于90°, 留出惯性余量; 抄近改小/转不够改大 */
#define TURN_EASE_DEG      25.0f   /* 距目标角此度数内开始减速→转弯更平滑   */
#define GYRO_LOST_MS       300     /* 超过此时间没有新帧→判掉线, 自动退回   */

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

/* ── 陀螺仪辅助运行时状态 ── */
static AS201_Handle g_gyro;             /* 驱动句柄                        */
static uint8_t   gyro_ok        = 0;    /* 1=陀螺仪在线且数据有效          */
static uint32_t  gyro_last_cnt  = 0;    /* 上次见到的有效帧计数(判掉线)    */
static uint32_t  gyro_last_ms   = 0;    /* 上次收到新帧的时刻(ms)          */
static float     gyro_gz        = 0.0f; /* 最新 Z 轴角速度 (°/s)           */
static float     turn_accum_deg = 0.0f; /* pivot 原地转阶段累计转过角度(°) */
/* 控制环固定 4ms 一拍 (scan_ch 归零), 积分用此固定 dt, 无需时间戳 */
#define CTRL_DT_S          0.004f

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
 *  陀螺仪 UART RX 中断 — 只把字节收进环形缓冲, 解析在控制环里做
 * ================================================================ */
#if GYRO_ENABLE
void UART_GYRO_INST_IRQHandler(void)
{
    AS201_ISR(&g_gyro);
}
#endif

/* ================================================================
 *  陀螺仪状态更新 — 在控制环 (250Hz) 里调用
 *    解析缓冲 → 取最新 gz → 用帧计数+超时判断在线/掉线
 * ================================================================ */
static void gyro_update(uint32_t now)
{
#if GYRO_ENABLE
    AS201_Poll(&g_gyro);

    /* 有新数据帧: 刷新 gz 与在线时刻 */
    if (AS201_HasNewFrame(&g_gyro, &gyro_last_cnt)) {
        const AS201_Data *d = AS201_GetData(&g_gyro);
        if (d != NULL && (d->update_flags & AS201_SUB_GYRO)) {
            gyro_gz      = d->gyro.z;   /* °/s, 绕 Z 轴 (车体转向轴) */
            gyro_last_ms = now;
            gyro_ok      = 1;
        }
    }
    /* 超时无新帧 → 判掉线, 自动退回纯灰度 */
    if (gyro_ok && (uint32_t)(now - gyro_last_ms) > GYRO_LOST_MS) {
        gyro_ok  = 0;
        gyro_gz  = 0.0f;
    }
#else
    (void)now;
#endif
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

        /* ── 陀螺仪解析/在线判定 (掉线自动退回纯灰度) ── */
        gyro_update(now);

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
                turn_accum_deg = 0.0f;              /* 复位转弯角度累计器 */
            } else {
                motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED); /* 直行推进 */
                break;
            }
        }

        if (pivot_state == 2) {
            /* 原地重转结束条件:
             *   基础(无陀螺/掉线): 原计时到 且 中间回中, 或硬超时2秒 —— 原逻辑不变
             *   增强(陀螺在线): 累计转角接近 TURN_TARGET_DEG 时按角度平滑收尾,
             *                   接近目标角前 TURN_EASE_DEG 内线性减小转速→不过冲、更平滑
             * 两条件是"或"关系: 满足任一即结束, 陀螺只会让它更早更稳地收尾, 不会卡死 */
            uint8_t centered = (sensor[3] == ACTIVE_LEVEL) ||
                               (sensor[4] == ACTIVE_LEVEL);

            int32_t turn_steer = PIVOT_STEER;   /* 默认全力(退回原行为) */

#if GYRO_ENABLE
            if (gyro_ok) {
                /* 4ms 定周期积分 gz(°/s) → 累计转过的角度(取绝对值, 与安装方向无关) */
                float dps = gyro_gz;
                float inc = dps * 0.004f;                 /* 4ms */
                turn_accum_deg += (inc < 0.0f) ? -inc : inc;

                float remain = TURN_TARGET_DEG - turn_accum_deg;
                if (remain <= 0.0f) {
                    /* 已转够角度 → 立即结束, 不必等计时/回中, 一致性最好 */
                    pivot_state  = 0;
                    pivot_unlock = now + 1000;
                    break;
                }
                if (remain < TURN_EASE_DEG) {
                    /* 进入缓冲区: 转速随剩余角度线性降低, 最低留 30% 防转不动 */
                    float ratio = remain / TURN_EASE_DEG;      /* 1→0 */
                    if (ratio < 0.30f) ratio = 0.30f;
                    turn_steer = (int32_t)(PIVOT_STEER * ratio);
                }
            }
#endif
            if ((now >= pivot_t_end && centered) ||
                (now >= pivot_t_end + 2000)) {   /* 硬超时兜底 */
                pivot_state  = 0;
                pivot_unlock = now + 1000;
            } else {
                motor(PIVOT_CREEP + pivot_dir * turn_steer,
                      PIVOT_CREEP - pivot_dir * turn_steer);
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

        /* ── 陀螺仪直道阻尼 ──────────────────────────────────────
         *  仅近直道(|err|小)且陀螺在线时施加: 用角速度 gz 做反向阻尼,
         *  抵消灰度离散误差引起的过冲摆动 → 直线更稳不抖。
         *  gz>0 表示车正在往某方向偏转, 就往反方向补差速把它按住。
         *  掉线(gyro_ok=0)时此项为 0, 完全等价原逻辑。               */
#if GYRO_ENABLE
        if (gyro_ok && GYRO_DAMP_K > 0 && abs_err <= GYRO_DAMP_ERRLIM) {
            int32_t damp = (int32_t)(GYRO_DAMP_SIGN * GYRO_DAMP_K * gyro_gz);
            if (damp >  GYRO_DAMP_MAX) damp =  GYRO_DAMP_MAX;
            if (damp < -GYRO_DAMP_MAX) damp = -GYRO_DAMP_MAX;
            steer -= damp;
        }
#endif

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

    /* ── 陀螺仪初始化 (可选增强, 失败/未接不影响循迹) ──────────────
     *  UART_GYRO 的引脚/波特率/8N1 已由 SYSCFG_DL_init() 配好, 这里
     *  只挂中断. 随后回读一次模块配置, 同步订阅掩码 (防非全订阅误判).
     *  SyncConfig 期间 TIMER 控制环尚未启动, 独占 AS201_Poll, 无竞争.
     *  未接陀螺时 SyncConfig 会在 500ms 内超时返回, gyro_ok 保持 0,
     *  控制环全程走纯灰度分支, 行为与原工程一致。                    */
#if GYRO_ENABLE
    AS201_Init(&g_gyro, UART_GYRO_INST, UART_GYRO_INST_INT_IRQN);
    AS201_SyncConfig(&g_gyro, 500, &g_millis);
#endif

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) { __WFI(); }
}
