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
 *   X1(sensor[0]) / X8(sensor[7]) 或持续大偏差触发 → 挂起一个弯
 *   pivot 确认完成后才计数；回到稳定中线后重新允许下一弯
 *   正方形一圈 4 个弯, TARGET_LAPS 圈 = TARGET_LAPS×4 个弯
 *   最后一个弯数到后继续跑完转弯, 回到直道停车
 *
 * 引脚（以 empty.syscfg 为准）:
 *   左电机: PWM3=PA10 / PWM4=PA11 [TIMA1 C0/C1]
 *   右电机: PWM1=PA0  / PWM2=PA1  [TIMA0 C0/C1]
 *   灰度:   AD0=PB25 AD1=PA25 AD2=PA27 OUT=PB20
 *   陀螺仪: UART_GYRO（当前为 UART0 PA28/PA31）
 *   蓝牙:   UART_BT（当前为 UART2 PA21/PA22）
 */

#include "ti_msp_dl_config.h"
#include "hlk_as201.h"
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
#define MOTOR_LEFT_POLARITY  (-1) /* 正命令应对应车体前进；台架确认后可调整 */
#define MOTOR_RIGHT_POLARITY (-1) /* 正命令应对应车体前进；台架确认后可调整 */

/* ── 双轮编码器稳速 (20ms PI；未校准/故障时自动退回原PWM) ── */
#define ENCODER_CLOSED_LOOP_ENABLE       1
#define SPEED_LOOP_DIVIDER               5  /* 5 x 4ms = 20ms             */
#define ENCODER_COUNTS_AT_SPEED_MAX     24  /* SPEED_MAX时每20ms x4计数   */
#define ENCODER_LEFT_SIGN              (+1) /* ENCA/U6; forward must be positive */
#define ENCODER_RIGHT_SIGN             (-1) /* ENCB/U4; forward must be positive */
#define SPEED_PI_KP                     12
#define SPEED_PI_KI                      1
#define SPEED_PI_I_MAX                 240
#define SPEED_CORRECTION_MAX           300
#define SPEED_CORRECTION_SLEW           60
#define SPEED_MIN_CLOSED_LOOP_COUNTS     3
#define SPEED_FEEDBACK_ARM_TICKS         3
#define SPEED_WRONG_DIR_TICKS            3
#define SPEED_NO_PULSE_TICKS            10
#define SPEED_INVALID_TICKS              3
#define SPEED_FAULT_RECOVER_TICKS        5
#define SPEED_FEEDBACK_MAX_MULTIPLIER    4
#define SPEED_FEEDBACK_MAX_MARGIN        8

/* Lost-line protection; the control loop runs every 4 ms. */
#define CONTROL_PERIOD_MS                 4
#define LINE_LOST_SEARCH_SPEED          220
#define LINE_LOST_SEARCH_STEER           80
#define LINE_LOST_SEARCH_TICKS          (120 / CONTROL_PERIOD_MS)
#define LINE_REACQUIRE_TICKS              2

#if LINE_LOST_SEARCH_SPEED <= LINE_LOST_SEARCH_STEER
#error "Lost-line search must keep both wheels driving forward"
#endif

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
#define PIVOT_STEER       360    /* ★ 温和起转差速                         */
#define PIVOT_STEER_MIN   300    /* 底盘实测所需的可靠最低转向差速         */
#define PIVOT_STEER_BOOST 480    /* 坡道堵转确认后的临时增力               */
#define PIVOT_TURN_MS     450    /* ★ 原地转持续时间(ms), 确保转过90°      */
#define PIVOT_MAX_TURN_MS 700    /* 不依赖陀螺的绝对时限, 防止转到180°     */
#define PIVOT_SLOPE_MAX_TURN_MS 1000 /* 确认低角速时允许的坡道转弯时限       */

/* ── 传感器 ── */
#define ACTIVE_LEVEL        1    /* 1=检测到黑线输出高电平                */

/* ── 圈数控制 ── */
#define TARGET_LAPS         5    /* ★ 目标圈数                            */
#define CORNERS_PER_LAP     4    /* 正方形每圈4个弯                       */
#define COUNT_STARTUP_GUARD_MS 1000 /* 上电后暂不检测弯道                  */
#define FINISH_RUNOUT_MS      1000 /* 最后一弯完成后至少继续循迹1秒        */
#define FINISH_STRAIGHT_TICKS    5 /* 连续20ms稳定直线后停车               */
#define CORNER_REARM_TICKS       5 /* 连续20ms稳定中线后允许下一弯         */
#define PIVOT_EXIT_GUARD_MS    100 /* pivot退出后的最短保护时间            */
#define OUTER_CONFIRM_TICKS      2 /* 外侧探头连续8ms才确认                 */

/* ── 陀螺仪辅助 (HLK-AS201, UART_GYRO=UART0 PA28/PA31, 20Hz 主动上报) ──
 *   全部为"叠加增强": 陀螺仪没接/掉线时 gyro_ok=0, 下列逻辑自动跳过,
 *   行为完全退回原纯灰度循迹, 不影响现有效果。                          */
#ifndef GYRO_ENABLE
#define GYRO_ENABLE          1     /* 0=编译期完全禁用陀螺仪辅助            */
#endif
#define GYRO_REPORT_FLAGS   (AS201_SUB_GYRO | AS201_SUB_ANGLE)
#define GYRO_POLL_BUDGET_BYTES 32U /* 控制ISR单次最多解析的UART字节数       */
#define GYRO_FILTER_ALPHA    0.50f /* 20Hz角速度一阶低通系数; 越大响应越快   */
#define GYRO_FRESH_MS         120  /* 控制只使用足够新的样本; 约2个上报周期  */
#define GYRO_DAMP_K          6     /* ★ 直道阻尼增益: steer -= K*gz(°/s)   */
                                   /*   越大越"稳"但反应钝, 0=关直道阻尼    */
#define GYRO_DAMP_SIGN     (+1)    /* ★ 阻尼符号: 若开阻尼后直道更抖, 改 -1 */
#define GYRO_DAMP_ERRLIM     6     /* 仅 |err|<=此值(近直道)才施加阻尼,     */
                                   /*   避免干扰正常过弯                    */
#define GYRO_DAMP_MAX      150     /* 阻尼项绝对上限, 防陀螺尖峰拉爆差速     */
#define GYRO_DAMP_DEADBAND   1.0f  /* 零偏/静止噪声死区(°/s)               */
#define TURN_TARGET_DEG    82.0f   /* ★ 转弯判定角度(°): 累计转过此角+回中→结束 */
                                   /*   略小于90°, 留出惯性余量; 抄近改小/转不够改大 */
#define TURN_EASE_DEG      30.0f   /* 距目标角此度数内连续减速               */
#define TURN_RATE_CRUISE   130.0f  /* 转弯前段目标角速度(°/s)               */
#define TURN_RATE_END       70.0f  /* 接近目标角时的目标角速度(°/s)         */
#define TURN_RATE_K          2.0f  /* 超速反馈增益: 每超1°/s减少的差速       */
#define TURN_STALL_RATE     35.0f  /* 低于此角速度视为可能堵转(°/s)         */
#define TURN_STALL_CONFIRM_TICKS 20 /* 连续20拍=80ms后才启用坡道增力          */
#define TURN_MIN_EXIT_DEG  65.0f   /* 有陀螺时计时回中退出的最小可信转角    */
#define TURN_MAX_DEG       92.0f   /* 漏检灰度时的转角硬上限, 防止转到180°  */
#define TURN_REACQUIRE_MIN_MS 180  /* 起转后至少等待此时长才接受灰度回中    */
#define TURN_CENTER_CONFIRM_TICKS 2 /* 连续8ms回中才退出                      */
#define TURN_YAW_DIR_CONFIRM_DEG 2.0f /* 累计yaw达到此值后才锁定方向          */
#define TURN_YAW_FRAME_MIN    0.5f  /* 单帧低于此角度不参与方向确认          */
#define TURN_WRONG_WAY_DEG   6.0f  /* 已学习方向后反转超过此角安全退出       */
#define GYRO_LOST_MS       300     /* 超过此时间没有新帧→判掉线, 自动退回   */
#define SHARP_ERR_CONFIRM_TICKS 5   /* 非外侧探头触发pivot需连续20ms确认      */

/* ================================================================
 *  全局变量
 * ================================================================ */
static uint8_t   sensor[8];
static int8_t    last_err = 0;
static uint8_t   scan_ch  = 0;
static uint16_t  line_lost_ticks = 0;
static uint8_t   line_recover_ticks = 0;
static int8_t    line_search_dir = 0;

/* ── 圈数计数 ── */
static uint16_t  corner_count    = 0;  /* 已数到的弯数                   */
static uint32_t  corner_detect_after = COUNT_STARTUP_GUARD_MS;
static uint8_t   corner_pending  = 0;  /* 已触发，等待转弯成功后提交计数 */
static uint8_t   corner_armed    = 1;
static uint8_t   corner_rearm_ticks = 0;
static uint8_t   finishing       = 0;  /* 1=最后一个弯已数到, 转完后停车 */
static uint8_t   car_stopped     = 0;  /* 1=已停车                       */
static uint32_t  finish_not_before = 0;
static uint8_t   finish_straight_ticks = 0;

/* ── 直角弯状态机 ── */
/*   0=正常循迹  1=检测到直角,直行推进中  2=原地转中 */
static uint8_t   pivot_state     = 0;
static uint32_t  pivot_t_end     = 0;  /* 当前阶段结束时刻(ms)           */
static int32_t   pivot_dir       = 1;  /* 转向方向: +1右 / -1左          */
static uint32_t  pivot_unlock    = 0;  /* pivot冷却: 转完后锁定,防重复触发 */

/* ── 陀螺仪辅助运行时状态 ── */
#if GYRO_ENABLE
static AS201_Handle g_gyro;             /* 驱动句柄                        */
#endif
static uint8_t   gyro_ok        = 0;    /* 1=陀螺仪在线且数据有效          */
static uint32_t  gyro_last_cnt  = 0;    /* 上次见到的有效帧计数(判掉线)    */
#if GYRO_ENABLE
static uint32_t  gyro_last_ms   = 0;    /* 上次收到新帧的时刻(ms)          */
#endif
static float     gyro_gz        = 0.0f; /* 最新 Z 轴角速度 (°/s)           */
static float     gyro_yaw       = 0.0f; /* 最新航向角 (°)                   */
#if GYRO_ENABLE
static uint8_t   gyro_filter_ready = 0;
#endif
static float     turn_accum_deg = 0.0f; /* pivot 原地转阶段累计转过角度(°) */
static float     turn_abs_deg   = 0.0f; /* 不分方向的总角行程，硬限保护   */
static uint32_t  turn_last_frame = 0;
static float     turn_last_yaw   = 0.0f;
static float     turn_yaw_probe_deg = 0.0f;
static int8_t    turn_yaw_probe_sign = 0;
static uint8_t   turn_yaw_probe_hits = 0;
static float     turn_reverse_deg = 0.0f;
static int8_t    turn_yaw_sign   = 0;
static int8_t    gyro_turn_polarity = 0; /* 右转对应yaw符号，首个成功弯学习 */
static uint8_t   turn_wrong_way  = 0;
static uint8_t   turn_yaw_ready  = 0;
static uint8_t   turn_center_hits = 0;
static uint8_t   turn_line_departed = 0;
static uint32_t  pivot_turn_start_ms = 0;
static uint8_t   turn_stall_ticks = 0;
static uint8_t   turn_boost_active = 0;
static uint8_t   outer_confirm_ticks = 0;
static int8_t    outer_candidate_dir = 0;
static uint8_t   sharp_err_ticks = 0;
static int8_t    sharp_candidate_dir = 0;

typedef struct {
    int32_t command;
    int32_t target;
    int32_t measured;
    int32_t integral;
    int32_t correction;
    int32_t pwm;
    uint8_t armed;
    uint8_t fault;
    uint8_t arm_ticks;
    uint8_t wrong_dir_ticks;
    uint8_t no_pulse_ticks;
    uint8_t invalid_ticks;
    uint8_t recover_ticks;
} WheelSpeedControl;

static WheelSpeedControl speed_left;
static WheelSpeedControl speed_right;
static uint8_t speed_feedback_suspended = 0;
static volatile int32_t encoder_left_window = 0;
static volatile int32_t encoder_right_window = 0;
static volatile uint32_t encoder_left_invalid = 0;
static volatile uint32_t encoder_right_invalid = 0;
static volatile uint8_t encoder_left_state = 0;
static volatile uint8_t encoder_right_state = 0;

/* ── JDY-31 蓝牙调试输出 (每100ms一行, 不影响循迹) ── */
static volatile uint8_t  dbg_ready  = 0;   /* 1=有新数据待发送            */
static volatile float    dbg_gz     = 0.0f;
static volatile float    dbg_yaw    = 0.0f;
static volatile uint8_t  dbg_ok     = 0;
static volatile float    dbg_turns  = 0.0f;
static volatile uint16_t dbg_corner_count = 0;
static volatile int32_t  dbg_l_target = 0;
static volatile int32_t  dbg_l_measured = 0;
static volatile int32_t  dbg_l_pwm = 0;
static volatile int32_t  dbg_r_target = 0;
static volatile int32_t  dbg_r_measured = 0;
static volatile int32_t  dbg_r_pwm = 0;
static volatile uint8_t  dbg_encoder_state = 0;
static volatile uint8_t  dbg_sensor_mask = 0;
static volatile uint16_t dbg_line_lost_ticks = 0;

/* ================================================================
 *  JDY-31 蓝牙串口帮助函数 (不用 printf, 避免引入重量级库)
 *    发送阻塞在 main() 里执行, ISR 可正常打断, 循迹不受影响
 * ================================================================ */
static void bt_putc(uint8_t c)
{
    DL_UART_Main_transmitDataBlocking(UART_BT_INST, c);
}

static void bt_puts(const char *s)
{
    while (*s) bt_putc((uint8_t)*s++);
}

/* 打印有符号整数 */
static void bt_puti(int32_t v)
{
    if (v < 0) { bt_putc('-'); v = -v; }
    if (v >= 10000) bt_putc((uint8_t)('0' + v / 10000 % 10));
    if (v >= 1000)  bt_putc((uint8_t)('0' + v / 1000  % 10));
    if (v >= 100)   bt_putc((uint8_t)('0' + v / 100   % 10));
    if (v >= 10)    bt_putc((uint8_t)('0' + v / 10    % 10));
    bt_putc((uint8_t)('0' + v % 10));
}

/* 打印浮点数, 保留 1 位小数 */
static void bt_putf1(float f)
{
    if (f < 0.0f) { bt_putc('-'); f = -f; }
    int32_t i = (int32_t)f;
    int32_t d = (int32_t)((f - (float)i) * 10.0f + 0.5f);
    if (d >= 10) { i++; d = 0; }   /* 进位 */
    bt_puti(i);
    bt_putc('.');
    bt_putc((uint8_t)('0' + d));
}

/* ================================================================
 *  MUX 选通 (74HC4051)
 * ================================================================ */
static void mux_set(uint8_t ch)
{
    uint8_t c = (ch >> 2) & 1;
    uint8_t b = (ch >> 1) & 1;
    uint8_t a =  ch        & 1;
    if (a) DL_GPIO_setPins(  GRP_GRAY_AD0_PORT, GRP_GRAY_AD0_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_AD0_PORT, GRP_GRAY_AD0_PIN);
    if (b) DL_GPIO_setPins(  GRP_GRAY_AD1_PORT, GRP_GRAY_AD1_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_AD1_PORT, GRP_GRAY_AD1_PIN);
    if (c) DL_GPIO_setPins(  GRP_GRAY_AD2_PORT, GRP_GRAY_AD2_PIN);
    else   DL_GPIO_clearPins(GRP_GRAY_AD2_PORT, GRP_GRAY_AD2_PIN);
}

/* ================================================================
 *  加权位置误差 [-30, +30]  负=偏左  正=偏右
 * ================================================================ */
static const int8_t weight[8] = {-30, -20, -15, -5, 5, 15, 20, 30};

static int8_t calc_err(uint8_t *line_visible)
{
    int16_t sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i] == ACTIVE_LEVEL) { sum += weight[i]; cnt++; }
    }
    *line_visible = (cnt != 0U);
    if (cnt == 0U) return 0;
    last_err = (int8_t)(sum / cnt);
    return last_err;
}

static uint8_t gyro_is_fresh(uint32_t now)
{
#if GYRO_ENABLE
    return gyro_ok && (uint32_t)(now - gyro_last_ms) <= GYRO_FRESH_MS;
#else
    (void)now;
    return 0;
#endif
}

#if GYRO_ENABLE
static float yaw_delta_deg(float current, float previous)
{
    float delta = current - previous;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}
#endif

static uint8_t time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

/* ================================================================
 *  双路正交编码器：四个GPIO双边沿中断，x4状态表译码
 * ================================================================ */
static const int8_t quadrature_step[16] = {
     0,  1, -1,  2,
    -1,  0,  2,  1,
     1,  2,  0, -1,
     2, -1,  1,  0
}; /* 2=非法跳变，通常表示干扰或漏边沿 */

static uint8_t encoder_read_left_state(void)
{
    uint32_t pin_a = DL_GPIO_readPins(
        GRP_ENCODER_LEFT_A_PORT, GRP_ENCODER_LEFT_A_PIN);
    uint32_t pin_b = DL_GPIO_readPins(
        GRP_ENCODER_LEFT_B_PORT, GRP_ENCODER_LEFT_B_PIN);
    return (uint8_t)(((pin_a & GRP_ENCODER_LEFT_A_PIN) ? 2U : 0U) |
                     ((pin_b & GRP_ENCODER_LEFT_B_PIN) ? 1U : 0U));
}

static uint8_t encoder_read_right_state(void)
{
    uint32_t pin_a = DL_GPIO_readPins(
        GRP_ENCODER_RIGHT_A_PORT, GRP_ENCODER_RIGHT_A_PIN);
    uint32_t pin_b = DL_GPIO_readPins(
        GRP_ENCODER_RIGHT_B_PORT, GRP_ENCODER_RIGHT_B_PIN);
    return (uint8_t)(((pin_a & GRP_ENCODER_RIGHT_A_PIN) ? 2U : 0U) |
                     ((pin_b & GRP_ENCODER_RIGHT_B_PIN) ? 1U : 0U));
}

static void encoder_decode_left(void)
{
    uint8_t next = encoder_read_left_state();
    int8_t step = quadrature_step[(encoder_left_state << 2) | next];
    encoder_left_state = next;
    if (step == 1) encoder_left_window++;
    else if (step == -1) encoder_left_window--;
    else if (step == 2) encoder_left_invalid++;
}

static void encoder_decode_right(void)
{
    uint8_t next = encoder_read_right_state();
    int8_t step = quadrature_step[(encoder_right_state << 2) | next];
    encoder_right_state = next;
    if (step == 1) encoder_right_window++;
    else if (step == -1) encoder_right_window--;
    else if (step == 2) encoder_right_invalid++;
}

void GROUP1_IRQHandler(void)
{
    const uint32_t gpioa_right = GRP_ENCODER_RIGHT_A_PIN;
    const uint32_t gpiob_left = GRP_ENCODER_LEFT_A_PIN |
                                GRP_ENCODER_LEFT_B_PIN;
    const uint32_t gpiob_right = GRP_ENCODER_RIGHT_B_PIN;
    const uint32_t gpiob_all = gpiob_left | gpiob_right;

    for (uint8_t retry = 0; retry < 4U; retry++) {
        uint32_t pending_a = DL_GPIO_getEnabledInterruptStatus(
            GRP_ENCODER_RIGHT_A_PORT, gpioa_right);
        uint32_t pending_b = DL_GPIO_getEnabledInterruptStatus(
            GRP_ENCODER_LEFT_A_PORT, gpiob_all);
        if ((pending_a | pending_b) == 0U) break;

        if (pending_a != 0U) {
            DL_GPIO_clearInterruptStatus(
                GRP_ENCODER_RIGHT_A_PORT, pending_a & gpioa_right);
        }
        if (pending_b != 0U) {
            DL_GPIO_clearInterruptStatus(
                GRP_ENCODER_LEFT_A_PORT, pending_b & gpiob_all);
        }
        if ((pending_b & gpiob_left) != 0U) {
            encoder_decode_left();
        }
        if ((pending_a & gpioa_right) != 0U ||
            (pending_b & gpiob_right) != 0U) {
            encoder_decode_right();
        }
    }
}

static void encoder_init(void)
{
    const uint32_t gpioa_mask = GRP_ENCODER_RIGHT_A_PIN;
    const uint32_t gpiob_mask = GRP_ENCODER_LEFT_A_PIN |
                                GRP_ENCODER_LEFT_B_PIN |
                                GRP_ENCODER_RIGHT_B_PIN;

    encoder_left_state = encoder_read_left_state();
    encoder_right_state = encoder_read_right_state();
    DL_GPIO_clearInterruptStatus(GRP_ENCODER_RIGHT_A_PORT, gpioa_mask);
    DL_GPIO_clearInterruptStatus(GRP_ENCODER_LEFT_A_PORT, gpiob_mask);
    NVIC_ClearPendingIRQ(GRP_ENCODER_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GRP_ENCODER_GPIOB_INT_IRQN);
    NVIC_SetPriority(GRP_ENCODER_GPIOA_INT_IRQN, 0);
    NVIC_SetPriority(GRP_ENCODER_GPIOB_INT_IRQN, 0);
    NVIC_EnableIRQ(GRP_ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GRP_ENCODER_GPIOB_INT_IRQN);
}

/* ================================================================
 *  电机底层输出（AT8236；向下计数PWM中 compare=0 为恒高/制动）
 * ================================================================ */
static void motor_pwm(int32_t l, int32_t r)
{
    l = clamp_i32(l, -MOTOR_MAX, MOTOR_MAX);
    r = clamp_i32(r, -MOTOR_MAX, MOTOR_MAX);

    l *= MOTOR_LEFT_POLARITY;
    r *= MOTOR_RIGHT_POLARITY;

    if (l >= 0) {
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, (uint32_t) l, GPIO_PWM_LEFT_C0_IDX);
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, 0U, GPIO_PWM_LEFT_C1_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, 0U, GPIO_PWM_LEFT_C0_IDX);
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, (uint32_t) (-l), GPIO_PWM_LEFT_C1_IDX);
    }

    if (r >= 0) {
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, (uint32_t) r, GPIO_PWM_RIGHT_C0_IDX);
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, 0U, GPIO_PWM_RIGHT_C1_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, 0U, GPIO_PWM_RIGHT_C0_IDX);
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, (uint32_t) (-r), GPIO_PWM_RIGHT_C1_IDX);
    }
}

static int32_t clamp_pwm_to_command(int32_t command, int32_t pwm)
{
    if (command > 0) return clamp_i32(pwm, 0, MOTOR_MAX);
    if (command < 0) return clamp_i32(pwm, -MOTOR_MAX, 0);
    return 0;
}

static int32_t speed_command_to_counts(int32_t command)
{
    int32_t magnitude = (command < 0) ? -command : command;
    int32_t counts = (magnitude * ENCODER_COUNTS_AT_SPEED_MAX +
                      SPEED_MAX / 2) / SPEED_MAX;
    return (command < 0) ? -counts : counts;
}

static void speed_reset_feedback(WheelSpeedControl *wheel, uint8_t clear_fault)
{
    wheel->integral = 0;
    wheel->correction = 0;
    wheel->pwm = wheel->command;
    wheel->armed = 0;
    wheel->arm_ticks = 0;
    wheel->wrong_dir_ticks = 0;
    wheel->no_pulse_ticks = 0;
    wheel->invalid_ticks = 0;
    wheel->recover_ticks = 0;
    if (clear_fault) wheel->fault = 0;
}

static void speed_set_wheel_command(WheelSpeedControl *wheel, int32_t command)
{
    int32_t old_command = wheel->command;
    command = clamp_i32(command, -MOTOR_MAX, MOTOR_MAX);
    uint8_t direction_changed = old_command != 0 && command != 0 &&
        ((old_command < 0) != (command < 0));
    int32_t old_magnitude = (old_command < 0) ? -old_command : old_command;
    int32_t new_magnitude = (command < 0) ? -command : command;
    uint8_t large_drop = old_magnitude > 0 &&
        new_magnitude * 2 <= old_magnitude;

    wheel->command = command;
    wheel->target = speed_command_to_counts(command);
    if (command == 0 || old_command == 0 || direction_changed || large_drop) {
        speed_reset_feedback(wheel, 1);
    } else {
        wheel->pwm = clamp_pwm_to_command(
            command, command + wheel->correction);
    }
}

/* 外环仍使用原PWM尺度；pivot原地转完全旁路PI，避免放大转弯力度。 */
static void motor(int32_t l, int32_t r)
{
#if ENCODER_CLOSED_LOOP_ENABLE
    speed_set_wheel_command(&speed_left, l);
    speed_set_wheel_command(&speed_right, r);
    if (speed_feedback_suspended || pivot_state == 2) {
        speed_reset_feedback(&speed_left, 0);
        speed_reset_feedback(&speed_right, 0);
    }
    motor_pwm(speed_left.pwm, speed_right.pwm);
#else
    motor_pwm(l, r);
#endif
}

static void speed_update_wheel(WheelSpeedControl *wheel, int32_t measured,
                               uint32_t invalid_delta)
{
    wheel->measured = measured;
    wheel->target = speed_command_to_counts(wheel->command);
    int32_t abs_target = (wheel->target < 0) ? -wheel->target : wheel->target;
    int32_t abs_command = (wheel->command < 0) ? -wheel->command : wheel->command;
    int32_t abs_measured = (measured < 0) ? -measured : measured;

    if (wheel->command == 0 || abs_target < SPEED_MIN_CLOSED_LOOP_COUNTS ||
        speed_feedback_suspended || pivot_state == 2) {
        speed_reset_feedback(wheel, wheel->command == 0);
        return;
    }

    uint8_t wrong_direction = measured != 0 &&
        ((measured < 0) != (wheel->target < 0));
    uint8_t plausible = abs_measured <=
        abs_target * SPEED_FEEDBACK_MAX_MULTIPLIER +
        SPEED_FEEDBACK_MAX_MARGIN;
    uint8_t bad_signal = invalid_delta >= 2U ||
        (measured != 0 && !wrong_direction && !plausible);
    uint8_t clean_feedback = measured != 0 && !wrong_direction &&
        plausible && invalid_delta == 0U;

    if (wrong_direction) {
        if (wheel->wrong_dir_ticks < SPEED_WRONG_DIR_TICKS)
            wheel->wrong_dir_ticks++;
    } else {
        wheel->wrong_dir_ticks = 0;
    }
    if (measured == 0) {
        if (wheel->no_pulse_ticks < SPEED_NO_PULSE_TICKS)
            wheel->no_pulse_ticks++;
    } else {
        wheel->no_pulse_ticks = 0;
    }
    if (bad_signal) {
        if (wheel->invalid_ticks < SPEED_INVALID_TICKS)
            wheel->invalid_ticks++;
    } else {
        wheel->invalid_ticks = 0;
    }

    if (wheel->wrong_dir_ticks >= SPEED_WRONG_DIR_TICKS ||
        wheel->no_pulse_ticks >= SPEED_NO_PULSE_TICKS ||
        wheel->invalid_ticks >= SPEED_INVALID_TICKS) {
        wheel->fault = 1;
        wheel->armed = 0;
        wheel->integral = 0;
        wheel->correction = 0;
    }

    if (wheel->fault) {
        if (clean_feedback) {
            if (wheel->recover_ticks < SPEED_FAULT_RECOVER_TICKS)
                wheel->recover_ticks++;
            if (wheel->recover_ticks >= SPEED_FAULT_RECOVER_TICKS) {
                wheel->fault = 0;
                wheel->armed = 1;
                wheel->recover_ticks = 0;
            }
        } else {
            wheel->recover_ticks = 0;
        }
        wheel->pwm = wheel->command;
        if (wheel->fault) return;
    }

    if (!wheel->armed) {
        if (clean_feedback) {
            if (wheel->arm_ticks < SPEED_FEEDBACK_ARM_TICKS)
                wheel->arm_ticks++;
            if (wheel->arm_ticks >= SPEED_FEEDBACK_ARM_TICKS)
                wheel->armed = 1;
        } else {
            wheel->arm_ticks = 0;
        }
        wheel->pwm = wheel->command;
        if (!wheel->armed) return;
    }

    if (wrong_direction || bad_signal) {
        wheel->integral = 0;
        wheel->correction = 0;
        wheel->pwm = wheel->command;
        return;
    }

    int32_t error = wheel->target - measured;
    wheel->integral = clamp_i32(
        wheel->integral + error * SPEED_PI_KI,
        -SPEED_PI_I_MAX, SPEED_PI_I_MAX);
    int32_t correction_limit = abs_command / 2;
    correction_limit = clamp_i32(correction_limit, 0, SPEED_CORRECTION_MAX);
    int32_t requested = clamp_i32(
        error * SPEED_PI_KP + wheel->integral,
        -correction_limit, correction_limit);
    int32_t delta = requested - wheel->correction;
    delta = clamp_i32(delta, -SPEED_CORRECTION_SLEW, SPEED_CORRECTION_SLEW);
    wheel->correction += delta;
    wheel->pwm = clamp_pwm_to_command(
        wheel->command, wheel->command + wheel->correction);
}

static void speed_control_tick(void)
{
#if ENCODER_CLOSED_LOOP_ENABLE
    static uint8_t divider = 0;
    if (++divider < SPEED_LOOP_DIVIDER) return;
    divider = 0;

    int32_t left_count;
    int32_t right_count;
    uint32_t left_invalid;
    uint32_t right_invalid;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    left_count = encoder_left_window;
    right_count = encoder_right_window;
    left_invalid = encoder_left_invalid;
    right_invalid = encoder_right_invalid;
    encoder_left_window = 0;
    encoder_right_window = 0;
    encoder_left_invalid = 0;
    encoder_right_invalid = 0;
    if (primask == 0U) __enable_irq();

    speed_update_wheel(
        &speed_left, left_count * ENCODER_LEFT_SIGN, left_invalid);
    speed_update_wheel(
        &speed_right, right_count * ENCODER_RIGHT_SIGN, right_invalid);
    motor_pwm(speed_left.pwm, speed_right.pwm);
#endif
}

/* Do not reuse a stale steering error when all sensors lose the line. */
static uint8_t line_loss_guard(uint8_t line_visible)
{
    if (line_visible) {
        if (line_lost_ticks == 0U) {
            speed_feedback_suspended = 0;
            return 0;
        }

        if (line_recover_ticks < LINE_REACQUIRE_TICKS) {
            line_recover_ticks++;
        }
        if (line_recover_ticks < LINE_REACQUIRE_TICKS) {
            int32_t steer = line_search_dir * LINE_LOST_SEARCH_STEER;
            speed_feedback_suspended = 1;
            motor(LINE_LOST_SEARCH_SPEED + steer,
                  LINE_LOST_SEARCH_SPEED - steer);
            return 1;
        }

        line_lost_ticks = 0;
        line_recover_ticks = 0;
        speed_feedback_suspended = 0;
        return 0;
    }

    if (line_lost_ticks == 0U) {
        line_search_dir = (last_err > 0) ? 1 : ((last_err < 0) ? -1 : 0);
    }
    if (line_lost_ticks < UINT16_MAX) {
        line_lost_ticks++;
    }
    line_recover_ticks = 0;
    speed_feedback_suspended = 1;

    if (line_lost_ticks <= LINE_LOST_SEARCH_TICKS) {
        int32_t steer = line_search_dir * LINE_LOST_SEARCH_STEER;
        motor(LINE_LOST_SEARCH_SPEED + steer,
              LINE_LOST_SEARCH_SPEED - steer);
    } else {
        motor(0, 0);
    }
    return 1;
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
    (void)AS201_PollBudget(&g_gyro, GYRO_POLL_BUDGET_BYTES);

    /* 融合要求角速度和航向角来自同一有效帧。 */
    if (AS201_HasNewFrame(&g_gyro, &gyro_last_cnt)) {
        const AS201_Data *d = AS201_GetData(&g_gyro);
        const uint8_t required = GYRO_REPORT_FLAGS;
        if (d != NULL && (d->update_flags & required) == required) {
            float raw_gz = d->gyro.z;
            if (!gyro_filter_ready) {
                gyro_gz = raw_gz;
                gyro_filter_ready = 1;
            } else {
                gyro_gz += GYRO_FILTER_ALPHA * (raw_gz - gyro_gz);
            }
            gyro_yaw     = d->angle.yaw;
            gyro_last_ms = now;
            gyro_ok      = 1;
        }
    }
    /* 超时无新帧 → 判掉线, 自动退回纯灰度 */
    if (gyro_ok && (uint32_t)(now - gyro_last_ms) > GYRO_LOST_MS) {
        gyro_ok  = 0;
        gyro_gz  = 0.0f;
        gyro_filter_ready = 0;
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
        speed_control_tick();

        /* ── 调试采样: 每 25 拍(100ms)打包一次, main() 发送 ──────────
         *   只读不写控制变量, 不影响循迹和陀螺仪逻辑              */
        {
            static uint8_t dbg_div = 0;
            if (++dbg_div >= 25) {
                uint8_t sensor_mask = 0;
                for (uint8_t i = 0; i < 8U; i++) {
                    if (sensor[i] == ACTIVE_LEVEL) sensor_mask |= (uint8_t)(1U << i);
                }
                dbg_div   = 0;
                dbg_gz    = gyro_gz;
                dbg_yaw   = gyro_ok ? gyro_yaw : 0.0f;
                dbg_ok    = gyro_ok;
                dbg_turns = turn_accum_deg;
                dbg_corner_count = corner_count;
                dbg_l_target = speed_left.target;
                dbg_l_measured = speed_left.measured;
                dbg_l_pwm = speed_left.pwm;
                dbg_r_target = speed_right.target;
                dbg_r_measured = speed_right.measured;
                dbg_r_pwm = speed_right.pwm;
                dbg_encoder_state = (speed_left.fault ? 1U : 0U) |
                                    (speed_right.fault ? 2U : 0U) |
                                    (speed_left.armed ? 0U : 4U) |
                                    (speed_right.armed ? 0U : 8U);
                dbg_sensor_mask = sensor_mask;
                dbg_line_lost_ticks = line_lost_ticks;
                dbg_ready = 1;
            }
        }

        /* ── 永久停车 ── */
        if (car_stopped) { motor(0, 0); break; }

        uint8_t line_visible = 0;
        int8_t  err     = calc_err(&line_visible);
        int32_t abs_err = (err < 0) ? -err : err;

        /* 最后一弯成功提交后，至少续跑1秒并稳定回到直线才停车。 */
        {
            uint8_t outer = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

            if (finishing) {
                uint8_t on_straight = !outer &&
                                      ((sensor[3] == ACTIVE_LEVEL) ||
                                       (sensor[4] == ACTIVE_LEVEL));
                if (pivot_state == 0 && time_reached(now, finish_not_before) &&
                    on_straight) {
                    if (finish_straight_ticks < FINISH_STRAIGHT_TICKS)
                        finish_straight_ticks++;
                } else {
                    finish_straight_ticks = 0;
                }
                if (finish_straight_ticks >= FINISH_STRAIGHT_TICKS) {
                    car_stopped = 1;
                    motor(0, 0);
                    break;
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
        int8_t outer_dir_now = 0;
        if (sensor[7] == ACTIVE_LEVEL && sensor[0] != ACTIVE_LEVEL) {
            outer_dir_now = 1;
        } else if (sensor[0] == ACTIVE_LEVEL &&
                   sensor[7] != ACTIVE_LEVEL) {
            outer_dir_now = -1;
        } else if (outer_now) {
            outer_dir_now = (err != 0) ? ((err > 0) ? 1 : -1) :
                            ((pivot_dir >= 0) ? 1 : -1);
        }

        if (pivot_state == 1) {
            /* 直行推进: 定时直行, 到时转 state2 原地转 */
            if (time_reached(now, pivot_t_end)) {
                pivot_state = 2;
                pivot_t_end = now + PIVOT_TURN_MS;  /* 原地转计时开始 */
                pivot_turn_start_ms = now;
                turn_accum_deg = 0.0f;              /* 复位转弯角度累计器 */
                turn_abs_deg = 0.0f;
                turn_last_frame = gyro_last_cnt;
                turn_last_yaw   = gyro_yaw;
                turn_yaw_probe_deg = 0.0f;
                turn_yaw_probe_sign = 0;
                turn_yaw_probe_hits = 0;
                turn_reverse_deg = 0.0f;
                turn_yaw_sign   = 0;
                turn_wrong_way  = 0;
                turn_yaw_ready  = gyro_is_fresh(now);
                turn_center_hits = 0;
                turn_line_departed = 0;
                turn_stall_ticks = 0;
                turn_boost_active = 0;
            } else {
                motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED); /* 直行推进 */
                break;
            }
        }

        if (pivot_state == 2) {
            /* 原地重转结束条件:
             *   增强: 用新yaw帧累计净转角并平滑减速; 达到目标且灰度回中
             *         才允许提前结束, 避免只到角度但尚未找回赛道
             *   兜底: 原计时回中与硬超时始终保留, 陀螺掉线不会卡死 */
            uint8_t center_raw = (sensor[2] == ACTIVE_LEVEL) ||
                                 (sensor[3] == ACTIVE_LEVEL) ||
                                 (sensor[4] == ACTIVE_LEVEL) ||
                                 (sensor[5] == ACTIVE_LEVEL);
            if (!center_raw) {
                turn_line_departed = 1;
            }

            uint8_t reacquire_allowed = turn_line_departed &&
                (uint32_t)(now - pivot_turn_start_ms) >= TURN_REACQUIRE_MIN_MS;
            if (center_raw && reacquire_allowed) {
                if (turn_center_hits < TURN_CENTER_CONFIRM_TICKS) {
                    turn_center_hits++;
                }
            } else {
                turn_center_hits = 0;
            }
            uint8_t centered = (turn_center_hits >= TURN_CENTER_CONFIRM_TICKS);
            uint8_t timed_centered =
                (time_reached(now, pivot_t_end) && centered);
            uint32_t max_turn_ms = PIVOT_MAX_TURN_MS;

            int32_t turn_steer = PIVOT_STEER;   /* 默认使用起转全力 */
            uint8_t gyro_fresh = 0;
            uint8_t gyro_target_reached = 0;
            uint8_t gyro_angle_limit = 0;

#if GYRO_ENABLE
            gyro_fresh = gyro_is_fresh(now);
            if (gyro_fresh) {
                if (!turn_yaw_ready) {
                    turn_last_frame = gyro_last_cnt;
                    turn_last_yaw   = gyro_yaw;
                    turn_yaw_probe_deg = 0.0f;
                    turn_yaw_probe_sign = 0;
                    turn_yaw_probe_hits = 0;
                    turn_reverse_deg = 0.0f;
                    turn_yaw_sign   = 0;
                    turn_yaw_ready  = 1;
                } else if (gyro_last_cnt != turn_last_frame) {
                    float delta = yaw_delta_deg(gyro_yaw, turn_last_yaw);
                    turn_last_frame = gyro_last_cnt;
                    turn_last_yaw   = gyro_yaw;

                    float abs_delta = (delta < 0.0f) ? -delta : delta;
                    turn_abs_deg += abs_delta;

                    if (gyro_turn_polarity != 0) {
                        int8_t expected_sign = (pivot_dir > 0) ?
                            gyro_turn_polarity : -gyro_turn_polarity;
                        float directed = delta * (float)expected_sign;
                        turn_yaw_sign = expected_sign;
                        turn_accum_deg += directed;
                        if (turn_accum_deg < 0.0f) turn_accum_deg = 0.0f;
                        if (directed < 0.0f) {
                            turn_reverse_deg += -directed;
                            if (turn_reverse_deg >= TURN_WRONG_WAY_DEG)
                                turn_wrong_way = 1;
                        } else {
                            turn_reverse_deg = 0.0f;
                        }
                    } else if (turn_yaw_sign == 0) {
                        if (abs_delta >= TURN_YAW_FRAME_MIN) {
                            int8_t sample_sign = (delta > 0.0f) ? 1 : -1;
                            if (sample_sign == turn_yaw_probe_sign) {
                                if (turn_yaw_probe_hits < 2U)
                                    turn_yaw_probe_hits++;
                                turn_yaw_probe_deg += delta;
                            } else {
                                turn_yaw_probe_sign = sample_sign;
                                turn_yaw_probe_hits = 1;
                                turn_yaw_probe_deg = delta;
                            }
                        }
                        float abs_probe = (turn_yaw_probe_deg < 0.0f) ?
                            -turn_yaw_probe_deg : turn_yaw_probe_deg;
                        if (turn_yaw_probe_hits >= 2U &&
                            abs_probe >= TURN_YAW_DIR_CONFIRM_DEG) {
                            turn_yaw_sign =
                                (turn_yaw_probe_deg > 0.0f) ? 1 : -1;
                            turn_accum_deg = abs_probe;
                            turn_yaw_probe_deg = 0.0f;
                        }
                    } else {
                        float directed = delta * (float)turn_yaw_sign;
                        turn_accum_deg += directed;
                        if (turn_accum_deg < 0.0f) turn_accum_deg = 0.0f;
                        if (directed < 0.0f) {
                            turn_reverse_deg += -directed;
                            if (turn_reverse_deg >= TURN_WRONG_WAY_DEG)
                                turn_wrong_way = 1;
                        } else {
                            turn_reverse_deg = 0.0f;
                        }
                    }
                }

                float remain = TURN_TARGET_DEG - turn_accum_deg;
                gyro_target_reached =
                    (turn_yaw_sign != 0 && remain <= 0.0f);

                float target_rate = TURN_RATE_CRUISE;
                if (remain < TURN_EASE_DEG) {
                    float ratio = remain / TURN_EASE_DEG;
                    if (ratio < 0.0f) ratio = 0.0f;
                    turn_steer = PIVOT_STEER_MIN +
                                 (int32_t)((PIVOT_STEER - PIVOT_STEER_MIN) * ratio);
                    target_rate = TURN_RATE_END +
                                  (TURN_RATE_CRUISE - TURN_RATE_END) * ratio;
                }

                /* 角速度只做超速抑制: 起步慢时不额外加力，避免突然猛转。 */
                float abs_rate = (gyro_gz < 0.0f) ? -gyro_gz : gyro_gz;
                if (abs_rate > target_rate) {
                    turn_steer -= (int32_t)((abs_rate - target_rate) * TURN_RATE_K);
                }
                if (turn_steer < PIVOT_STEER_MIN) {
                    turn_steer = PIVOT_STEER_MIN;
                }

                if (remain > TURN_EASE_DEG && abs_rate < TURN_STALL_RATE) {
                    if (turn_stall_ticks < TURN_STALL_CONFIRM_TICKS) {
                        turn_stall_ticks++;
                    }
                } else {
                    turn_stall_ticks = 0;
                }
                if (turn_stall_ticks >= TURN_STALL_CONFIRM_TICKS) {
                    turn_steer = PIVOT_STEER_BOOST;
                    turn_boost_active = 1;
                }

                /* 新鲜陀螺数据存在时不允许仅凭计时提交一个低角度弯。 */
                if (turn_accum_deg < TURN_MIN_EXIT_DEG) {
                    timed_centered = 0;
                }
                if ((turn_yaw_sign != 0 || turn_boost_active) &&
                    turn_accum_deg < TURN_MIN_EXIT_DEG) {
                    max_turn_ms = PIVOT_SLOPE_MAX_TURN_MS;
                }
            } else {
                turn_yaw_ready = 0;
                turn_yaw_probe_deg = 0.0f;
                turn_yaw_probe_sign = 0;
                turn_yaw_probe_hits = 0;
            }
            gyro_angle_limit = (turn_abs_deg >= TURN_MAX_DEG);
#endif
            uint8_t fallback_angle_ok = (turn_yaw_sign == 0) ||
                (turn_accum_deg >= TURN_MIN_EXIT_DEG);
            uint8_t turn_confirmed = gyro_fresh ?
                (gyro_target_reached && centered) :
                (timed_centered && fallback_angle_ok);
            uint8_t turn_safety_limited =
                gyro_angle_limit || turn_wrong_way;
            uint8_t turn_timed_out =
                ((uint32_t)(now - pivot_turn_start_ms) >= max_turn_ms);

            if (turn_confirmed || turn_safety_limited || turn_timed_out) {
                pivot_state  = 0;
                turn_boost_active = 0;
                pivot_unlock = now + PIVOT_EXIT_GUARD_MS;

                if (turn_confirmed && !turn_safety_limited &&
                    !turn_timed_out && corner_pending && !finishing) {
                    if (gyro_turn_polarity == 0 && turn_yaw_sign != 0) {
                        gyro_turn_polarity = (pivot_dir > 0) ?
                            turn_yaw_sign : -turn_yaw_sign;
                    }
                    corner_count++;
                    if (corner_count >=
                        (uint16_t)TARGET_LAPS * CORNERS_PER_LAP) {
                        finishing = 1;
                        finish_not_before = now + FINISH_RUNOUT_MS;
                        finish_straight_ticks = 0;
                    }
                }
                corner_pending = 0;
            } else {
                motor(PIVOT_CREEP + pivot_dir * turn_steer,
                      PIVOT_CREEP - pivot_dir * turn_steer);
                break;
            }
        }

        /* Pivot may legitimately lose the line; guard only normal driving. */
        if (line_loss_guard(line_visible)) {
            break;
        }

        /* 完成/放弃一个弯后，必须重新看到稳定中线才允许下一次触发。 */
        uint8_t center_path = (sensor[3] == ACTIVE_LEVEL) ||
                              (sensor[4] == ACTIVE_LEVEL);
        if (!corner_armed && !finishing && pivot_state == 0 &&
            time_reached(now, pivot_unlock) && !outer_now && center_path) {
            if (corner_rearm_ticks < CORNER_REARM_TICKS)
                corner_rearm_ticks++;
            if (corner_rearm_ticks >= CORNER_REARM_TICKS) {
                corner_armed = 1;
                corner_rearm_ticks = 0;
            }
        } else if (!corner_armed) {
            corner_rearm_ticks = 0;
        }

        /* 外侧和大偏差均需方向一致的连续采样，过滤单拍毛刺。 */
        if (!finishing && corner_armed && pivot_state == 0 &&
            time_reached(now, pivot_unlock) &&
            time_reached(now, corner_detect_after) && outer_dir_now != 0) {
            if (outer_candidate_dir != outer_dir_now) {
                outer_candidate_dir = outer_dir_now;
                outer_confirm_ticks = 1;
            } else if (outer_confirm_ticks < OUTER_CONFIRM_TICKS) {
                outer_confirm_ticks++;
            }
        } else if (pivot_state == 0) {
            outer_confirm_ticks = 0;
            outer_candidate_dir = 0;
        }
        uint8_t outer_confirmed =
            (outer_confirm_ticks >= OUTER_CONFIRM_TICKS);

        if (!finishing && corner_armed && pivot_state == 0 &&
            time_reached(now, pivot_unlock) &&
            time_reached(now, corner_detect_after) && line_visible &&
            abs_err >= 20) {
            int8_t sharp_dir_now = (err > 0) ? 1 : -1;
            if (sharp_candidate_dir != sharp_dir_now) {
                sharp_candidate_dir = sharp_dir_now;
                sharp_err_ticks = 1;
            } else if (sharp_err_ticks < SHARP_ERR_CONFIRM_TICKS) {
                sharp_err_ticks++;
            }
        } else if (pivot_state == 0) {
            sharp_err_ticks = 0;
            sharp_candidate_dir = 0;
        }
        uint8_t sharp_err_confirmed =
            (sharp_err_ticks >= SHARP_ERR_CONFIRM_TICKS);

        if (!finishing && corner_armed && pivot_state == 0 &&
            time_reached(now, pivot_unlock) &&
            time_reached(now, corner_detect_after) &&
            (outer_confirmed || sharp_err_confirmed)) {
            pivot_dir = outer_confirmed ?
                outer_candidate_dir : sharp_candidate_dir;
            corner_pending = 1;
            corner_armed = 0;
            corner_rearm_ticks = 0;
            outer_confirm_ticks = 0;
            outer_candidate_dir = 0;
            sharp_err_ticks = 0;
            sharp_candidate_dir = 0;
            pivot_state = 1;
            pivot_t_end = now + PIVOT_ADVANCE_MS;
            motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED);
            break;
        }

        /* ── 闭环驾驶 (直道 + 小弯) ──────────────────────────────
         *  速度二次衰减: err=0→600, err=10→556, err=20→423, err=30→200
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
        if (gyro_is_fresh(now) && line_visible && GYRO_DAMP_K > 0 &&
            abs_err <= GYRO_DAMP_ERRLIM) {
            float damp_dps = gyro_gz;
            if (damp_dps > -GYRO_DAMP_DEADBAND &&
                damp_dps < GYRO_DAMP_DEADBAND) {
                damp_dps = 0.0f;
            }
            int32_t damp = (int32_t)(GYRO_DAMP_SIGN * GYRO_DAMP_K * damp_dps);
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

    /* SysTick: follow the CPU clock generated from empty.syscfg. */
    SysTick->LOAD = (CPUCLK_FREQ / 1000UL) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;

    mux_set(0);

    motor(0, 0);
    DL_TimerA_startCounter(PWM_RIGHT_INST);
    DL_TimerA_startCounter(PWM_LEFT_INST);
    encoder_init();

    /* ── 陀螺仪初始化 (可选增强, 失败/未接不影响循迹) ──────────────
     *  UART_GYRO 的引脚/波特率/8N1 已由 SYSCFG_DL_init() 配好, 这里
     *  只挂中断. 随后按需配置 GYRO+ANGLE、20Hz 与主动上报并回读确认.
     *  配置期间 TIMER 控制环尚未启动, 独占 AS201_Poll, 无竞争.
     *  未接陀螺时配置会在 500ms 内超时返回, gyro_ok 保持 0,
     *  控制环全程走纯灰度分支, 行为与原工程一致。                    */
#if GYRO_ENABLE
    if (AS201_Init(&g_gyro, UART_GYRO_INST, UART_GYRO_INST_INT_IRQN) == AS201_OK) {
        NVIC_SetPriority(UART_GYRO_INST_INT_IRQN, 1);
        AS201_EnsureReportConfig(&g_gyro, GYRO_REPORT_FLAGS,
                                 AS201_RATE_20HZ, true, 500, &g_millis);
    }
#endif

    /* 优先级：编码器0 > 陀螺UART1 > 控制定时器2。 */
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) {
        __WFI();

        /* ── JDY-31 蓝牙调试输出 ─────────────────────────────────
         *   ISR 每 100ms 采样一次置 dbg_ready, 这里阻塞发送。
         *   115200 波特率约 100 字符 ≈ 9ms, 发送期间 ISR 照常打断,
         *   循迹和陀螺仪逻辑完全不受影响。                        */
        uint8_t dbg_has_sample = 0;
        float dbg_gz_snapshot = 0.0f;
        float dbg_yaw_snapshot = 0.0f;
        uint8_t dbg_ok_snapshot = 0;
        float dbg_turns_snapshot = 0.0f;
        uint16_t dbg_corner_snapshot = 0;
        int32_t dbg_l_target_snapshot = 0;
        int32_t dbg_l_measured_snapshot = 0;
        int32_t dbg_l_pwm_snapshot = 0;
        int32_t dbg_r_target_snapshot = 0;
        int32_t dbg_r_measured_snapshot = 0;
        int32_t dbg_r_pwm_snapshot = 0;
        uint8_t dbg_encoder_state_snapshot = 0;
        uint8_t dbg_sensor_mask_snapshot = 0;
        uint16_t dbg_line_lost_snapshot = 0;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (dbg_ready) {
            dbg_ready = 0;
            dbg_gz_snapshot = dbg_gz;
            dbg_yaw_snapshot = dbg_yaw;
            dbg_ok_snapshot = dbg_ok;
            dbg_turns_snapshot = dbg_turns;
            dbg_corner_snapshot = dbg_corner_count;
            dbg_l_target_snapshot = dbg_l_target;
            dbg_l_measured_snapshot = dbg_l_measured;
            dbg_l_pwm_snapshot = dbg_l_pwm;
            dbg_r_target_snapshot = dbg_r_target;
            dbg_r_measured_snapshot = dbg_r_measured;
            dbg_r_pwm_snapshot = dbg_r_pwm;
            dbg_encoder_state_snapshot = dbg_encoder_state;
            dbg_sensor_mask_snapshot = dbg_sensor_mask;
            dbg_line_lost_snapshot = dbg_line_lost_ticks;
            dbg_has_sample = 1;
        }
        if (primask == 0U) {
            __enable_irq();
        }

        if (dbg_has_sample) {
            /* es: bit0/1左右故障，bit2/3左右闭环尚未建立。 */
            bt_puts("gz:");    bt_putf1(dbg_gz_snapshot);
            bt_puts(" yaw:");  bt_putf1(dbg_yaw_snapshot);
            bt_puts(" ok:");   bt_putc((uint8_t)('0' + dbg_ok_snapshot));
            bt_puts(" turns:"); bt_putf1(dbg_turns_snapshot);
            bt_puts(" cc:"); bt_puti(dbg_corner_snapshot);
            bt_puts(" lc:"); bt_puti(dbg_l_target_snapshot);
            bt_putc('/'); bt_puti(dbg_l_measured_snapshot);
            bt_putc('/'); bt_puti(dbg_l_pwm_snapshot);
            bt_puts(" rc:"); bt_puti(dbg_r_target_snapshot);
            bt_putc('/'); bt_puti(dbg_r_measured_snapshot);
            bt_putc('/'); bt_puti(dbg_r_pwm_snapshot);
            bt_puts(" es:"); bt_puti(dbg_encoder_state_snapshot);
            bt_puts(" sm:"); bt_puti(dbg_sensor_mask_snapshot);
            bt_puts(" ll:"); bt_puti(dbg_line_lost_snapshot);
            bt_puts("\r\n");
        }
    }
}
