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
 * 引脚:
 *   AT8236 A桥/左轮: PWM4=PA11(正), PWM3=PA10(反) [TIMA1 CC1/CC0]
 *   AT8236 B桥/右轮: PWM2=PA12(正), PWM1=PA7 (反) [TIMA0 CC3/CC1]
 *   灰度: AD0=PB25 AD1=PA25 AD2=PA27 OUT=PB20
 *   左编码器: A=PB23 B=PB27; 右编码器: A=PA18 B=PB14
 *   陀螺仪: UART0 TX=PA28 RX=PA31; 蓝牙: UART2 TX=PA21 RX=PA22
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

/* ── 双轮编码器稳速 (MG513X-P28, A/B 双通道, GPIO 软件 x4 译码) ──
 * 目标单位是每20ms的x4计数，不依赖轮径。先观察蓝牙 lc/rc 的 measured，
 * 再把 ENCODER_COUNTS_AT_SPEED_MAX 调成空载现有速度，可保持加重前后速度。 */
#define ENCODER_CLOSED_LOOP_ENABLE       1
#define SPEED_LOOP_DIVIDER               5  /* 5×4ms = 20ms              */
#define SPEED_LOOP_NOMINAL_MS            20
#define SPEED_SAMPLE_MIN_MS             16  /* 异常采样窗直接重同步         */
#define SPEED_SAMPLE_MAX_MS             28
#define ENCODER_COUNTS_AT_SPEED_MAX     24  /* ★ SPEED_MAX时每20ms计数    */
#define ENCODER_LEFT_SIGN              (+1) /* ★ 前进时计数须为正          */
#define ENCODER_RIGHT_SIGN             (-1) /* ★ 若前进为负则对调A/B或改符号 */
#define SPEED_PI_KP                     18  /* PWM/计数                    */
#define SPEED_PI_KI                      2  /* PWM/计数/20ms               */
#define SPEED_PI_I_MAX                 600  /* 积分项限幅                   */
#define SPEED_PI_CORRECTION_MAX        900  /* 相对原PWM的最大补偿          */
#define SPEED_WRONG_DIR_CONFIRM_TICKS    3  /* 60ms反向后退出闭环防失控     */
#define SPEED_REVERSE_GRACE_TICKS        8  /* 换向后最多160ms等待惯性消退  */
#define SPEED_REVERSE_READY_TICKS        2  /* 连续40ms方向正确即可提前闭环 */
#define SPEED_FEEDBACK_ARM_TICKS          3  /* 连续60ms可信后才首次闭环     */
#define SPEED_FEEDBACK_MAX_MULTIPLIER     4  /* 超过目标4倍视为脉冲噪声     */
#define SPEED_FEEDBACK_MAX_MARGIN         8  /* 低目标时的合理计数余量       */
#define SPEED_MIN_CLOSED_LOOP_COUNTS     3  /* 低速量化太粗时保持原开环行为 */
#define SPEED_NO_PULSE_CONFIRM_TICKS    10  /* 可信目标下200ms无脉冲判故障  */
#define SPEED_PIVOT_NO_PULSE_TICKS      25  /* 原地转允许500ms有限抗堵转    */
#define SPEED_INVALID_CONFIRM_TICKS      3  /* 连续60ms非法跳变判信号故障   */
#define SPEED_INVALID_MIN_PER_TICK        2  /* 单窗至少2次非法跳变才累计故障 */
#define SPEED_FAULT_RECOVER_TICKS        5  /* 连续100ms可信反馈自动恢复     */
#define SPEED_NO_PULSE_CORRECTION_MAX  400  /* 堵转/掉线确认前的补偿上限    */
#define SPEED_STARTUP_ASSIST_DELAY_TICKS  2  /* 40ms未动后才开始有限助力      */
#define SPEED_STARTUP_ASSIST_MIN_COUNTS    8  /* 低速/转弯内轮不启用起步助力  */
#define SPEED_STARTUP_ASSIST_STEP        50  /* 每20ms增加的起步助力PWM       */
#define SPEED_STARTUP_ASSIST_MAX        250  /* 未确认编码器时绝不超过此助力  */
#define SPEED_PIVOT_CORRECTION_MAX      120  /* 原地转只允许小幅轮速修正      */
#define SPEED_CORRECTION_SLEW_PER_TICK  120  /* 每20ms补偿变化上限            */

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
#define COUNT_STARTUP_GUARD_MS 1000 /* 上电后暂不计弯，防起跑误触发          */
#define FINISH_RUNOUT_MS   1000  /* 最后一弯完成后至少继续循迹的时间        */
#define CORNER_REARM_TICKS    5  /* 连续20ms稳定中线后允许检测下一弯       */
#define PIVOT_EXIT_GUARD_MS 100  /* 退出pivot后的最短状态稳定时间          */

/* ── 陀螺仪辅助 (HLK-AS201, UART_GYRO=UART0 PA28/PA31, 20Hz 主动上报) ──
 *   全部为"叠加增强": 陀螺仪没接/掉线时 gyro_ok=0, 下列逻辑自动跳过,
 *   行为完全退回原纯灰度循迹, 不影响现有效果。                          */
#ifndef GYRO_ENABLE
#define GYRO_ENABLE          1     /* 0=编译期禁用陀螺仪算法辅助            */
#endif
#define GYRO_REPORT_FLAGS   (AS201_SUB_GYRO | AS201_SUB_ANGLE)
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
#define TURN_CENTER_CONFIRM_TICKS 1 /* 角度门控后单拍回中即可退出             */
#define TURN_YAW_DIR_CONFIRM_DEG 2.0f /* 累计净yaw达到此值后才锁定转向符号    */
#define TURN_WRONG_WAY_DEG   6.0f  /* 已学习方向后，反转超过此角立即放弃     */
#define GYRO_LOST_MS       300     /* 超过此时间没有新帧→判掉线, 自动退回   */
#define OUTER_CONFIRM_TICKS  2     /* 外侧探头连续8ms有效才确认直角弯        */
#define SHARP_ERR_CONFIRM_TICKS 5   /* 非外侧探头触发pivot需连续20ms确认      */

/* ================================================================
 *  全局变量
 * ================================================================ */
static uint8_t   sensor[8];
static int8_t    last_err = 0;
static uint8_t   scan_ch  = 0;

/* ── 圈数计数 ── */
static uint16_t  corner_count    = 0;  /* 已数到的弯数                   */
static uint32_t  corner_unlock   = COUNT_STARTUP_GUARD_MS;
static uint8_t   corner_pending  = 0;  /* 1=已触发，等待转弯完成后提交计数 */
static uint8_t   corner_armed    = 1;  /* 1=允许检测一个新弯               */
static uint8_t   corner_rearm_ticks = 0;
static uint8_t   finishing       = 0;  /* 1=最后一个弯已数到, 转完后停车 */
static uint8_t   car_stopped     = 0;  /* 1=已停车                       */

/* ── 直角弯状态机 ── */
/*   0=正常循迹  1=检测到直角,直行推进中  2=原地转中 */
static uint8_t   pivot_state     = 0;
static uint32_t  pivot_t_end     = 0;  /* 当前阶段结束时刻(ms)           */
static int32_t   pivot_dir       = 1;  /* 转向方向: +1右 / -1左          */
static uint32_t  pivot_unlock    = 0;  /* pivot退出后的短时状态保护       */

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
static float     turn_abs_deg   = 0.0f; /* 不分方向的总角行程，用于硬保护    */
static uint32_t  turn_last_frame = 0;
static float     turn_last_yaw   = 0.0f;
static float     turn_yaw_probe_deg = 0.0f; /* 定向确认/锁定后反转检测累计量   */
static int8_t    turn_yaw_sign   = 0;    /* 净yaw确认后的本次转弯正方向      */
#if GYRO_ENABLE
static int8_t    gyro_turn_polarity = 0; /* yaw符号/pivot_dir，首个成功弯学习 */
#endif
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

/* ── 双轮编码器与内环稳速 ── */
typedef struct {
    int32_t command;          /* 外环给出的原PWM尺度速度命令              */
    int32_t target;           /* 每20ms目标x4计数                         */
    int32_t measured;         /* 每20ms实测x4计数                         */
    int32_t integral;         /* PI积分输出(PWM单位)                      */
    int32_t pwm;              /* 实际下发PWM                              */
    uint8_t wrong_dir_ticks;  /* 编码器方向错误连续计数                    */
    uint8_t reverse_grace_ticks; /* 换向后的开环宽限期                     */
    uint8_t no_pulse_ticks;   /* 有可信速度目标但无脉冲的连续计数          */
    uint8_t invalid_ticks;    /* 异常反馈窗口连续计数                      */
    uint8_t good_feedback_ticks; /* 故障恢复/换向确认计数                  */
    uint8_t feedback_seen;    /* 当前方向已收到过可信反馈                  */
    uint8_t direction_fault;  /* 1=方向异常，已回退原PWM前馈               */
    uint8_t signal_fault;     /* 1=无脉冲/非法跳变，已回退原PWM前馈        */
} WheelSpeedControl;

static WheelSpeedControl speed_left;
static WheelSpeedControl speed_right;
static volatile uint32_t encoder_left_count  = 0;
static volatile uint32_t encoder_right_count = 0;
static volatile uint32_t encoder_left_invalid  = 0;
static volatile uint32_t encoder_right_invalid = 0;
static volatile uint8_t encoder_left_state  = 0;
static volatile uint8_t encoder_right_state = 0;
static uint32_t encoder_left_last  = 0;
static uint32_t encoder_right_last = 0;
static uint32_t encoder_left_invalid_control_last  = 0;
static uint32_t encoder_right_invalid_control_last = 0;

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
static volatile uint8_t  dbg_encoder_fault = 0;

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

static uint8_t line_is_visible(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i] == ACTIVE_LEVEL) return 1;
    }
    return 0;
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

/* ================================================================
 *  双路正交编码器: 四个GPIO双边沿中断，x4状态表译码
 * ================================================================ */
static const int8_t quadrature_step[16] = {
     0,  1, -1,  2,
    -1,  0,  2,  1,
     1,  2,  0, -1,
     2, -1,  1,  0
}; /* 2=非法跳变，通常表示干扰或ISR漏边沿 */

static uint8_t encoder_read_left_state(void)
{
    uint32_t pins = DL_GPIO_readPins(GRP_ENCODER_LEFT_A_PORT,
                                     GRP_ENCODER_LEFT_A_PIN |
                                     GRP_ENCODER_LEFT_B_PIN);
    return (uint8_t)(((pins & GRP_ENCODER_LEFT_A_PIN) ? 2U : 0U) |
                     ((pins & GRP_ENCODER_LEFT_B_PIN) ? 1U : 0U));
}

static uint8_t encoder_read_right_state(void)
{
    uint32_t pin_a = DL_GPIO_readPins(GRP_ENCODER_RIGHT_A_PORT,
                                      GRP_ENCODER_RIGHT_A_PIN);
    uint32_t pin_b = DL_GPIO_readPins(GRP_ENCODER_RIGHT_B_PORT,
                                      GRP_ENCODER_RIGHT_B_PIN);
    return (uint8_t)(((pin_a & GRP_ENCODER_RIGHT_A_PIN) ? 2U : 0U) |
                     ((pin_b & GRP_ENCODER_RIGHT_B_PIN) ? 1U : 0U));
}

static void encoder_decode_left(void)
{
    uint8_t next = encoder_read_left_state();
    int8_t step = quadrature_step[(encoder_left_state << 2) | next];
    encoder_left_state = next;
    if (step == 1) {
        encoder_left_count++;
    } else if (step == -1) {
        encoder_left_count--;
    } else if (step == 2) {
        encoder_left_invalid++;
    }
}

static void encoder_decode_right(void)
{
    uint8_t next = encoder_read_right_state();
    int8_t step = quadrature_step[(encoder_right_state << 2) | next];
    encoder_right_state = next;
    if (step == 1) {
        encoder_right_count++;
    } else if (step == -1) {
        encoder_right_count--;
    } else if (step == 2) {
        encoder_right_invalid++;
    }
}

void GROUP1_IRQHandler(void)
{
    const uint32_t gpioa_right_mask = GRP_ENCODER_RIGHT_A_PIN;
    const uint32_t gpiob_left_mask = GRP_ENCODER_LEFT_A_PIN |
                                     GRP_ENCODER_LEFT_B_PIN;
    const uint32_t gpiob_right_mask = GRP_ENCODER_RIGHT_B_PIN;
    const uint32_t gpiob_mask = gpiob_left_mask | gpiob_right_mask;

    /* PA18在GPIOA，其余编码器脚在GPIOB；按端口清除后每轮只译码一次。 */
    for (uint8_t retry = 0; retry < 4; retry++) {
        uint32_t pending_gpioa = DL_GPIO_getEnabledInterruptStatus(
            GRP_ENCODER_RIGHT_A_PORT, gpioa_right_mask);
        uint32_t pending_gpiob = DL_GPIO_getEnabledInterruptStatus(
            GRP_ENCODER_LEFT_A_PORT, gpiob_mask);

        if ((pending_gpioa | pending_gpiob) == 0U) break;
        if (pending_gpioa != 0U) {
            DL_GPIO_clearInterruptStatus(GRP_ENCODER_RIGHT_A_PORT,
                                         pending_gpioa & gpioa_right_mask);
        }
        if (pending_gpiob != 0U) {
            DL_GPIO_clearInterruptStatus(GRP_ENCODER_LEFT_A_PORT,
                                         pending_gpiob & gpiob_mask);
        }
        if ((pending_gpiob & gpiob_left_mask) != 0U) {
            encoder_decode_left();
        }
        if (pending_gpioa != 0U ||
            (pending_gpiob & gpiob_right_mask) != 0U) {
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

    encoder_left_state  = encoder_read_left_state();
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
 *  电机PWM底层  (AT8236: motor_pwm(0,0) = 制动)
 * ================================================================ */
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

/* 速度内环只能减到0制动，绝不能擅自把外环命令反向。 */
static int32_t clamp_pwm_to_command_direction(int32_t command, int32_t pwm)
{
    if (command > 0) return clamp_i32(pwm, 0, MOTOR_MAX);
    if (command < 0) return clamp_i32(pwm, -MOTOR_MAX, 0);
    return 0;
}

static int32_t speed_pivot_correction_limit(void)
{
    return turn_boost_active ?
        0 : SPEED_PIVOT_CORRECTION_MAX;
}

static void speed_decay_correction(WheelSpeedControl *wheel)
{
    int32_t correction = wheel->pwm - wheel->command;
    if (correction > SPEED_CORRECTION_SLEW_PER_TICK) {
        correction -= SPEED_CORRECTION_SLEW_PER_TICK;
    } else if (correction < -SPEED_CORRECTION_SLEW_PER_TICK) {
        correction += SPEED_CORRECTION_SLEW_PER_TICK;
    } else {
        correction = 0;
    }
    wheel->pwm = clamp_pwm_to_command_direction(
        wheel->command, wheel->command + correction);
}

static void motor_pwm(int32_t l, int32_t r)
{
    l = clamp_i32(l, -MOTOR_MAX, MOTOR_MAX);
    r = clamp_i32(r, -MOTOR_MAX, MOTOR_MAX);

    /*
     * 向下计数PWM中 compare=0 为恒高。先把目标方向输入置为恒高，
     * 让换向经过 IN1=IN2=高的制动状态，再在另一输入写入驱动力。
     */
    if (l >= 0) {
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, 0, GPIO_PWM_LEFT_C1_IDX);       /* PWM4: 正 */
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, l, GPIO_PWM_LEFT_C0_IDX);       /* PWM3: 调速 */
    } else {
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, 0, GPIO_PWM_LEFT_C0_IDX);       /* PWM3: 反 */
        DL_TimerA_setCaptureCompareValue(
            PWM_LEFT_INST, -l, GPIO_PWM_LEFT_C1_IDX);      /* PWM4: 调速 */
    }
    if (r >= 0) {
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, 0, GPIO_PWM_RIGHT_C3_IDX);     /* PWM2: 正 */
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, r, GPIO_PWM_RIGHT_C1_IDX);     /* PWM1: 调速 */
    } else {
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, 0, GPIO_PWM_RIGHT_C1_IDX);     /* PWM1: 反 */
        DL_TimerA_setCaptureCompareValue(
            PWM_RIGHT_INST, -r, GPIO_PWM_RIGHT_C3_IDX);    /* PWM2: 调速 */
    }
}

static int32_t speed_command_to_counts(int32_t command)
{
    int32_t magnitude = (command < 0) ? -command : command;
    int32_t counts = (magnitude * ENCODER_COUNTS_AT_SPEED_MAX + SPEED_MAX / 2) /
                     SPEED_MAX;
    return (command < 0) ? -counts : counts;
}

static void speed_set_wheel_command(WheelSpeedControl *wheel, int32_t command)
{
    int32_t old_command = wheel->command;
    command = clamp_i32(command, -MOTOR_MAX, MOTOR_MAX);
    int32_t new_target = speed_command_to_counts(command);
    uint8_t direction_changed = old_command != 0 && command != 0 &&
        ((old_command < 0) != (command < 0));
    int32_t old_magnitude = (old_command < 0) ? -old_command : old_command;
    int32_t new_magnitude = (command < 0) ? -command : command;
    uint8_t large_speed_drop = old_magnitude > 0 &&
        new_magnitude * 2 <= old_magnitude;

    if (command == 0) {
        wheel->command = 0;
        wheel->target = 0;
        wheel->integral = 0;
        wheel->pwm = 0;
        wheel->wrong_dir_ticks = 0;
        wheel->reverse_grace_ticks = 0;
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        wheel->good_feedback_ticks = 0;
        wheel->feedback_seen = 0;
        wheel->direction_fault = 0;
        wheel->signal_fault = 0;
        return;
    }

    /* 目标低于编码器分辨率时立即清旧补偿，避免弯道内轮跨零反转。 */
    int32_t abs_target = (new_target < 0) ? -new_target : new_target;
    if (abs_target < SPEED_MIN_CLOSED_LOOP_COUNTS) {
        wheel->command = command;
        wheel->target = new_target;
        wheel->integral = 0;
        wheel->pwm = command;
        wheel->wrong_dir_ticks = 0;
        if (direction_changed) {
            wheel->reverse_grace_ticks = SPEED_REVERSE_GRACE_TICKS;
        } else if (old_command == 0) {
            wheel->reverse_grace_ticks = 0;
        }
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        wheel->good_feedback_ticks = 0;
        if (old_command == 0 || direction_changed) {
            wheel->feedback_seen = 0;
            wheel->direction_fault = 0;
            wheel->signal_fault = 0;
        }
        return;
    }

    if (old_command == 0 || direction_changed) {
        wheel->integral = 0;
        wheel->wrong_dir_ticks = 0;
        wheel->reverse_grace_ticks = (old_command != 0) ?
            SPEED_REVERSE_GRACE_TICKS : 0;
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        wheel->good_feedback_ticks = 0;
        wheel->feedback_seen = 0;
        wheel->direction_fault = 0;
        wheel->signal_fault = 0;
        wheel->pwm = command;
    } else {
        /* 外环命令变化立即体现在PWM上，同时保留当前PI补偿。 */
        int32_t preserved_correction = wheel->pwm - old_command;
        if (large_speed_drop) {
            preserved_correction = 0;
            wheel->integral = 0;
        }
        if (pivot_state == 2) {
            int32_t pivot_limit = speed_pivot_correction_limit();
            preserved_correction = clamp_i32(preserved_correction,
                -pivot_limit, pivot_limit);
            wheel->integral = 0;
        }
        wheel->pwm = clamp_pwm_to_command_direction(
            command, command + preserved_correction);
    }
    wheel->command = command;
    wheel->target = new_target;
}

/* 外环仍按原PWM尺度调用；底层将其解释为轮速目标并立即施加前馈。 */
static void motor(int32_t l, int32_t r)
{
    speed_set_wheel_command(&speed_left, l);
    speed_set_wheel_command(&speed_right, r);
    motor_pwm(speed_left.pwm, speed_right.pwm);
}

static void speed_update_wheel(WheelSpeedControl *wheel, int32_t measured,
                               uint32_t invalid_delta, uint32_t elapsed_ms)
{
    wheel->measured = measured;
    wheel->target = speed_command_to_counts(wheel->command);

    if (wheel->command == 0 || wheel->target == 0) {
        wheel->integral = 0;
        wheel->pwm = wheel->command;
        wheel->wrong_dir_ticks = 0;
        if (wheel->reverse_grace_ticks > 0) {
            wheel->reverse_grace_ticks--;
        }
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        wheel->good_feedback_ticks = 0;
        return;
    }

    int32_t abs_target = (wheel->target < 0) ? -wheel->target : wheel->target;
    int32_t abs_measured = (measured < 0) ? -measured : measured;
    uint8_t no_pulse_limit = (pivot_state == 2) ?
        SPEED_PIVOT_NO_PULSE_TICKS : SPEED_NO_PULSE_CONFIRM_TICKS;
    uint8_t wrong_direction = measured != 0 &&
        ((measured < 0) != (wheel->target < 0));
    uint8_t plausible_feedback = abs_measured <=
        abs_target * SPEED_FEEDBACK_MAX_MULTIPLIER + SPEED_FEEDBACK_MAX_MARGIN;
    uint8_t invalid_window =
        invalid_delta >= SPEED_INVALID_MIN_PER_TICK &&
        invalid_delta > (((uint32_t)abs_measured + 3U) / 4U);
    uint8_t implausible_window = measured != 0 && !wrong_direction &&
        !plausible_feedback;
    uint8_t bad_signal_window = invalid_window || implausible_window;
    uint8_t clean_feedback = measured != 0 && !wrong_direction &&
        plausible_feedback && !invalid_window;

    /* 低速时单周期只有0~2个脉冲，PI会被量化噪声驱动；保留原开环更稳。 */
    if (abs_target < SPEED_MIN_CLOSED_LOOP_COUNTS) {
        wheel->integral = 0;
        wheel->pwm = wheel->command;
        wheel->wrong_dir_ticks = 0;
        if (wheel->reverse_grace_ticks > 0) {
            wheel->reverse_grace_ticks--;
        }
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        wheel->good_feedback_ticks = 0;
        return;
    }

    /* H桥换向时先开环等待；新方向连续可信后可早于160ms恢复闭环。 */
    if (wheel->reverse_grace_ticks > 0) {
        wheel->reverse_grace_ticks--;
        wheel->integral = 0;
        wheel->pwm = wheel->command;
        wheel->wrong_dir_ticks = 0;
        wheel->no_pulse_ticks = 0;
        wheel->invalid_ticks = 0;
        if (clean_feedback) {
            if (wheel->good_feedback_ticks < SPEED_REVERSE_READY_TICKS) {
                wheel->good_feedback_ticks++;
            }
            if (wheel->good_feedback_ticks >= SPEED_REVERSE_READY_TICKS) {
                wheel->reverse_grace_ticks = 0;
                wheel->good_feedback_ticks = 0;
                wheel->feedback_seen = 1;
                return;
            } else {
                return;
            }
        } else {
            wheel->good_feedback_ticks = 0;
            return;
        }
    }

    /* 严重非法跳变或同向超量脉冲不更新PI，避免噪声造成瞬时制动。 */
    if (bad_signal_window) {
        if (wheel->invalid_ticks < SPEED_INVALID_CONFIRM_TICKS) {
            wheel->invalid_ticks++;
        }
        wheel->good_feedback_ticks = 0;
        if (wheel->invalid_ticks >= SPEED_INVALID_CONFIRM_TICKS) {
            wheel->signal_fault = 1;
        }
        wheel->integral = 0;
        if (wheel->direction_fault || wheel->signal_fault) {
            wheel->pwm = wheel->command;
        } else {
            speed_decay_correction(wheel);
        }
        return;
    }
    wheel->invalid_ticks = 0;

    if (wrong_direction) {
        if (wheel->wrong_dir_ticks < SPEED_WRONG_DIR_CONFIRM_TICKS) {
            wheel->wrong_dir_ticks++;
        }
    } else if (wheel->wrong_dir_ticks > 0) {
        wheel->wrong_dir_ticks--;
    }

    uint8_t plausible_missing = measured == 0 ||
        (!wrong_direction && !plausible_feedback);
    if (plausible_missing) {
        if (wheel->no_pulse_ticks < no_pulse_limit) {
            wheel->no_pulse_ticks++;
        }
    } else if (wheel->no_pulse_ticks > 0) {
        wheel->no_pulse_ticks--;
    }

    if (wheel->wrong_dir_ticks >= SPEED_WRONG_DIR_CONFIRM_TICKS) {
        wheel->direction_fault = 1;
    }
    if (wrong_direction) {
        wheel->good_feedback_ticks = 0;
        wheel->integral = 0;
        if (wheel->direction_fault || wheel->signal_fault) {
            wheel->pwm = wheel->command;
        } else {
            speed_decay_correction(wheel);
        }
        return;
    }
    if (wheel->no_pulse_ticks >= no_pulse_limit) {
        wheel->signal_fault = 1;
    }
    if (wheel->direction_fault || wheel->signal_fault) {
        if (clean_feedback) {
            if (wheel->good_feedback_ticks < SPEED_FAULT_RECOVER_TICKS) {
                wheel->good_feedback_ticks++;
            }
        } else {
            wheel->good_feedback_ticks = 0;
        }

        /* 故障期间保持原开环；连续可信反馈后从零积分平滑恢复。 */
        wheel->integral = 0;
        wheel->pwm = wheel->command;
        if (wheel->good_feedback_ticks >= SPEED_FAULT_RECOVER_TICKS) {
            wheel->direction_fault = 0;
            wheel->signal_fault = 0;
            wheel->wrong_dir_ticks = 0;
            wheel->no_pulse_ticks = 0;
            wheel->invalid_ticks = 0;
            wheel->good_feedback_ticks = 0;
            wheel->feedback_seen = 1;
        }
        return;
    }

    /* 首次闭环前要求连续可信反馈；静摩擦时只给有限、渐进的起步助力。 */
    if (!wheel->feedback_seen) {
        wheel->integral = 0;
        wheel->pwm = wheel->command;
        if (clean_feedback) {
            if (wheel->good_feedback_ticks < SPEED_FEEDBACK_ARM_TICKS) {
                wheel->good_feedback_ticks++;
            }
            if (wheel->good_feedback_ticks >= SPEED_FEEDBACK_ARM_TICKS) {
                wheel->feedback_seen = 1;
                wheel->good_feedback_ticks = 0;
            }
            return;
        }

        wheel->good_feedback_ticks = 0;
        uint8_t both_wheels_same_direction =
            speed_left.command != 0 && speed_right.command != 0 &&
            ((speed_left.command < 0) == (speed_right.command < 0));
        if (abs_target >= SPEED_STARTUP_ASSIST_MIN_COUNTS &&
            both_wheels_same_direction && pivot_state != 2 && measured == 0 &&
            wheel->no_pulse_ticks > SPEED_STARTUP_ASSIST_DELAY_TICKS) {
            int32_t assist =
                (wheel->no_pulse_ticks - SPEED_STARTUP_ASSIST_DELAY_TICKS) *
                SPEED_STARTUP_ASSIST_STEP;
            if (assist > SPEED_STARTUP_ASSIST_MAX) {
                assist = SPEED_STARTUP_ASSIST_MAX;
            }
            if (wheel->target < 0) assist = -assist;
            wheel->pwm = clamp_pwm_to_command_direction(
                wheel->command, wheel->command + assist);
        }
        return;
    }
    wheel->good_feedback_ticks = 0;

#if ENCODER_CLOSED_LOOP_ENABLE
    int32_t error = wheel->target - measured;
    int32_t correction_limit = (measured == 0) ?
        SPEED_NO_PULSE_CORRECTION_MAX : SPEED_PI_CORRECTION_MAX;
    if (pivot_state == 2) {
        int32_t pivot_limit = speed_pivot_correction_limit();
        if (correction_limit > pivot_limit) correction_limit = pivot_limit;
    }
    if (pivot_state == 2) wheel->integral = 0;

    int32_t integral_step = SPEED_PI_KI * error * (int32_t)elapsed_ms /
                            SPEED_LOOP_NOMINAL_MS;
    int32_t integral_next = (pivot_state == 2) ? 0 : clamp_i32(
        wheel->integral + integral_step,
        -SPEED_PI_I_MAX, SPEED_PI_I_MAX);
    int32_t proportional = SPEED_PI_KP * error;
    int32_t correction_raw = proportional + integral_next;
    int32_t correction_low = -correction_limit;
    int32_t correction_high = correction_limit;

    /* 同时把PWM零点和MOTOR_MAX纳入抗饱和边界。 */
    if (wheel->command > 0) {
        if (correction_low < -wheel->command) {
            correction_low = -wheel->command;
        }
        if (correction_high > MOTOR_MAX - wheel->command) {
            correction_high = MOTOR_MAX - wheel->command;
        }
    } else {
        if (correction_low < -MOTOR_MAX - wheel->command) {
            correction_low = -MOTOR_MAX - wheel->command;
        }
        if (correction_high > -wheel->command) {
            correction_high = -wheel->command;
        }
    }

    /* 补偿饱和时冻结同方向积分，防止脱困后速度冲高。 */
    if ((correction_raw > correction_high && error > 0) ||
        (correction_raw < correction_low && error < 0)) {
        integral_next = wheel->integral;
        correction_raw = proportional + integral_next;
    }
    wheel->integral = integral_next;
    int32_t correction = clamp_i32(correction_raw,
        correction_low, correction_high);
    int32_t current_correction = wheel->pwm - wheel->command;
    if (current_correction > 0) {
        if (correction > current_correction + SPEED_CORRECTION_SLEW_PER_TICK) {
            correction = current_correction + SPEED_CORRECTION_SLEW_PER_TICK;
        } else if (correction < -SPEED_CORRECTION_SLEW_PER_TICK) {
            correction = -SPEED_CORRECTION_SLEW_PER_TICK;
        }
    } else if (current_correction < 0) {
        if (correction < current_correction - SPEED_CORRECTION_SLEW_PER_TICK) {
            correction = current_correction - SPEED_CORRECTION_SLEW_PER_TICK;
        } else if (correction > SPEED_CORRECTION_SLEW_PER_TICK) {
            correction = SPEED_CORRECTION_SLEW_PER_TICK;
        }
    } else {
        correction = clamp_i32(correction,
            -SPEED_CORRECTION_SLEW_PER_TICK,
            SPEED_CORRECTION_SLEW_PER_TICK);
    }
    correction = clamp_i32(correction, correction_low, correction_high);
    wheel->pwm = clamp_pwm_to_command_direction(
        wheel->command, wheel->command + correction);
#else
    wheel->integral = 0;
    wheel->pwm = wheel->command;
#endif
}

static int32_t speed_scale_delta(int32_t delta, uint32_t elapsed_ms)
{
    if (delta >= 0) {
        return (delta * SPEED_LOOP_NOMINAL_MS + (int32_t)(elapsed_ms / 2U)) /
               (int32_t)elapsed_ms;
    }
    return -(((-delta) * SPEED_LOOP_NOMINAL_MS +
              (int32_t)(elapsed_ms / 2U)) / (int32_t)elapsed_ms);
}

static void speed_resync_wheel(WheelSpeedControl *wheel)
{
    wheel->measured = 0;
    wheel->integral = 0;
    wheel->pwm = wheel->command;
    wheel->wrong_dir_ticks = 0;
    wheel->no_pulse_ticks = 0;
    wheel->invalid_ticks = 0;
    wheel->good_feedback_ticks = 0;
    wheel->feedback_seen = 0;
}

static void speed_control_tick(uint32_t now)
{
    static uint8_t divider = 0;
    static uint8_t sample_ready = 0;
    static uint8_t last_pivot_mode = 0;
    static uint32_t sample_last_ms = 0;
    if (++divider < SPEED_LOOP_DIVIDER) return;
    divider = 0;

    uint32_t left_now = encoder_left_count;
    uint32_t right_now = encoder_right_count;
    uint32_t left_invalid_now = encoder_left_invalid;
    uint32_t right_invalid_now = encoder_right_invalid;
    int32_t left_delta = (int32_t)(left_now - encoder_left_last);
    int32_t right_delta = (int32_t)(right_now - encoder_right_last);
    uint32_t left_invalid_delta =
        left_invalid_now - encoder_left_invalid_control_last;
    uint32_t right_invalid_delta =
        right_invalid_now - encoder_right_invalid_control_last;
    encoder_left_last = left_now;
    encoder_right_last = right_now;
    encoder_left_invalid_control_last = left_invalid_now;
    encoder_right_invalid_control_last = right_invalid_now;

    uint8_t pivot_mode = (pivot_state == 2);
    if (pivot_mode != last_pivot_mode) {
        speed_left.integral = 0;
        speed_right.integral = 0;
        speed_left.wrong_dir_ticks = 0;
        speed_right.wrong_dir_ticks = 0;
        speed_left.no_pulse_ticks = 0;
        speed_right.no_pulse_ticks = 0;
        speed_left.invalid_ticks = 0;
        speed_right.invalid_ticks = 0;
        speed_left.good_feedback_ticks = 0;
        speed_right.good_feedback_ticks = 0;
        last_pivot_mode = pivot_mode;
    }

    uint32_t elapsed_ms = now - sample_last_ms;
    sample_last_ms = now;
    if (!sample_ready || elapsed_ms < SPEED_SAMPLE_MIN_MS ||
        elapsed_ms > SPEED_SAMPLE_MAX_MS) {
        sample_ready = 1;
        speed_resync_wheel(&speed_left);
        speed_resync_wheel(&speed_right);
        motor_pwm(speed_left.pwm, speed_right.pwm);
        return;
    }

    left_delta = speed_scale_delta(left_delta, elapsed_ms);
    right_delta = speed_scale_delta(right_delta, elapsed_ms);

    speed_update_wheel(&speed_left, left_delta * ENCODER_LEFT_SIGN,
                       left_invalid_delta, elapsed_ms);
    speed_update_wheel(&speed_right, right_delta * ENCODER_RIGHT_SIGN,
                       right_invalid_delta, elapsed_ms);
    motor_pwm(speed_left.pwm, speed_right.pwm);
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

        /* ── 每20ms读取双轮编码器并更新PI稳速 ── */
        speed_control_tick(now);

        /* ── 陀螺仪解析/在线判定 (掉线自动退回纯灰度) ── */
        gyro_update(now);

        /* ── 调试采样: 每 25 拍(100ms)打包一次, main() 发送 ──────────
         *   只读不写控制变量, 不影响循迹和陀螺仪逻辑              */
        {
            static uint8_t dbg_div = 0;
            static uint32_t dbg_l_invalid_last = 0;
            static uint32_t dbg_r_invalid_last = 0;
            if (++dbg_div >= 25) {
                uint32_t dbg_l_invalid_snapshot = encoder_left_invalid;
                uint32_t dbg_r_invalid_snapshot = encoder_right_invalid;
                dbg_div   = 0;
                dbg_gz    = gyro_gz;
                dbg_yaw   = gyro_ok ? gyro_yaw : 0.0f;
                dbg_ok    = gyro_ok;
                dbg_turns = turn_accum_deg;
                dbg_corner_count = corner_count;
                dbg_l_target   = speed_left.target;
                dbg_l_measured = speed_left.measured;
                dbg_l_pwm      = speed_left.pwm;
                dbg_r_target   = speed_right.target;
                dbg_r_measured = speed_right.measured;
                dbg_r_pwm      = speed_right.pwm;
                dbg_encoder_fault =
                    (speed_left.direction_fault ? 1U : 0U) |
                    (speed_right.direction_fault ? 2U : 0U) |
                    (speed_left.signal_fault ? 16U : 0U) |
                    (speed_right.signal_fault ? 32U : 0U);
                if (dbg_l_invalid_snapshot != dbg_l_invalid_last) {
                    dbg_encoder_fault |= 4U;
                }
                if (dbg_r_invalid_snapshot != dbg_r_invalid_last) {
                    dbg_encoder_fault |= 8U;
                }
                dbg_l_invalid_last = dbg_l_invalid_snapshot;
                dbg_r_invalid_last = dbg_r_invalid_snapshot;
                dbg_ready = 1;
            }
        }

        /* ── 永久停车 ── */
        if (car_stopped) { motor(0, 0); break; }

        int8_t  err     = calc_err();
        int32_t abs_err = (err < 0) ? -err : err;
        uint8_t line_visible = line_is_visible();

        /* 最后一弯完成后，回到直道再停车。计弯在 pivot 启动处完成。 */
        {
            uint8_t outer = (sensor[0] == ACTIVE_LEVEL) ||
                            (sensor[7] == ACTIVE_LEVEL);

            if (finishing) {
                uint8_t on_straight = !outer &&
                                      ((sensor[3] == ACTIVE_LEVEL) ||
                                       (sensor[4] == ACTIVE_LEVEL));
                if (pivot_state == 0 &&
                    time_reached(now, corner_unlock) && on_straight) {
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
            if (err != 0) {
                outer_dir_now = (err > 0) ? 1 : -1;
            } else {
                /* 对称宽横线没有瞬时方向信息，沿用上一实际转弯方向。 */
                outer_dir_now = (pivot_dir >= 0) ? 1 : -1;
            }
        }

        if (pivot_state == 1) {
            /* 直行推进: 定时直行, 到时转 state2 原地转 */
            if (time_reached(now, pivot_t_end)) {
                pivot_state = 2;
                pivot_t_end = now + PIVOT_TURN_MS;  /* 原地转计时开始 */
                pivot_turn_start_ms = now;
                turn_accum_deg = 0.0f;              /* 复位转弯角度累计器 */
                turn_abs_deg   = 0.0f;
                turn_last_frame = gyro_last_cnt;
                turn_last_yaw   = gyro_yaw;
                turn_yaw_probe_deg = 0.0f;
                turn_yaw_sign   = 0;
                turn_wrong_way  = 0;
                turn_yaw_ready  = gyro_is_fresh(now);
                turn_center_hits = 0;
                turn_line_departed = 0;
                turn_stall_ticks = 0;
            } else {
                motor(PIVOT_ADV_SPEED, PIVOT_ADV_SPEED); /* 直行推进 */
                break;
            }
        }

        if (pivot_state == 2) {
            turn_boost_active = 0;
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

#if GYRO_ENABLE
            uint8_t gyro_fresh = gyro_is_fresh(now);
            uint8_t gyro_target_reached = 0;
            uint8_t gyro_angle_limit = 0;
            if (gyro_fresh) {
                if (!turn_yaw_ready) {
                    turn_last_frame = gyro_last_cnt;
                    turn_last_yaw   = gyro_yaw;
                    turn_yaw_probe_deg = 0.0f;
                    turn_yaw_sign   = 0;
                    turn_yaw_ready  = 1;
                } else if (gyro_last_cnt != turn_last_frame) {
                    float delta = yaw_delta_deg(gyro_yaw, turn_last_yaw);
                    turn_last_frame = gyro_last_cnt;
                    turn_last_yaw   = gyro_yaw;
                    float abs_delta = (delta < 0.0f) ? -delta : delta;
                    turn_abs_deg += abs_delta;

                    if (turn_yaw_sign == 0) {
                        if (gyro_turn_polarity != 0) {
                            int8_t expected_sign = (pivot_dir > 0) ?
                                gyro_turn_polarity : -gyro_turn_polarity;
                            turn_yaw_probe_deg +=
                                delta * (float)expected_sign;
                            if (turn_yaw_probe_deg >=
                                TURN_YAW_DIR_CONFIRM_DEG) {
                                turn_yaw_sign = expected_sign;
                                turn_accum_deg += turn_yaw_probe_deg;
                                turn_yaw_probe_deg = 0.0f;
                            } else if (turn_yaw_probe_deg <=
                                       -TURN_WRONG_WAY_DEG) {
                                turn_wrong_way = 1;
                            }
                        } else {
                            turn_yaw_probe_deg += delta;
                            float abs_probe = (turn_yaw_probe_deg < 0.0f) ?
                                -turn_yaw_probe_deg : turn_yaw_probe_deg;
                            if (abs_probe >= TURN_YAW_DIR_CONFIRM_DEG) {
                                turn_yaw_sign =
                                    (turn_yaw_probe_deg > 0.0f) ? 1 : -1;
                                turn_accum_deg += abs_probe;
                                turn_yaw_probe_deg = 0.0f;
                            }
                        }
                    } else {
                        float directed_delta =
                            delta * (float)turn_yaw_sign;
                        turn_accum_deg += directed_delta;
                        if (turn_accum_deg < 0.0f) turn_accum_deg = 0.0f;
                        if (gyro_turn_polarity != 0) {
                            turn_yaw_probe_deg += directed_delta;
                            if (turn_yaw_probe_deg > 0.0f) {
                                turn_yaw_probe_deg = 0.0f;
                            } else if (turn_yaw_probe_deg <=
                                       -TURN_WRONG_WAY_DEG) {
                                turn_wrong_way = 1;
                            }
                        }
                    }
                }

                float remain = TURN_TARGET_DEG - turn_accum_deg;
                gyro_target_reached = (remain <= 0.0f);
                gyro_angle_limit = (turn_abs_deg >= TURN_MAX_DEG);

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

                if ((turn_yaw_sign != 0 || turn_boost_active) &&
                    turn_accum_deg < TURN_MIN_EXIT_DEG) {
                    timed_centered = 0;
                    max_turn_ms = PIVOT_SLOPE_MAX_TURN_MS;
                }
            } else {
                turn_yaw_ready = 0;
                turn_yaw_probe_deg = 0.0f;
                turn_stall_ticks = 0;
            }
#endif
            uint8_t turn_confirmed =
#if GYRO_ENABLE
                (gyro_target_reached && centered) ||
#endif
                timed_centered;
            uint8_t turn_safety_limited =
#if GYRO_ENABLE
                gyro_angle_limit || turn_wrong_way;
#else
                0;
#endif
            uint8_t turn_timed_out =
                ((uint32_t)(now - pivot_turn_start_ms) >= max_turn_ms);
            if (turn_confirmed || turn_safety_limited || turn_timed_out) {
                pivot_state  = 0;
                turn_boost_active = 0;
                pivot_unlock = now + PIVOT_EXIT_GUARD_MS;

                if (turn_confirmed && !turn_safety_limited &&
                    !turn_timed_out && corner_pending && !finishing) {
#if GYRO_ENABLE
                    if (gyro_turn_polarity == 0 && turn_yaw_sign != 0) {
                        gyro_turn_polarity = (pivot_dir > 0) ?
                            turn_yaw_sign : -turn_yaw_sign;
                    }
#endif
                    corner_count++;
                    if (corner_count >=
                        (uint16_t)TARGET_LAPS * CORNERS_PER_LAP) {
                        finishing = 1;
                        corner_unlock = now + FINISH_RUNOUT_MS;
                    }
                }
                corner_pending = 0;
            } else {
                motor(PIVOT_CREEP + pivot_dir * turn_steer,
                      PIVOT_CREEP - pivot_dir * turn_steer);
                break;
            }
        }

        /* 完成一个弯后必须重新看到稳定中线，不能仅靠固定时间解锁。 */
        uint8_t center_path = (sensor[3] == ACTIVE_LEVEL) ||
                              (sensor[4] == ACTIVE_LEVEL);
        if (!corner_armed && !finishing && pivot_state == 0 &&
            time_reached(now, pivot_unlock) && !outer_now && center_path) {
            if (corner_rearm_ticks < CORNER_REARM_TICKS) {
                corner_rearm_ticks++;
            }
            if (corner_rearm_ticks >= CORNER_REARM_TICKS) {
                corner_armed = 1;
                corner_rearm_ticks = 0;
            }
        } else if (!corner_armed) {
            corner_rearm_ticks = 0;
        }

        /* state 0: 外侧与大偏差都需连续确认，过滤丢线和单帧毛刺。 */
        if (!finishing && corner_armed && pivot_state == 0 &&
            time_reached(now, pivot_unlock) &&
            time_reached(now, corner_unlock) && outer_dir_now != 0) {
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
            time_reached(now, corner_unlock) && line_visible && abs_err >= 20) {
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
        uint8_t sharp_err_confirmed = (sharp_err_ticks >= SHARP_ERR_CONFIRM_TICKS);

        if (!finishing && corner_armed && pivot_state == 0 &&
            time_reached(now, pivot_unlock) &&
            time_reached(now, corner_unlock) &&
            (outer_confirmed || sharp_err_confirmed)) {
            if (outer_confirmed) {
                pivot_dir = outer_candidate_dir;
            } else {
                pivot_dir = sharp_candidate_dir;
            }

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

    /* SysTick跟随SysConfig主频，保持1ms节拍。 */
    SysTick->LOAD = (CPUCLK_FREQ / 1000UL) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;

    mux_set(0);

    motor_pwm(0, 0);
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
        AS201_EnsureReportConfig(&g_gyro, GYRO_REPORT_FLAGS,
                                 AS201_RATE_20HZ, true, 500, &g_millis);
    }
#endif

    /* 编码器中断优先级0可抢占控制环，避免高转速时丢边沿。 */
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) {
        __WFI();

        /* ── JDY-31 蓝牙调试输出 ─────────────────────────────────
         *   ISR 每 100ms 采样一次置 dbg_ready, 这里阻塞发送。
         *   115200 波特率约 90 字符 ≈ 8ms, 发送期间 ISR 照常打断,
         *   循迹和陀螺仪逻辑完全不受影响。                        */
        uint8_t dbg_has_sample = 0;
        float dbg_gz_snapshot = 0.0f;
        float dbg_yaw_snapshot = 0.0f;
        uint8_t dbg_ok_snapshot = 0;
        float dbg_turns_snapshot = 0.0f;
        uint16_t dbg_corner_count_snapshot = 0;
        int32_t dbg_l_target_snapshot = 0;
        int32_t dbg_l_measured_snapshot = 0;
        int32_t dbg_l_pwm_snapshot = 0;
        int32_t dbg_r_target_snapshot = 0;
        int32_t dbg_r_measured_snapshot = 0;
        int32_t dbg_r_pwm_snapshot = 0;
        uint8_t dbg_encoder_fault_snapshot = 0;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (dbg_ready) {
            dbg_ready = 0;
            dbg_gz_snapshot = dbg_gz;
            dbg_yaw_snapshot = dbg_yaw;
            dbg_ok_snapshot = dbg_ok;
            dbg_turns_snapshot = dbg_turns;
            dbg_corner_count_snapshot = dbg_corner_count;
            dbg_l_target_snapshot = dbg_l_target;
            dbg_l_measured_snapshot = dbg_l_measured;
            dbg_l_pwm_snapshot = dbg_l_pwm;
            dbg_r_target_snapshot = dbg_r_target;
            dbg_r_measured_snapshot = dbg_r_measured;
            dbg_r_pwm_snapshot = dbg_r_pwm;
            dbg_encoder_fault_snapshot = dbg_encoder_fault;
            dbg_has_sample = 1;
        }
        if (primask == 0U) {
            __enable_irq();
        }

        if (dbg_has_sample) {
            /* ef位0/1方向错，2/3非法跳变，4/5为左右反馈信号故障。 */
            bt_puts("gz:");    bt_putf1(dbg_gz_snapshot);
            bt_puts(" yaw:");  bt_putf1(dbg_yaw_snapshot);
            bt_puts(" ok:");   bt_putc((uint8_t)('0' + dbg_ok_snapshot));
            bt_puts(" turns:"); bt_putf1(dbg_turns_snapshot);
            bt_puts(" cc:"); bt_puti(dbg_corner_count_snapshot);
            bt_puts(" lc:"); bt_puti(dbg_l_target_snapshot);
            bt_putc('/');     bt_puti(dbg_l_measured_snapshot);
            bt_putc('/');     bt_puti(dbg_l_pwm_snapshot);
            bt_puts(" rc:"); bt_puti(dbg_r_target_snapshot);
            bt_putc('/');     bt_puti(dbg_r_measured_snapshot);
            bt_putc('/');     bt_puti(dbg_r_pwm_snapshot);
            bt_puts(" ef:"); bt_puti(dbg_encoder_fault_snapshot);
            bt_puts("\r\n");
        }
    }
}
