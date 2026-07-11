/**
 * @file empty.c
 * @brief 八路灰度巡线 — MSPM0G3507 + A4950 + MG513XP28_12V
 *        架构：定时器中断(100Hz) + 双轮独立增量式PI速度控制
 *
 * 引脚映射:
 *   左电机: 正转=CC2(PA15) 反转=CC0(PB14)  [TIMA0]
 *   右电机: 正转=CC3(PA12) 反转=CC1(PA7)   [TIMA0]
 *   继电器: PB16
 *   灰度: AD0=PA24 AD1=PA25 AD2=PA26 OUT=PA27
 *   左编码器: PA23(A) PA0(B)
 *   右编码器: PB24(A) PB25(B)
 *   串口: TX=PA10 RX=PA11 115200
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ================================================================
 *  参数配置（所有可调参数集中在这里）
 * ================================================================ */

/* ── 灰度传感器 ── */
#define ACTIVE_VALUE     1U     /* 检测到线时的电平：1=高有效 0=低有效（依模块批次实测）*/
#define SENSOR_FLIP      0U     /* 0=X1在左(正常) 1=X1在右(安装反向时置1)*/
#define BASE_PULSE_FAST  25U    /* 直线基础速度 (脉冲/10ms) */
#define BASE_PULSE_MIN   12U    /* 急弯时最低速度 */
#define CURVE_FACTOR      2U    /* 每单位误差降速量：base -= |err| * CURVE_FACTOR */
#define RAMP_STEP         4.0f  /* 目标速度每10ms最大变化量（防急加速失速）*/

/* ── 位置环（PD控制）── */
#define STEER_KP          7     /* 位置比例系数 */
#define STEER_KD          2     /* 位置微分系数（抑制过冲，越大越不敏感）*/

/* ── 速度环（增量式PI）── */
#define KP               2.5f   /* 速度PI比例 */
#define KI               0.25f  /* 速度PI积分（减小防止积分饱和）*/
#define MIN_PWM          800U   /* 最低有效PWM（低于此电机停转）*/
#define MOTOR_MAX        3200U

/* ================================================================
 *  编码器（按周期计数，每控制周期清零）
 * ================================================================ */
static volatile uint16_t enc_l_cnt = 0;
static volatile uint16_t enc_r_cnt = 0;

void GROUP1_IRQHandler(void)
{
    if (DL_GPIO_getPendingInterrupt(GRP_ENC_LEFT_PORT)) {
        DL_GPIO_clearInterruptStatus(GRP_ENC_LEFT_PORT,
            GRP_ENC_LEFT_ENC_L_A_PIN | GRP_ENC_LEFT_ENC_L_B_PIN);
        enc_l_cnt++;
    }
    if (DL_GPIO_getPendingInterrupt(GRP_ENC_RIGHT_PORT)) {
        DL_GPIO_clearInterruptStatus(GRP_ENC_RIGHT_PORT,
            GRP_ENC_RIGHT_ENC_R_A_PIN | GRP_ENC_RIGHT_ENC_R_B_PIN);
        enc_r_cnt++;
    }
}

/* ================================================================
 *  延时（仅初始化使用）
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
    char buf[160]; va_list a;
    va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    uart_puts(buf);
}

/* ================================================================
 *  电机驱动（TIMA0 4通道）
 * ================================================================ */
static void motor_set(int32_t left, int32_t right)
{
    if (left  >  (int32_t)MOTOR_MAX) left  =  (int32_t)MOTOR_MAX;
    if (left  < -(int32_t)MOTOR_MAX) left  = -(int32_t)MOTOR_MAX;
    if (right >  (int32_t)MOTOR_MAX) right =  (int32_t)MOTOR_MAX;
    if (right < -(int32_t)MOTOR_MAX) right = -(int32_t)MOTOR_MAX;

    if (left >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)left, DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,             DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,             DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-left), DL_TIMER_CC_0_INDEX);
    }
    if (right >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)right, DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,              DL_TIMER_CC_1_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,              DL_TIMER_CC_3_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)(-right), DL_TIMER_CC_1_INDEX);
    }
}

static void motor_stop(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U, DL_TIMER_CC_3_INDEX);
}

/* ================================================================
 *  灰度传感器
 *  MUX真值表 (74HC4051): ch=0→X1, ch=7→X8
 *  AD0=bit0, AD1=bit1, AD2=bit2
 *  EN引脚内置10K下拉，固定使能，无需控制
 * ================================================================ */
static void mux_select(uint8_t ch)
{
    /* 原子 SET/CLR，一次写完三位，避免RMW竞争 */
    uint32_t set_mask = 0, clr_mask = 0;
    (ch & 0x01) ? (set_mask |= GRP_GRAY_AD0_PIN) : (clr_mask |= GRP_GRAY_AD0_PIN);
    (ch & 0x02) ? (set_mask |= GRP_GRAY_AD1_PIN) : (clr_mask |= GRP_GRAY_AD1_PIN);
    (ch & 0x04) ? (set_mask |= GRP_GRAY_AD2_PIN) : (clr_mask |= GRP_GRAY_AD2_PIN);
    if (set_mask) DL_GPIO_setPins(GRP_GRAY_PORT, set_mask);
    if (clr_mask) DL_GPIO_clearPins(GRP_GRAY_PORT, clr_mask);
}

static void gray_scan_all(uint8_t data[8])
{
    for (uint8_t ch = 0; ch < 8; ch++) {
        uint8_t idx = SENSOR_FLIP ? (7U - ch) : ch;   /* 支持安装方向翻转 */
        mux_select(ch);
        delay_us(100);   /* 官方建议100µs（原50µs），信号更稳定 */
        uint8_t raw = DL_GPIO_readPins(GRP_GRAY_OUT_PORT, GRP_GRAY_OUT_OUT_PIN) ? 1U : 0U;
        data[idx] = (raw == ACTIVE_VALUE) ? 1U : 0U;  /* 极性适配 */
    }
}

/* 加权平均误差，返回 [-7,+7] 或 127(丢线) */
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

/* ================================================================
 *  速度环：增量式PI（每轮独立）
 *
 *  Δu = KP*(e[k]-e[k-1]) + KI*e[k]
 *  u[k] = u[k-1] + Δu
 *  防饱和：输出限幅 + MIN_PWM下限
 * ================================================================ */
typedef struct {
    float e_last;
    float pwm;
} PID_t;

static PID_t pid_l = {0, (float)MIN_PWM};
static PID_t pid_r = {0, (float)MIN_PWM};

static int32_t pid_update(PID_t *pid, float target, float actual)
{
    float e    = target - actual;
    float delta = KP * (e - pid->e_last) + KI * e;
    pid->e_last = e;
    pid->pwm   += delta;

    /* 反饱和：目标>0时不低于MIN_PWM */
    if (target > 0.0f && pid->pwm < (float)MIN_PWM) pid->pwm = (float)MIN_PWM;
    if (pid->pwm > (float)MOTOR_MAX) pid->pwm = (float)MOTOR_MAX;
    if (pid->pwm < 0.0f)             pid->pwm = 0.0f;
    return (int32_t)pid->pwm;
}

/* ================================================================
 *  100Hz 控制定时器中断
 * ================================================================ */
static uint8_t  sensor[8];
static uint16_t print_cnt = 0;

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_PID_INST)) {

    case DL_TIMER_IIDX_ZERO:
    {
        /* ── 1. 读编码器（当前周期脉冲数，读后清零）── */
        uint16_t spd_l, spd_r;
        __disable_irq();
        spd_l = enc_l_cnt;  enc_l_cnt = 0;
        spd_r = enc_r_cnt;  enc_r_cnt = 0;
        __enable_irq();

        /* ── 2. 读灰度传感器 ── */
        gray_scan_all(sensor);
        int32_t err = calc_error(sensor);

        /* ── 3. 丢线保护：连续5帧全零才停车 ── */
        static uint8_t lost_cnt = 0;
        if (err == 127) {
            if (++lost_cnt >= 5) {
                motor_stop();
                pid_l.pwm = (float)MIN_PWM;  pid_l.e_last = 0;
                pid_r.pwm = (float)MIN_PWM;  pid_r.e_last = 0;
            }
            break;   /* 丢线期间保持上一帧输出 */
        }
        lost_cnt = 0;

        /* ── 4. 位置环（PD）：计算转向量 ──
         *  P项：正比于偏差（比例纠偏）
         *  D项：正比于偏差变化率（阻尼，防过冲振荡）
         */
        static int32_t prev_err = 0;
        int32_t d_err  = err - prev_err;
        prev_err       = err;
        int32_t steer  = err * STEER_KP + d_err * STEER_KD;

        /* ── 5. 弯道自动降速 ──
         *  直线(|err|=0): base = BASE_PULSE_FAST
         *  弯道(|err|=7): base = BASE_PULSE_FAST - 7*2 = 最低到 BASE_PULSE_MIN
         */
        int32_t abs_err = err < 0 ? -err : err;
        int32_t base_i  = (int32_t)BASE_PULSE_FAST - abs_err * (int32_t)CURVE_FACTOR;
        if (base_i < (int32_t)BASE_PULSE_MIN) base_i = (int32_t)BASE_PULSE_MIN;
        float base = (float)base_i;

        /* ── 6. 单侧加速目标速度 ──
         *  慢轮 = base（不降速，防止死区停转）
         *  快轮 = base + |steer|
         */
        float target_l = base;
        float target_r = base;
        if (steer > 0) target_l += (float)steer;
        else           target_r -= (float)steer;

        /* ── 7. 目标斜坡限制（防急加速）── */
        static float ramp_l = (float)BASE_PULSE_FAST;
        static float ramp_r = (float)BASE_PULSE_FAST;
        if      (target_l > ramp_l + RAMP_STEP) ramp_l += RAMP_STEP;
        else if (target_l < ramp_l - RAMP_STEP) ramp_l -= RAMP_STEP;
        else                                     ramp_l  = target_l;
        if      (target_r > ramp_r + RAMP_STEP) ramp_r += RAMP_STEP;
        else if (target_r < ramp_r - RAMP_STEP) ramp_r -= RAMP_STEP;
        else                                     ramp_r  = target_r;

        /* ── 8. 速度环PI → 输出PWM ── */
        int32_t pwm_l = pid_update(&pid_l, ramp_l, (float)spd_l);
        int32_t pwm_r = pid_update(&pid_r, ramp_r, (float)spd_r);
        motor_set(pwm_l, pwm_r);

        /* ── 9. 调试串口（每500ms一次）── */
        if (++print_cnt >= 50) {
            print_cnt = 0;
            int32_t eL = (int32_t)ramp_l - (int32_t)spd_l;
            int32_t eR = (int32_t)ramp_r - (int32_t)spd_r;
            uart_printf("S:%d%d%d%d%d%d%d%d e=%d bs=%d tL=%d tR=%d "
                        "sL=%d sR=%d eL=%d eR=%d pwL=%d pwR=%d\r\n",
                        sensor[0],sensor[1],sensor[2],sensor[3],
                        sensor[4],sensor[5],sensor[6],sensor[7],
                        (int)err,   (int)base_i,
                        (int)ramp_l,(int)ramp_r,
                        (int)spd_l, (int)spd_r,
                        (int)eL,    (int)eR,
                        (int)pwm_l, (int)pwm_r);
        }
        break;
    }
    default: break;
    }
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    delay_ms(200);

    NVIC_EnableIRQ(GRP_ENC_LEFT_INT_IRQN);
    NVIC_EnableIRQ(GRP_ENC_RIGHT_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    uart_puts("\r\n=== 八路灰度巡线 MSPM0G3507 ===\r\n");
    uart_printf("FAST=%u MIN=%u CURVE=%u KP=%.1f KI=%.2f\r\n",
                BASE_PULSE_FAST, BASE_PULSE_MIN, CURVE_FACTOR,
                (double)KP, (double)KI);

    DL_TimerA_startCounter(PWM_DRIVE_INST);
    DL_GPIO_setPins(GRP_RELAY_PORT, GRP_RELAY_MOTOR_PIN);
    DL_TimerG_startCounter(TIMER_PID_INST);

    delay_ms(500);
    uart_puts("Running.\r\n");

    while (1) { __WFI(); }
}
