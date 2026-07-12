/**
 * @file empty.c
 * @brief 八路灰度巡线 — MSPM0G3507 + A4950 + MG513XP28_12V
 *        架构参考 11_PID_car：adjust_motor→calculate_speed→DC_MOTOR_PID
 *
 * 引脚:
 *   左电机: 正转=CC2(PA15) 反转=CC0(PB14)  [TIMA0]
 *   右电机: 正转=CC3(PA12) 反转=CC1(PA7)   [TIMA0]
 *   继电器: PB16   灰度: AD0=PA24 AD1=PA25 AD2=PA26 OUT=PA27
 *   左编码器: PA23/PA0  右编码器: PB24/PB25
 *   串口: PA10(TX) PA11(RX) 115200
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/* ================================================================
 *  硬件规格（根据实物修改）
 * ================================================================ */
#define MOTOR_ENC_PPR        20U    /* 编码器每圈脉冲数（电机轴）*/
#define MOTOR_GEAR_RATIO     28U    /* 减速比 28:1 */
#define MOTOR_WHEEL_D_MM     65U    /* 车轮直径 mm */
/* 每圈车轮对应的编码器中断次数 = PPR×4(AB双沿)×减速比 */
#define PULSES_PER_WHEEL_REV (MOTOR_ENC_PPR * 4U * MOTOR_GEAR_RATIO)  /* 2240 */

/* ================================================================
 *  控制参数（参考 11_PID_car 调参风格，单位 mm/s）
 * ================================================================ */
/* ── 位置环 ── */
#define BASE_SPEED_MM       200.0f  /* 直线基础速度 mm/s */
#define BASE_SPEED_MIN_MM    90.0f  /* 急弯最低速度 mm/s */
#define CURVE_FACTOR_MM      15.0f  /* 每单位误差降速 mm/s */
#define STEER_GAIN_MM        35.0f  /* 最大额外转向速度 mm/s（err=7 时）*/
#define STEER_KD              2     /* 位置微分（抑制过冲）*/

/* ── 速度环（位置式PI）── */
#define KP                   6.0f   /* 比例（PWM/mm/s）*/
#define KI                   1.5f   /* 积分 */
#define INTEGRAL_MAX         1200.0f/* 积分限幅 */
#define MIN_PWM               800U  /* 最低有效PWM */
#define MOTOR_MAX             3200U

/* ── 灰度传感器 ── */
#define SENSOR_ACTIVE_VAL     1U    /* 检测到线=1；若反请改0 */
#define SENSOR_FLIP           0U    /* 安装方向正常=0，反装=1 */

/* ================================================================
 *  编码器（按周期计数）
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
 *  电机PWM（底层驱动，不含速度控制）
 * ================================================================ */
static void motor_set_pwm(int32_t left, int32_t right)
{
    if (left  >  (int32_t)MOTOR_MAX) left  =  (int32_t)MOTOR_MAX;
    if (left  < -(int32_t)MOTOR_MAX) left  = -(int32_t)MOTOR_MAX;
    if (right >  (int32_t)MOTOR_MAX) right =  (int32_t)MOTOR_MAX;
    if (right < -(int32_t)MOTOR_MAX) right = -(int32_t)MOTOR_MAX;

    if (left >= 0) {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, (uint32_t)left,  DL_TIMER_CC_2_INDEX);
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,              DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_DRIVE_INST, 0U,              DL_TIMER_CC_2_INDEX);
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
 *  速度计算（参考 11_PID_car: calculate_speed）
 *  输入: 编码器中断次数/10ms
 *  输出: mm/s
 * ================================================================ */
static float calculate_speed(uint16_t counts)
{
    /* v = counts/PULSES_PER_WHEEL_REV * 100 * π * WHEEL_D */
    return (float)counts * 100.0f / (float)PULSES_PER_WHEEL_REV
           * 3.14159265f * (float)MOTOR_WHEEL_D_MM;
}

/* ================================================================
 *  灰度传感器（MUX扫描，参考官方驱动）
 * ================================================================ */
static uint8_t sensor[8];

static void mux_select(uint8_t ch)
{
    uint32_t sm = 0, cm = 0;
    (ch & 0x01) ? (sm |= GRP_GRAY_AD0_PIN) : (cm |= GRP_GRAY_AD0_PIN);
    (ch & 0x02) ? (sm |= GRP_GRAY_AD1_PIN) : (cm |= GRP_GRAY_AD1_PIN);
    (ch & 0x04) ? (sm |= GRP_GRAY_AD2_PIN) : (cm |= GRP_GRAY_AD2_PIN);
    if (sm) DL_GPIO_setPins(GRP_GRAY_PORT, sm);
    if (cm) DL_GPIO_clearPins(GRP_GRAY_PORT, cm);
}

static void gray_scan_all(void)
{
    for (uint8_t ch = 0; ch < 8; ch++) {
        uint8_t idx = SENSOR_FLIP ? (7U - ch) : ch;
        mux_select(ch);
        delay_us(100);   /* 官方建议100µs */
        uint8_t raw = DL_GPIO_readPins(GRP_GRAY_OUT_PORT, GRP_GRAY_OUT_OUT_PIN) ? 1U : 0U;
        sensor[idx] = (raw == SENSOR_ACTIVE_VAL) ? 1U : 0U;
    }
}

/* 加权平均位置误差 [-7,+7]，127=丢线 */
static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static int32_t calc_position_error(void)
{
    int32_t sum = 0, cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor[i]) { sum += weight[i]; cnt++; }
    }
    if (cnt == 0) return 127;
    return sum / cnt;
}

/* ================================================================
 *  位置控制器（参考 11_PID_car: adjust_motor）
 *  输入: 位置误差 err
 *  输出: 左右轮目标速度 mm/s（写入 *tL, *tR）
 * ================================================================ */
static void position_control(int32_t err, float *tL, float *tR)
{
    static int32_t prev_err = 0;

    /* 弯道自动降速 */
    int32_t abs_err = err < 0 ? -err : err;
    float base = BASE_SPEED_MM - (float)abs_err * CURVE_FACTOR_MM;
    if (base < BASE_SPEED_MIN_MM) base = BASE_SPEED_MIN_MM;

    /* PD 位置误差 → 转向量 */
    int32_t d_err = err - prev_err;
    prev_err = err;
    float steer = ((float)err * 1.0f + (float)d_err * (float)STEER_KD)
                  * STEER_GAIN_MM / 7.0f;

    /* 单侧加速：慢轮保持 base，快轮叠加转向量 */
    if (steer > 0.0f) {          /* 线偏右 → 左轮加速 */
        *tL = base + steer;
        *tR = base;
    } else if (steer < 0.0f) {   /* 线偏左 → 右轮加速 */
        *tL = base;
        *tR = base - steer;
    } else {
        *tL = base;
        *tR = base;
    }
    if (*tL > BASE_SPEED_MM + STEER_GAIN_MM) *tL = BASE_SPEED_MM + STEER_GAIN_MM;
    if (*tR > BASE_SPEED_MM + STEER_GAIN_MM) *tR = BASE_SPEED_MM + STEER_GAIN_MM;
}

/* ================================================================
 *  速度PI控制器（参考 11_PID_car: DC_MOTOR_PID — 位置式PI）
 *  输入: target/actual（mm/s）
 *  输出: PWM [MIN_PWM, MOTOR_MAX]
 * ================================================================ */
typedef struct {
    float integral;
} SpeedPI_t;

static SpeedPI_t pi_l = {0};
static SpeedPI_t pi_r = {0};

static int32_t speed_pi(SpeedPI_t *pi, float target, float actual)
{
    float error = target - actual;

    /* 积分累积 + 限幅（防积分饱和）*/
    pi->integral += KI * error;
    if (pi->integral >  INTEGRAL_MAX) pi->integral =  INTEGRAL_MAX;
    if (pi->integral < -INTEGRAL_MAX) pi->integral = -INTEGRAL_MAX;

    float output = KP * error + pi->integral;

    /* 底部限幅：目标>0 时保持最低PWM，防电机死区 */
    if (target > 0.0f && output < (float)MIN_PWM) output = (float)MIN_PWM;
    if (output > (float)MOTOR_MAX) output = (float)MOTOR_MAX;
    if (output < 0.0f)             output = 0.0f;
    return (int32_t)output;
}

/* ================================================================
 *  100Hz 定时器控制中断
 *  流程（同参考代码）:
 *    gray_scan → position_control → calculate_speed → speed_pi → motor_set
 * ================================================================ */
static uint16_t print_cnt = 0;

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_PID_INST)) {

    case DL_TIMER_IIDX_ZERO:
    {
        /* Step 1: 读编码器 → 计算实际速度 mm/s */
        uint16_t cl, cr;
        __disable_irq();
        cl = enc_l_cnt;  enc_l_cnt = 0;
        cr = enc_r_cnt;  enc_r_cnt = 0;
        __enable_irq();
        float spd_l = calculate_speed(cl);
        float spd_r = calculate_speed(cr);

        /* Step 2: 读灰度传感器 */
        gray_scan_all();
        int32_t err = calc_position_error();

        /* Step 3: 丢线保护（连续5帧才停车）*/
        static uint8_t lost_cnt = 0;
        if (err == 127) {
            if (++lost_cnt >= 5) {
                motor_stop();
                pi_l.integral = 0;
                pi_r.integral = 0;
            }
            break;
        }
        lost_cnt = 0;

        /* Step 4: 位置控制 → 目标速度 mm/s */
        float tL = 0.0f, tR = 0.0f;
        position_control(err, &tL, &tR);

        /* Step 5: 速度PI → 输出PWM */
        int32_t pwm_l = speed_pi(&pi_l, tL, spd_l);
        int32_t pwm_r = speed_pi(&pi_r, tR, spd_r);
        motor_set_pwm(pwm_l, pwm_r);

        /* Step 6: 调试输出（每500ms）*/
        if (++print_cnt >= 50) {
            print_cnt = 0;
            uart_printf("S:%d%d%d%d%d%d%d%d e=%d "
                        "tL=%d tR=%d sL=%d sR=%d "
                        "eL=%d eR=%d pwL=%d pwR=%d\r\n",
                        sensor[0],sensor[1],sensor[2],sensor[3],
                        sensor[4],sensor[5],sensor[6],sensor[7],
                        (int)err,
                        (int)tL,   (int)tR,
                        (int)spd_l,(int)spd_r,
                        (int)(tL - spd_l), (int)(tR - spd_r),
                        (int)pwm_l,(int)pwm_r);
        }
        break;
    }
    default: break;
    }
}

/* ================================================================
 *  主函数（仅初始化，控制全在中断）
 * ================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    delay_ms(200);

    NVIC_EnableIRQ(GRP_ENC_LEFT_INT_IRQN);
    NVIC_EnableIRQ(GRP_ENC_RIGHT_INT_IRQN);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    uart_puts("\r\n=== 八路灰度巡线（参考11_PID_car架构）===\r\n");
    uart_printf("BASE=%.0f MIN=%.0f KP=%.1f KI=%.2f\r\n",
                (double)BASE_SPEED_MM, (double)BASE_SPEED_MIN_MM,
                (double)KP, (double)KI);
    uart_printf("PULSES/REV=%u WHEEL=%umm\r\n",
                PULSES_PER_WHEEL_REV, MOTOR_WHEEL_D_MM);

    DL_TimerA_startCounter(PWM_DRIVE_INST);
    DL_GPIO_setPins(GRP_RELAY_PORT, GRP_RELAY_MOTOR_PIN);
    DL_TimerG_startCounter(TIMER_PID_INST);

    delay_ms(500);
    uart_puts("Running.\r\n");

    while (1) { __WFI(); }
}
