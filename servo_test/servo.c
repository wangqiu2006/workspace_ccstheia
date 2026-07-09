/**
 * @file servo.c
 * @brief 舵机驱动实现 - TI MSPM0G3507
 */

#include "servo.h"

/* -----------------------------------------------------------------------
 *  内部状态
 * --------------------------------------------------------------------- */

/** 当前角度软件记录值 */
static uint8_t s_current_angle = 90U;

/* -----------------------------------------------------------------------
 *  内部辅助函数
 * --------------------------------------------------------------------- */

/**
 * @brief 简单阻塞延时（基于 NOP 循环）
 * @note  32MHz下约 4000次NOP ≈ 1ms（与 xiaochexunji 项目一致）
 */
static void servo_delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000U; i++) {
        __asm("nop");
    }
}

/**
 * @brief 将角度转换为 PWM 计数值
 * @param angle 角度 [0, 180]
 * @return PWM 计数值 [SERVO_PULSE_MIN, SERVO_PULSE_MAX]
 */
static uint16_t servo_angle_to_counts(uint8_t angle)
{
    /* 限幅保护 */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    /*
     * 线性映射:
     *   counts = PULSE_MIN + angle * (PULSE_MAX - PULSE_MIN) / 180
     *          = 2000 + angle * 2000 / 180
     *          = 2000 + angle * 11.11...
     *
     * 使用整数运算：乘以2000再除以180，避免浮点
     */
    uint32_t pulse = (uint32_t)SERVO_PULSE_MIN
                   + ((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180U;

    return (uint16_t)pulse;
}

/* -----------------------------------------------------------------------
 *  公共函数实现
 * --------------------------------------------------------------------- */

void Servo_Init(void)
{
    /* SysConfig 已完成 Timer / PWM 的底层初始化，此处只设定初始位置 */
    s_current_angle = 90U;
    Servo_SetPulse(SERVO_PULSE_MID);

    /* 等待舵机到达中间位置 */
    servo_delay_ms(500U);
}

void Servo_SetAngle(uint8_t angle)
{
    /* 限幅 */
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    s_current_angle = angle;
    Servo_SetPulse(servo_angle_to_counts(angle));
}

void Servo_SetPulse(uint16_t pulse_counts)
{
    /* 限幅保护，防止舵机过冲 */
    if (pulse_counts < SERVO_PULSE_MIN) {
        pulse_counts = SERVO_PULSE_MIN;
    }
    if (pulse_counts > SERVO_PULSE_MAX) {
        pulse_counts = SERVO_PULSE_MAX;
    }

    /*
     * MSPM0G3507 DriverLib: 设置 PWM 比较值
     * DL_TimerG_setCaptureCompareValue(TIMG12, pulse_counts, DL_TIMER_CC_0_INDEX)
     *
     * 注意: SysConfig 生成的实例名可能不同，
     *       请根据 ti_msp_dl_config.h 中实际宏名修改 PWM_SERVO_INST
     */
    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST,
        (uint32_t)pulse_counts,
        DL_TIMER_CC_0_INDEX
    );
}

uint8_t Servo_GetAngle(void)
{
    return s_current_angle;
}

void Servo_Sweep(uint32_t step_delay_ms)
{
    uint8_t angle;

    /* 正向扫描: 0° -> 180° */
    for (angle = SERVO_ANGLE_MIN; angle <= SERVO_ANGLE_MAX; angle++) {
        Servo_SetAngle(angle);
        servo_delay_ms(step_delay_ms);
    }

    /* 反向扫描: 180° -> 0° */
    for (angle = SERVO_ANGLE_MAX; angle > SERVO_ANGLE_MIN; angle--) {
        Servo_SetAngle(angle);
        servo_delay_ms(step_delay_ms);
    }
    /* 确保回到0° */
    Servo_SetAngle(SERVO_ANGLE_MIN);
    servo_delay_ms(step_delay_ms);
}
