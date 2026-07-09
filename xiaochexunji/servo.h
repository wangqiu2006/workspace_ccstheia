/**
 * @file servo.h
 * @brief 舵机驱动 - 整合到 xiaochexunji 项目
 *
 * 硬件参数:
 *   MCU  : MSPM0G3507 @ 32MHz
 *   Timer: TIMG12 (PWM_SERVO), 16分频 -> 2MHz
 *   周期 : 40000 counts = 20ms (50Hz)
 *   信号引脚: PA0 (PWM_SERVO CCP0)
 *
 * 注意: PA26 已被 74HC4051 MUX_C 占用，PA16 不是 TIMG12 有效引脚。
 *
 * 脉冲宽度映射:
 *   0°   -> 2000 counts (1.0ms)
 *   90°  -> 3000 counts (1.5ms)
 *   180° -> 4000 counts (2.0ms)
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 *  宏定义
 * --------------------------------------------------------------------- */

/** 舵机PWM计数器周期 (2MHz / 50Hz = 40000, Load = 39999) */
#define SERVO_PERIOD_COUNTS     40000U

/** 脉冲宽度边界 (单位: counts @ 4MHz) */
#define SERVO_PULSE_MIN         4000U   /**< 0°   对应 1.0ms */
#define SERVO_PULSE_MID         6000U   /**< 90°  对应 1.5ms */
#define SERVO_PULSE_MAX         8000U   /**< 180° 对应 2.0ms */

/** 舵机角度范围 */
#define SERVO_ANGLE_MIN         0U
#define SERVO_ANGLE_MAX         180U

/* -----------------------------------------------------------------------
 *  函数声明
 * --------------------------------------------------------------------- */

/**
 * @brief 初始化舵机，置于中间位置 (90°)
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * @param angle 目标角度 [0, 180]，单位：度
 */
void Servo_SetAngle(uint8_t angle);

/**
 * @brief 设置舵机脉冲宽度（精确控制）
 * @param pulse_counts PWM计数值 [SERVO_PULSE_MIN, SERVO_PULSE_MAX]
 */
void Servo_SetPulse(uint16_t pulse_counts);

/**
 * @brief 舵机全范围扫描 0° -> 180° -> 0°（阻塞式）
 * @param step_delay_ms 每步延迟(ms)，建议 10~20
 */
void Servo_Sweep(uint32_t step_delay_ms);

/**
 * @brief 获取当前角度（软件记录值）
 * @return 当前角度 [0, 180]
 */
uint8_t Servo_GetAngle(void);

#endif /* SERVO_H_ */
