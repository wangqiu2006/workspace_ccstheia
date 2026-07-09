/**
 * @file servo.h
 * @brief 舵机驱动头文件 - TI MSPM0G3507
 *
 * 硬件参数:
 *   MCU  : MSPM0G3507 @ 32MHz
 *   Timer: TIMG12, 16分频 -> 2MHz 工作频率
 *   周期 : 40000 counts = 20ms (50Hz)
 *   信号引脚: PA26 (PWM_CH0)
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

/** 舵机PWM周期计数 (2MHz / 50Hz = 40000) */
#define SERVO_PERIOD_COUNTS     40000U

/** 脉冲宽度边界 (单位: counts @ 2MHz) */
#define SERVO_PULSE_MIN         2000U   /**< 0°   对应 1.0ms */
#define SERVO_PULSE_MID         3000U   /**< 90°  对应 1.5ms */
#define SERVO_PULSE_MAX         4000U   /**< 180° 对应 2.0ms */

/** 舵机角度范围 */
#define SERVO_ANGLE_MIN         0U
#define SERVO_ANGLE_MAX         180U

/* -----------------------------------------------------------------------
 *  函数声明
 * --------------------------------------------------------------------- */

/**
 * @brief 初始化舵机，将其置于中间位置(90°)
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * @param angle 目标角度，范围 [0, 180]，单位：度
 */
void Servo_SetAngle(uint8_t angle);

/**
 * @brief 设置舵机脉冲宽度（精确控制）
 * @param pulse_counts PWM计数值，范围 [SERVO_PULSE_MIN, SERVO_PULSE_MAX]
 */
void Servo_SetPulse(uint16_t pulse_counts);

/**
 * @brief 舵机平滑扫描 0° -> 180° -> 0°（阻塞式）
 * @param step_delay_ms 每步延迟(ms)，越小越快，建议 10~20
 */
void Servo_Sweep(uint32_t step_delay_ms);

/**
 * @brief 获取当前角度（软件记录值）
 * @return 当前角度 [0, 180]
 */
uint8_t Servo_GetAngle(void);

#endif /* SERVO_H_ */
