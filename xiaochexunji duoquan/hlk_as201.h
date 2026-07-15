/**
 * @file    hlk_as201.h
 * @brief   HLK-AS201 系列姿态传感器驱动 (TI MSPM0G3507 / CCS)
 * @date    2026-07-15
 *
 * 严格依据《HLK-AS201系列姿态传感器模块说明书 V1.1》第 11 章通信协议实现。
 *
 * 硬件连接:
 *   AS201 UART_TX(引脚15) → MSPM0 UART RX (如 PA11)
 *   AS201 UART_RX(引脚16) → MSPM0 UART TX (如 PA10)
 *   AS201 VCC33           → 3.3V
 *   AS201 GND             → GND
 *
 * 串口参数: 默认 115200, 8 数据位, 1 停止位, 无校验 (8-N-1)
 *
 * 协议帧格式 (小端, 十六进制):
 *   head(2) | len(1) | cmd(1) | data(N) | check(1) | tail(2)
 *   0xFA 0xFB          .................          0xFC 0xFD
 *   - len   : cmd 字段到 check 的总字节数 = 1(cmd) + N(data) + 1(check)
 *   - check : (cmd + data 各字节) 的累加和取低 8 位  ← 不含 head/tail/len
 */

#ifndef __HLK_AS201_H__
#define __HLK_AS201_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 协议常量
 *===========================================================================*/
#define AS201_HEAD0                 0xFA
#define AS201_HEAD1                 0xFB
#define AS201_TAIL0                 0xFC
#define AS201_TAIL1                 0xFD

/* 命令集 (说明书 11.4) */
#define AS201_CMD_DATA_REPORT       0x00    /* 姿态数据上报 (模块→主机)      */
#define AS201_CMD_GET_VERSION       0x10    /* 获取版本号                    */
#define AS201_CMD_RESTORE_FACTORY   0x11    /* 恢复出厂 (自动重启)           */
#define AS201_CMD_ZERO_YAW          0x12    /* Z轴角度归零 (仅六轴)          */
#define AS201_CMD_ANGLE_REF         0x13    /* 角度参考: XYZ 欧拉角置零(六轴)*/
#define AS201_CMD_GET_DIRECTION     0x14    /* 获取安装方向 (仅六轴)         */
#define AS201_CMD_SET_DIRECTION     0x15    /* 设置安装方向 (仅六轴)         */
#define AS201_CMD_SET_RATE          0x16    /* 设置回传速率                  */
#define AS201_CMD_REBOOT            0x17    /* 重启                          */
#define AS201_CMD_SET_BAUD          0x18    /* 设置串口波特率 (重启生效)     */
#define AS201_CMD_GET_CONFIG        0x19    /* 获取配置                      */
#define AS201_CMD_REPORT_SWITCH     0x1A    /* 数据上报开关                  */
#define AS201_CMD_GET_ONCE          0x1B    /* 获取数据 1 次 (需关闭上报)    */
#define AS201_CMD_CALIB_ACC_GYRO    0x1C    /* 加速度/角速度零偏校准         */
#define AS201_CMD_CALIB_MAG_START   0x1D    /* 磁场校准开始 (九/十轴)        */
#define AS201_CMD_CALIB_MAG_DONE    0x1E    /* 磁场校准完成 (九/十轴)        */
#define AS201_CMD_SET_SUBSCRIBE     0x1F    /* 设置上报数据订阅标识          */

/* 回传速率码 (cmd 0x16) */
#define AS201_RATE_0P1HZ            0x01
#define AS201_RATE_0P5HZ            0x02
#define AS201_RATE_1HZ              0x03
#define AS201_RATE_2HZ              0x04
#define AS201_RATE_5HZ              0x05
#define AS201_RATE_10HZ             0x06
#define AS201_RATE_20HZ             0x07    /* 默认 */

/* 波特率码 (cmd 0x18) */
#define AS201_BAUD_4800             0x01
#define AS201_BAUD_9600             0x02
#define AS201_BAUD_19200            0x03
#define AS201_BAUD_38400            0x04
#define AS201_BAUD_57600            0x05
#define AS201_BAUD_115200           0x06    /* 默认 */
#define AS201_BAUD_230400           0x07
#define AS201_BAUD_460800           0x08
#define AS201_BAUD_500000           0x09
#define AS201_BAUD_921600           0x0A

/* 订阅标识位 (cmd 0x1F, 也是 data 域字段出现顺序的掩码) */
#define AS201_SUB_ACCEL             (1 << 0)    /* 加速度 6 字节 */
#define AS201_SUB_GYRO              (1 << 1)    /* 角速度 6 字节 */
#define AS201_SUB_ANGLE             (1 << 2)    /* 角度   6 字节 */
#define AS201_SUB_MAG               (1 << 3)    /* 磁场   6 字节 */
#define AS201_SUB_QUAT              (1 << 4)    /* 四元数 8 字节 */
#define AS201_SUB_TEMP              (1 << 5)    /* 温度   2 字节 */
#define AS201_SUB_PRESSURE          (1 << 6)    /* 气压   4 字节 */
#define AS201_SUB_HEIGHT            (1 << 7)    /* 高度   4 字节 */
#define AS201_SUB_ALL               0xFF

/* sensor_type (数据上报 data[0] 的 bit0~1) */
#define AS201_SENSOR_10AXIS         0x00
#define AS201_SENSOR_9AXIS          0x01
#define AS201_SENSOR_6AXIS          0x02

/* 缓冲区 */
#define AS201_RX_BUF_SIZE           256     /* 环形接收缓冲区 */
#define AS201_FRAME_MAX             64      /* 单帧 cmd..check 最大字节 (数据帧 len=45) */

/*===========================================================================
 * 转换系数 (说明书第 8~9 页, 原始值为小端 int16, 乘以系数得真实值)
 *===========================================================================*/
#define AS201_K_ACCEL               0.00478515625f      /* → m/s²   */
#define AS201_K_GYRO                0.0625f             /* → °/s    */
#define AS201_K_ANGLE               0.0054931640625f    /* → °      */
#define AS201_K_MAG                 0.006103515625f     /* → µT     */
#define AS201_K_QUAT                0.000030517578125f  /* 无量纲    */
#define AS201_K_TEMP                0.01f               /* → ℃      */
#define AS201_K_PRESSURE            0.0002384185791f    /* → Pa (int32) */
#define AS201_K_HEIGHT              0.0010728836f       /* → m  (int32) */

/*===========================================================================
 * 数据类型
 *===========================================================================*/
typedef struct { float x, y, z; } AS201_Vec3;
typedef struct { float q0, q1, q2, q3; } AS201_Quat;   /* q0=W q1=X q2=Y q3=Z */
typedef struct { float roll, pitch, yaw; } AS201_Euler;

typedef struct {
    AS201_Vec3   accel;         /* m/s²  */
    AS201_Vec3   gyro;          /* °/s   */
    AS201_Euler  angle;         /* °     */
    AS201_Vec3   mag;           /* µT    */
    AS201_Quat   quat;
    float        temperature;   /* ℃     */
    float        pressure;      /* Pa    */
    float        height;        /* m     */

    uint8_t      sensor_type;   /* 0=10轴 1=9轴 2=6轴 */
    uint8_t      mag_accuracy;  /* 0=未校准/强干扰 3=已校准 */
    uint8_t      update_flags;  /* 见 AS201_SUB_* */
} AS201_Data;

typedef enum {
    AS201_OK           =  0,
    AS201_ERR_TIMEOUT  = -1,
    AS201_ERR_CHECKSUM = -2,
    AS201_ERR_FRAME    = -3,
    AS201_ERR_PARAM    = -4,
} AS201_Status;

typedef struct {
    /* 硬件绑定 */
    UART_Regs        *uart;
    IRQn_Type         irq;

    /* 环形接收缓冲区 (ISR 写, Poll 读) */
    uint8_t           rx_buf[AS201_RX_BUF_SIZE];
    volatile uint16_t rx_head;              /* ISR 写, Poll 读 */
    volatile uint16_t rx_tail;              /* Poll 写, ISR 读(判满); 须 volatile */

    /* 帧解析状态机 */
    uint8_t           state;
    uint8_t           frame[AS201_FRAME_MAX];   /* 存 cmd..check */
    uint8_t           idx;                      /* 已收字节 */
    uint8_t           need;                     /* 本帧 cmd..check+tail 剩余字节数 */

    /* 订阅掩码: 决定数据上报帧中各字段是否存在及顺序. 默认全订阅 */
    uint8_t           subscription;

    AS201_Data        data;
    volatile uint32_t frame_count;              /* 收到有效数据帧计数         */
    volatile uint32_t overflow_count;           /* 环形缓冲满而丢字节次数(ISR)*/
    uint32_t          len_mismatch_count;       /* 数据帧长度与订阅掩码不符次数*/
    uint32_t          checksum_err_count;       /* 帧校验和错误次数           */

    /* 0x19 获取配置 回包缓存 (订阅标识掉电保存, 上电须回读以免长度自检误判) */
    uint8_t           cfg_subscription;         /* 模块实际订阅标识           */
    uint8_t           cfg_rate;                 /* 模块实际回传速率码         */
    uint8_t           cfg_baud;                 /* 模块实际波特率码           */
    uint8_t           cfg_report_on;            /* 模块实际上报开关 0/1       */
    volatile bool     cfg_valid;                /* 已收到至少一次配置回包     */
} AS201_Handle;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief 初始化驱动 (UART 硬件请先用 SysConfig 生成 DL_UART_Main_init 完成)
 * @param h    句柄
 * @param uart UART 基址 (如 UART0)
 * @param irq  UART 中断号 (如 UART0_INT_IRQn)
 * @note  本函数只清状态、设默认订阅、使能 RX 中断与 NVIC, 不重配 UART 参数
 */
AS201_Status AS201_Init(AS201_Handle *h, UART_Regs *uart, IRQn_Type irq);

/** @brief 主循环轮询解析. 20Hz 全字段上报下, 调用间隔建议 ≤10ms */
void AS201_Poll(AS201_Handle *h);

/** @brief 取最新数据 (只读) */
const AS201_Data *AS201_GetData(const AS201_Handle *h);

/** @brief 自 last_count 以来是否有新帧; 用 *last_count 记录基准 */
bool AS201_HasNewFrame(const AS201_Handle *h, uint32_t *last_count);

/** @brief UART RX 中断入口, 在 UARTx_IRQHandler 中调用 */
void AS201_ISR(AS201_Handle *h);

/*---- 配置命令 (阻塞发送) ----*/

/** @brief 设置订阅标识 (AS201_SUB_* 组合). 同步更新本地解析掩码 */
AS201_Status AS201_SetSubscription(AS201_Handle *h, uint8_t flags);

/** @brief 设置回传速率, rate 用 AS201_RATE_* */
AS201_Status AS201_SetRate(AS201_Handle *h, uint8_t rate_code);

/** @brief 数据上报开关. on=true 开, false 关 */
AS201_Status AS201_SetReportEnable(AS201_Handle *h, bool on);

/** @brief 加速度/角速度零偏校准 (模块须水平静止) */
AS201_Status AS201_CalibAccGyro(AS201_Handle *h);

/** @brief 磁场校准开始 (九/十轴, 随后绕 XYZ 各转一圈以上) */
AS201_Status AS201_CalibMagStart(AS201_Handle *h);

/** @brief 磁场校准完成 (九/十轴, offset 存 flash) */
AS201_Status AS201_CalibMagDone(AS201_Handle *h);

/** @brief 恢复出厂设置 (自动重启) */
AS201_Status AS201_RestoreFactory(AS201_Handle *h);

/** @brief 重启模块 */
AS201_Status AS201_Reboot(AS201_Handle *h);

/** @brief 请求版本号 (异步, 回包在数据流中以 cmd=0x10 返回) */
AS201_Status AS201_RequestVersion(AS201_Handle *h);

/**
 * @brief 请求模块配置 (异步). 回包 cmd=0x19 由 AS201_Poll 解析后
 *        自动同步 h->subscription 与 h->cfg_* 字段, 并置 cfg_valid=true
 * @note  订阅标识掉电保存: 若模块曾被改为非全订阅, 本地默认 AS201_SUB_ALL
 *        会导致数据帧长度自检恒不符而丢弃全部帧. 上电后应回读一次配置。
 */
AS201_Status AS201_RequestConfig(AS201_Handle *h);

/**
 * @brief 阻塞式同步配置: 发送 0x19 并轮询等待回包更新本地订阅掩码
 * @param timeout_ms  最长等待毫秒
 * @param tick_ms     调用方提供的当前毫秒计数 (取地址, 用于超时判断)
 * @return AS201_OK 已同步; AS201_ERR_TIMEOUT 超时未收到配置回包
 * @note  内部循环调用 AS201_Poll; 需 RX 中断已使能。典型放在 Init 之后调用一次。
 */
AS201_Status AS201_SyncConfig(AS201_Handle *h,
                              uint32_t timeout_ms,
                              volatile uint32_t *tick_ms);

/** @brief 通用命令发送: 组帧 head/len/cmd/data/check/tail 并阻塞发出 */
AS201_Status AS201_SendCommand(AS201_Handle *h, uint8_t cmd,
                               const uint8_t *data, uint8_t dlen);

#ifdef __cplusplus
}
#endif
#endif /* __HLK_AS201_H__ */
