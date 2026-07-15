/**
 * test_vofa.c — VOFA+ 有线测试代码
 *
 * 使用方式:
 *   1. Type-C 接电脑, LaunchPad 跳线帽 J101 RXD/TXD 保持 ON
 *   2. 编译烧录后打开 VOFA+, 新建串口连接
 *   3. 协议选 JustFloat, 波特率 115200
 *   4. 添加波形图控件, 绑定通道 0~3 即可看到数据
 *
 * 数据通道:
 *   通道0: ADC 灰度值 (PA27, 0~4095)
 *   通道1: 正弦波测试信号
 *   通道2: 锯齿波测试信号
 *   通道3: 方波测试信号
 */

#include "ti_msp_dl_config.h"
#include <string.h>
#include <math.h>

/* ===================== JustFloat 协议 ===================== */

static const uint8_t JUSTFLOAT_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/**
 * 发送 JustFloat 数据帧到 VOFA+
 * @param data       float 数组指针
 * @param ch_count   通道数量
 */
void vofa_send(float *data, uint8_t ch_count)
{
    uint8_t buf[128];  // 最多 31 通道
    uint32_t len = ch_count * 4;

    memcpy(buf, data, len);
    memcpy(buf + len, JUSTFLOAT_TAIL, 4);

    for (uint32_t i = 0; i < len + 4; i++) {
        DL_UART_transmitDataBlocking(UART_JDY31_INST, buf[i]);
    }
}

/* ===================== 简单延时 ===================== */

void delay_ms(uint32_t ms)
{
    // 32MHz CPUCLK, 粗略延时
    for (volatile uint32_t i = 0; i < ms * 4000; i++) {
        __asm("nop");
    }
}

/* ===================== ADC 读取 ===================== */

/**
 * 读取 ADC12_0 转换结果 (PA27, Channel 0)
 * 返回 0~4095 原始值
 */
uint16_t adc_read(void)
{
    DL_ADC12_startConversion(ADC12_0_INST);

    while (DL_ADC12_getStatus(ADC12_0_INST) &
           DL_ADC12_STATUS_CONVERSION_ACTIVE) {
        /* 等待转换完成 */
    }

    return DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);
}

/* ===================== 主函数 ===================== */

int main(void)
{
    SYSCFG_DL_init();
    delay_ms(100);  // 上电稳定

    float   data[4];   // VOFA+ 4 通道
    uint32_t t = 0;

    while (1) {
        /* 通道0: ADC 灰度传感器值 */
        data[0] = (float)adc_read();

        /* 通道1: 正弦波 (幅度 2048, 偏置 2048, 周期约 200 帧) */
        data[1] = sinf(2.0f * 3.14159f * t / 200.0f) * 2048.0f + 2048.0f;

        /* 通道2: 锯齿波 (0 → 4095, 每 100 帧复位) */
        data[2] = (float)(t % 100) * 40.95f;

        /* 通道3: 方波 (高 4095 / 低 0, 每 50 帧翻转) */
        data[3] = ((t / 50) % 2) ? 4095.0f : 0.0f;

        vofa_send(data, 4);

        t++;
        delay_ms(20);  // 约 50Hz 刷新
    }
}
