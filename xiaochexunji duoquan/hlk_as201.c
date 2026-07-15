/**
 * @file    hlk_as201.c
 * @brief   HLK-AS201 姿态传感器驱动实现 (MSPM0G3507 / CCS)
 *
 * 依据《HLK-AS201系列姿态传感器模块说明书 V1.1》第 11 章实现。
 *
 * 数据上报帧 (cmd=0x00):
 *   head(FA FB) | len | 0x00 | sensor_type(1) | [各订阅字段...] | check | tail(FC FD)
 *   - sensor_type: bit0~1 轴数(0=10,1=9,2=6); bit2~3 磁力计精度
 *   - 各字段是否出现由"订阅标识"决定, 出现顺序固定:
 *     加速度6 → 角速度6 → 角度6 → 磁场6 → 四元数8 → 温度2 → 气压4 → 高度4
 *   - 所有分量小端; 加/角/角度/磁/四元数/温度为 int16, 气压/高度为 int32
 */

#include "hlk_as201.h"

/*===========================================================================
 * 小端解包
 *===========================================================================*/
static inline int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/** @brief 校验和 = (cmd + data 各字节) 累加取低 8 位 */
static uint8_t calc_check(const uint8_t *cmd_to_data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += cmd_to_data[i];
    return (uint8_t)(sum & 0xFF);
}

/*===========================================================================
 * 机型能力掩码
 *   说明书第 1 页选型表 + 第 16 页显示逻辑:
 *   上报字段 = 机型硬件能力 ∩ 订阅标识。
 *   - 6 轴 : 无磁场、无气压、无高度
 *   - 9 轴 : 无气压、无高度 (有磁场)
 *   - 10轴 : 全部
 *   加速度/角速度/角度/四元数/温度所有机型都有。
 *===========================================================================*/
static uint8_t capability_mask(uint8_t sensor_type)
{
    switch (sensor_type) {
    case AS201_SENSOR_6AXIS:
        return (uint8_t)(AS201_SUB_ALL & ~(AS201_SUB_MAG |
                                           AS201_SUB_PRESSURE |
                                           AS201_SUB_HEIGHT));
    case AS201_SENSOR_9AXIS:
        return (uint8_t)(AS201_SUB_ALL & ~(AS201_SUB_PRESSURE |
                                           AS201_SUB_HEIGHT));
    case AS201_SENSOR_10AXIS:
    default:
        return AS201_SUB_ALL;
    }
}

/** @brief 按有效字段掩码计算数据上报 payload 字节数 (不含 sensor_type) */
static uint8_t report_payload_len(uint8_t active)
{
    uint8_t n = 0;
    if (active & AS201_SUB_ACCEL)    n += 6;
    if (active & AS201_SUB_GYRO)     n += 6;
    if (active & AS201_SUB_ANGLE)    n += 6;
    if (active & AS201_SUB_MAG)      n += 6;
    if (active & AS201_SUB_QUAT)     n += 8;
    if (active & AS201_SUB_TEMP)     n += 2;
    if (active & AS201_SUB_PRESSURE) n += 4;
    if (active & AS201_SUB_HEIGHT)   n += 4;
    return n;
}

/*===========================================================================
 * 数据上报帧解析 (frame[] 布局: [0]=cmd(0x00) [1]=sensor_type [2..]=payload)
 *===========================================================================*/
static void parse_data_report(AS201_Handle *h, const uint8_t *f, uint8_t flen)
{
    /* f[0]=0x00, f[1]=sensor_type, payload 从 f[2] 起 */
    uint8_t st  = f[1];
    const uint8_t *p = &f[2];
    const uint8_t *end = f + flen;      /* flen = 1(cmd) + payload_len */

    h->data.sensor_type  = st & 0x03;
    h->data.mag_accuracy = (st >> 2) & 0x03;
    h->data.update_flags = 0;

    /* 有效字段 = 订阅标识 ∩ 机型能力: 6/9 轴不含的字段绝不出现在帧里,
     * 否则会从缺失字段起整体错位 */
    uint8_t sub = (uint8_t)(h->subscription & capability_mask(h->data.sensor_type));

    /* 帧长自检: 期望 payload 长度须与实收一致, 不符说明订阅掩码与模块
     * 实际不同步 (或机型判断有误), 跳过本帧避免静默错位 */
    uint8_t expect = report_payload_len(sub);
    uint8_t actual = (uint8_t)(end - p);
    if (expect != actual) {
        h->len_mismatch_count++;
        return;
    }

    /* 逐字段按订阅顺序解析, 每步前检查剩余长度, 防越界 */
    if (sub & AS201_SUB_ACCEL) {
        if (p + 6 > end) return;
        h->data.accel.x = rd_i16(p + 0) * AS201_K_ACCEL;
        h->data.accel.y = rd_i16(p + 2) * AS201_K_ACCEL;
        h->data.accel.z = rd_i16(p + 4) * AS201_K_ACCEL;
        h->data.update_flags |= AS201_SUB_ACCEL;
        p += 6;
    }
    if (sub & AS201_SUB_GYRO) {
        if (p + 6 > end) return;
        h->data.gyro.x = rd_i16(p + 0) * AS201_K_GYRO;
        h->data.gyro.y = rd_i16(p + 2) * AS201_K_GYRO;
        h->data.gyro.z = rd_i16(p + 4) * AS201_K_GYRO;
        h->data.update_flags |= AS201_SUB_GYRO;
        p += 6;
    }
    if (sub & AS201_SUB_ANGLE) {
        if (p + 6 > end) return;
        h->data.angle.roll  = rd_i16(p + 0) * AS201_K_ANGLE;
        h->data.angle.pitch = rd_i16(p + 2) * AS201_K_ANGLE;
        h->data.angle.yaw   = rd_i16(p + 4) * AS201_K_ANGLE;
        h->data.update_flags |= AS201_SUB_ANGLE;
        p += 6;
    }
    if (sub & AS201_SUB_MAG) {
        if (p + 6 > end) return;
        h->data.mag.x = rd_i16(p + 0) * AS201_K_MAG;
        h->data.mag.y = rd_i16(p + 2) * AS201_K_MAG;
        h->data.mag.z = rd_i16(p + 4) * AS201_K_MAG;
        h->data.update_flags |= AS201_SUB_MAG;
        p += 6;
    }
    if (sub & AS201_SUB_QUAT) {
        if (p + 8 > end) return;
        h->data.quat.q0 = rd_i16(p + 0) * AS201_K_QUAT;
        h->data.quat.q1 = rd_i16(p + 2) * AS201_K_QUAT;
        h->data.quat.q2 = rd_i16(p + 4) * AS201_K_QUAT;
        h->data.quat.q3 = rd_i16(p + 6) * AS201_K_QUAT;
        h->data.update_flags |= AS201_SUB_QUAT;
        p += 8;
    }
    if (sub & AS201_SUB_TEMP) {
        if (p + 2 > end) return;
        h->data.temperature = rd_i16(p) * AS201_K_TEMP;
        h->data.update_flags |= AS201_SUB_TEMP;
        p += 2;
    }
    if (sub & AS201_SUB_PRESSURE) {
        if (p + 4 > end) return;
        h->data.pressure = rd_i32(p) * AS201_K_PRESSURE;
        h->data.update_flags |= AS201_SUB_PRESSURE;
        p += 4;
    }
    if (sub & AS201_SUB_HEIGHT) {
        if (p + 4 > end) return;
        h->data.height = rd_i32(p) * AS201_K_HEIGHT;
        h->data.update_flags |= AS201_SUB_HEIGHT;
        p += 4;
    }
}

/** @brief 处理一个校验通过的完整帧 (frame[] = cmd..data, 不含 check/tail) */
static void handle_frame(AS201_Handle *h, uint8_t cmd, uint8_t payload_len)
{
    /* h->frame[0]=cmd, h->frame[1..payload_len]=data */
    switch (cmd) {
    case AS201_CMD_DATA_REPORT:
    case AS201_CMD_GET_ONCE:            /* 0x1B 回包体同 0x00 */
        /* frame 内容: [0]=cmd [1]=sensor_type [2..] payload
         * 传给解析器的 end = frame + 1 + payload_len (payload_len 含 sensor_type) */
        parse_data_report(h, h->frame, (uint8_t)(1 + payload_len));
        h->frame_count++;
        break;

    case AS201_CMD_GET_CONFIG:          /* 0x19 回包: 订阅标识/速率/波特率/上报开关 */
        /* 说明书示例: FA FB 06 19 FF 01 06 00 xx FC FD
         * data = [订阅标识][回传速率][串口波特率][数据上报开关], 共 4 字节 */
        if (payload_len >= 4) {
            h->cfg_subscription = h->frame[1];
            h->cfg_rate         = h->frame[2];
            h->cfg_baud         = h->frame[3];
            h->cfg_report_on    = h->frame[4];
            /* 关键: 用模块实际订阅同步本地解析掩码, 否则长度自检会误判 */
            h->subscription     = h->cfg_subscription;
            h->cfg_valid        = true;
        }
        break;

    /* 其他命令 (版本/ACK) 可在此扩展; 此处忽略 */
    default:
        break;
    }
}

/*===========================================================================
 * 环形缓冲区 (ISR 写 / Poll 读)
 *===========================================================================*/
static bool buf_get(AS201_Handle *h, uint8_t *b)
{
    if (h->rx_head == h->rx_tail) return false;
    *b = h->rx_buf[h->rx_tail];
    h->rx_tail = (uint16_t)((h->rx_tail + 1) % AS201_RX_BUF_SIZE);
    return true;
}

/*===========================================================================
 * 帧接收状态机
 *   0 等 HEAD0(FA)  1 等 HEAD1(FB)  2 等 len  3 收 cmd..check  4 等 TAIL0(FC)  5 等 TAIL1(FD)
 *===========================================================================*/
enum { S_H0, S_H1, S_LEN, S_BODY, S_T0, S_T1 };

static void fsm(AS201_Handle *h, uint8_t b)
{
    switch (h->state) {
    case S_H0:
        if (b == AS201_HEAD0) h->state = S_H1;
        break;

    case S_H1:
        if (b == AS201_HEAD1)      h->state = S_LEN;
        else if (b == AS201_HEAD0) h->state = S_H1;   /* 连续 FA, 停在此 */
        else                       h->state = S_H0;
        break;

    case S_LEN:
        /* len = cmd..check 的字节数, 至少 2 (cmd + check), 不超过缓冲 */
        if (b < 2 || b > AS201_FRAME_MAX) { h->state = S_H0; break; }
        h->need = b;            /* 还需接收 need 个字节 (cmd..check) */
        h->idx  = 0;
        h->state = S_BODY;
        break;

    case S_BODY:
        h->frame[h->idx++] = b;
        if (h->idx >= h->need) h->state = S_T0;   /* cmd..check 收齐 */
        break;

    case S_T0:
        if (b == AS201_TAIL0) {
            h->state = S_T1;
        } else {
            h->state = (b == AS201_HEAD0) ? S_H1 : S_H0;
        }
        break;

    case S_T1:
        if (b == AS201_TAIL1) {
            /* 帧完整: frame[0..need-1] = cmd..check
             * check = frame[need-1]; 校验范围 frame[0..need-2] (cmd+data) */
            uint8_t recv_check = h->frame[h->need - 1];
            uint8_t calc = calc_check(h->frame, (uint8_t)(h->need - 1));
            if (recv_check == calc) {
                uint8_t cmd = h->frame[0];
                uint8_t payload_len = (uint8_t)(h->need - 2); /* data 字节数 */
                handle_frame(h, cmd, payload_len);
            } else {
                h->checksum_err_count++;
            }
            h->state = S_H0;
        } else {
            /* 帧尾第二字节不符: 重同步. 若恰为 FA 则可能是下一帧起始,
             * 回到 S_H1 而非 S_H0, 少丢一帧 */
            h->state = (b == AS201_HEAD0) ? S_H1 : S_H0;
        }
        break;

    default:
        h->state = S_H0;
        break;
    }
}

/*===========================================================================
 * 公用 API
 *===========================================================================*/
AS201_Status AS201_Init(AS201_Handle *h, UART_Regs *uart, IRQn_Type irq)
{
    if (h == NULL || uart == NULL) return AS201_ERR_PARAM;

    memset(h, 0, sizeof(*h));
    h->uart         = uart;
    h->irq          = irq;
    h->state        = S_H0;
    h->subscription = AS201_SUB_ALL;    /* 模块默认全订阅; 用 SetSubscription 改 */

    DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(irq);
    NVIC_EnableIRQ(irq);
    return AS201_OK;
}

void AS201_Poll(AS201_Handle *h)
{
    if (h == NULL || h->uart == NULL) return;
    uint8_t b;
    while (buf_get(h, &b)) fsm(h, b);
}

const AS201_Data *AS201_GetData(const AS201_Handle *h)
{
    return (h == NULL) ? NULL : &h->data;
}

bool AS201_HasNewFrame(const AS201_Handle *h, uint32_t *last_count)
{
    if (h == NULL || last_count == NULL) return false;
    uint32_t now = h->frame_count;
    if (now != *last_count) { *last_count = now; return true; }
    return false;
}

/*---- 命令发送 ----*/
AS201_Status AS201_SendCommand(AS201_Handle *h, uint8_t cmd,
                               const uint8_t *data, uint8_t dlen)
{
    if (h == NULL || h->uart == NULL) return AS201_ERR_PARAM;
    if (dlen > (AS201_FRAME_MAX - 2)) return AS201_ERR_PARAM;

    uint8_t body[AS201_FRAME_MAX];      /* cmd..check */
    uint8_t n = 0;
    body[n++] = cmd;
    for (uint8_t i = 0; i < dlen; i++) body[n++] = data[i];
    /* check 覆盖 cmd+data. 先算再存: 避免同一表达式内 n++ 与读 n 的
     * 求值顺序未定义 (否则校验和可能把 check 字节自身也算进去) */
    body[n] = calc_check(body, n);
    n++;

    uint8_t len = n;                    /* len = cmd..check 字节数 */

    DL_UART_Main_transmitDataBlocking(h->uart, AS201_HEAD0);
    DL_UART_Main_transmitDataBlocking(h->uart, AS201_HEAD1);
    DL_UART_Main_transmitDataBlocking(h->uart, len);
    for (uint8_t i = 0; i < n; i++)
        DL_UART_Main_transmitDataBlocking(h->uart, body[i]);
    DL_UART_Main_transmitDataBlocking(h->uart, AS201_TAIL0);
    DL_UART_Main_transmitDataBlocking(h->uart, AS201_TAIL1);
    return AS201_OK;
}

AS201_Status AS201_SetSubscription(AS201_Handle *h, uint8_t flags)
{
    AS201_Status s = AS201_SendCommand(h, AS201_CMD_SET_SUBSCRIBE, &flags, 1);
    if (s == AS201_OK) h->subscription = flags;  /* 本地解析掩码须同步 */
    return s;
}

AS201_Status AS201_SetRate(AS201_Handle *h, uint8_t rate_code)
{
    if (rate_code < AS201_RATE_0P1HZ || rate_code > AS201_RATE_20HZ)
        return AS201_ERR_PARAM;
    return AS201_SendCommand(h, AS201_CMD_SET_RATE, &rate_code, 1);
}

AS201_Status AS201_SetReportEnable(AS201_Handle *h, bool on)
{
    uint8_t d = on ? 1 : 0;
    return AS201_SendCommand(h, AS201_CMD_REPORT_SWITCH, &d, 1);
}

AS201_Status AS201_CalibAccGyro(AS201_Handle *h)
{
    uint8_t d = 1;                      /* data=1, 示例 FA FB 03 1C 01 1D FC FD */
    return AS201_SendCommand(h, AS201_CMD_CALIB_ACC_GYRO, &d, 1);
}

AS201_Status AS201_CalibMagStart(AS201_Handle *h)
{
    uint8_t d = 1;
    return AS201_SendCommand(h, AS201_CMD_CALIB_MAG_START, &d, 1);
}

AS201_Status AS201_CalibMagDone(AS201_Handle *h)
{
    uint8_t d = 1;
    return AS201_SendCommand(h, AS201_CMD_CALIB_MAG_DONE, &d, 1);
}

AS201_Status AS201_RestoreFactory(AS201_Handle *h)
{
    uint8_t d = 1;
    return AS201_SendCommand(h, AS201_CMD_RESTORE_FACTORY, &d, 1);
}

AS201_Status AS201_Reboot(AS201_Handle *h)
{
    uint8_t d = 1;
    return AS201_SendCommand(h, AS201_CMD_REBOOT, &d, 1);
}

AS201_Status AS201_RequestVersion(AS201_Handle *h)
{
    uint8_t d = 1;                      /* 示例 FA FB 03 10 01 11 FC FD */
    return AS201_SendCommand(h, AS201_CMD_GET_VERSION, &d, 1);
}

AS201_Status AS201_RequestConfig(AS201_Handle *h)
{
    uint8_t d = 1;                      /* 示例 FA FB 03 19 01 1A FC FD */
    if (h != NULL) h->cfg_valid = false;    /* 清标志, 等回包置位 */
    return AS201_SendCommand(h, AS201_CMD_GET_CONFIG, &d, 1);
}

AS201_Status AS201_SyncConfig(AS201_Handle *h,
                              uint32_t timeout_ms,
                              volatile uint32_t *tick_ms)
{
    if (h == NULL || h->uart == NULL || tick_ms == NULL) return AS201_ERR_PARAM;

    AS201_Status s = AS201_RequestConfig(h);
    if (s != AS201_OK) return s;

    uint32_t start = *tick_ms;
    while (!h->cfg_valid) {
        AS201_Poll(h);                          /* 驱动解析, 回包会置 cfg_valid */
        if ((uint32_t)(*tick_ms - start) >= timeout_ms) {
            return AS201_ERR_TIMEOUT;
        }
    }
    return AS201_OK;
}

/*===========================================================================
 * 中断服务
 *===========================================================================*/
void AS201_ISR(AS201_Handle *h)
{
    if (h == NULL || h->uart == NULL) return;

    /* MSPM0 的 IIDX 是"索引值", 不是位掩码, 必须用 == 判断 */
    switch (DL_UART_Main_getPendingInterrupt(h->uart)) {
    case DL_UART_MAIN_IIDX_RX:
        /* RX FIFO 阈值可能 >1, 循环取空 */
        while (DL_UART_Main_isRXFIFOEmpty(h->uart) == false) {
            uint8_t  d    = DL_UART_Main_receiveData(h->uart);
            uint16_t next = (uint16_t)((h->rx_head + 1) % AS201_RX_BUF_SIZE);
            /* SPSC 环形队列: 生产者(本 ISR)只写 head, 消费者(Poll)只写 tail.
             * 满时丢弃"新"字节而非改 tail —— 若在此改 tail 会与 buf_get 里
             * 的 tail 自增产生数据竞争, 破坏无锁前提. 溢出计数便于现场排查. */
            if (next == h->rx_tail) {
                h->overflow_count++;
                break;      /* 缓冲已满, 停止读取, 剩余留在 FIFO/下次中断 */
            }
            h->rx_buf[h->rx_head] = d;
            h->rx_head = next;
        }
        break;
    default:
        break;
    }
}
