#include "st7789.h"

// SPI 轮询超时，防止 MCU 死锁（约 3ms @ 32MHz）
#define SPI_TIMEOUT  100000U

// 内部毫秒延时
static void delay_ms(uint32_t ms) {
    delay_cycles(CPUCLK_FREQ / 1000 * ms);
}

// 利用硬件 FIFO 写单字节，带超时保护
inline void SPI_WriteByte(uint8_t data) {
    volatile uint32_t t = SPI_TIMEOUT;
    while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --t);
    DL_SPI_transmitData8(SPI_LCD_INST, data);
}

void LCD_Write_Cmd(uint8_t cmd) {
    LCD_CS_CLR();
    LCD_DC_CLR();
    SPI_WriteByte(cmd);
    volatile uint32_t t = SPI_TIMEOUT;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --t);
    LCD_CS_SET();
}

void LCD_Write_Data(uint8_t data) {
    LCD_CS_CLR();
    LCD_DC_SET();
    SPI_WriteByte(data);
    volatile uint32_t t = SPI_TIMEOUT;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --t);
    LCD_CS_SET();
}

void LCD_Write_Data16(uint16_t data) {
    LCD_CS_CLR();
    LCD_DC_SET();
    SPI_WriteByte(data >> 8);
    SPI_WriteByte(data & 0xFF);
    volatile uint32_t t = SPI_TIMEOUT;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --t);
    LCD_CS_SET();
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_Write_Cmd(0x2A);
    LCD_Write_Data16(x1);
    LCD_Write_Data16(x2);
    LCD_Write_Cmd(0x2B);
    LCD_Write_Data16(y1);
    LCD_Write_Data16(y2);
    LCD_Write_Cmd(0x2C);
}

void LCD_Init(void) {
    LCD_BLK_SET();

    // 硬件复位时序
    LCD_RES_SET();  delay_ms(10);
    LCD_RES_CLR();  delay_ms(50);
    LCD_RES_SET();  delay_ms(120);

    // ST7789 唤醒序列
    LCD_Write_Cmd(0x11); delay_ms(120);
    LCD_Write_Cmd(0x36); LCD_Write_Data(0x00);
    LCD_Write_Cmd(0x3A); LCD_Write_Data(0x05);
    LCD_Write_Cmd(0x21);

    // 电压及帧率控制参数
    LCD_Write_Cmd(0xB2);
    LCD_Write_Data(0x0C); LCD_Write_Data(0x0C);
    LCD_Write_Data(0x00); LCD_Write_Data(0x33); LCD_Write_Data(0x33);
    LCD_Write_Cmd(0xB7); LCD_Write_Data(0x35);
    LCD_Write_Cmd(0xBB); LCD_Write_Data(0x19);
    LCD_Write_Cmd(0xC0); LCD_Write_Data(0x2C);
    LCD_Write_Cmd(0xC2); LCD_Write_Data(0x01);
    LCD_Write_Cmd(0xC3); LCD_Write_Data(0x12);
    LCD_Write_Cmd(0xC4); LCD_Write_Data(0x20);
    LCD_Write_Cmd(0xC6); LCD_Write_Data(0x0F);
    LCD_Write_Cmd(0xD0); LCD_Write_Data(0xA4); LCD_Write_Data(0xA1);

    LCD_Write_Cmd(0x29); delay_ms(20);
}

void LCD_Clear(uint16_t color) {
    uint32_t total = (uint32_t)LCD_WIDTH * LCD_HEIGHT;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    LCD_Address_Set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    LCD_CS_CLR();
    LCD_DC_SET();

    while (total--) {
        volatile uint32_t t = SPI_TIMEOUT;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --t);
        if (!t) break;
        DL_SPI_transmitData8(SPI_LCD_INST, hi);

        t = SPI_TIMEOUT;
        while (DL_SPI_isTXFIFOFull(SPI_LCD_INST) && --t);
        if (!t) break;
        DL_SPI_transmitData8(SPI_LCD_INST, lo);
    }

    volatile uint32_t t = SPI_TIMEOUT;
    while (DL_SPI_isBusy(SPI_LCD_INST) && --t);
    LCD_CS_SET();
}

