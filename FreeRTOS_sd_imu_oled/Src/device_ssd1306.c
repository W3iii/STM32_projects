/*
 * ssd1306.c
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#include "device_ssd1306.h"
#include "stm32f407xx.h"
#include "driver_spi1.h"
#include "device_ssd1306_font.h"

extern void delay_ms(uint32_t ms);

/* ============================================================================
 *                              GPIO 控制 macros
 * ============================================================================ */

#define OLED_CS_LOW()    (GPIOA->BSRR = (1U << (4 + 16))) // PA4 reset
#define OLED_CS_HIGH()   (GPIOA->BSRR = (1U << 4))        // PA4 set
#define OLED_DC_LOW()    (GPIOB->BSRR = (1U << (1 + 16))) // PB1 reset
#define OLED_DC_HIGH()   (GPIOB->BSRR = (1U << 1))        // PB1 set
#define OLED_RES_LOW()   (GPIOB->BSRR = (1U << (0 + 16))) // PB0 reset
#define OLED_RES_HIGH()  (GPIOB->BSRR = (1U << 0))		  // PB0 set

/* ============================================================================
 *                              Framebuffer
 *
 *  128 col × 8 page = 1024 bytes
 *  每個 byte 表示垂直 8 pixel（bit 0 在上，bit 7 在下）
 *
 *  Index 公式：
 *      pixel (x, y) → framebuffer[(y / 8) * 128 + x] 的第 (y % 8) 個 bit
 * ============================================================================ */

static uint8_t framebuffer[1024];

static void ssd1306_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PA4 = OLED CS
    GPIOA->MODER &= ~(0x3U << (4 * 2));
    GPIOA->MODER |=  (GPIO_MODER_OUTPUT << (4 * 2));

    // PB0 = OLED RES, PB1 = OLED DC
    GPIOB->MODER &= ~(0x3U << (0 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (0 * 2));
    GPIOB->MODER &= ~(0x3U << (1 * 2));
    GPIOB->MODER |=  (GPIO_MODER_OUTPUT << (1 * 2));

    OLED_CS_HIGH();
    OLED_RES_HIGH();
    OLED_DC_HIGH();
}

static void ssd1306_send_cmd(uint8_t cmd)
{
    OLED_DC_LOW();
    OLED_CS_LOW();
    spi1_send_byte(cmd);
    OLED_CS_HIGH();
}

static void ssd1306_reset(void)
{
    OLED_RES_HIGH();
    delay_ms(1);
    OLED_RES_LOW();
    delay_ms(10);
    OLED_RES_HIGH();
    delay_ms(10);
}

void ssd1306_init(void)
{
    ssd1306_gpio_init();
    ssd1306_reset();

    ssd1306_send_cmd(0xAE);
    ssd1306_send_cmd(0x20);
    ssd1306_send_cmd(0x00);
    ssd1306_send_cmd(0xB0);
    ssd1306_send_cmd(0xC8);
    ssd1306_send_cmd(0x00);
    ssd1306_send_cmd(0x10);
    ssd1306_send_cmd(0x40);
    ssd1306_send_cmd(0x81);
    ssd1306_send_cmd(0xFF);
    ssd1306_send_cmd(0xA1);
    ssd1306_send_cmd(0xA6);
    ssd1306_send_cmd(0xA8);
    ssd1306_send_cmd(0x3F);
    ssd1306_send_cmd(0xA4);
    ssd1306_send_cmd(0xD3);
    ssd1306_send_cmd(0x00);
    ssd1306_send_cmd(0xD5);
    ssd1306_send_cmd(0xF0);
    ssd1306_send_cmd(0xD9);
    ssd1306_send_cmd(0x22);
    ssd1306_send_cmd(0xDA);
    ssd1306_send_cmd(0x12);
    ssd1306_send_cmd(0xDB);
    ssd1306_send_cmd(0x20);
    ssd1306_send_cmd(0x8D);
    ssd1306_send_cmd(0x14);
    ssd1306_send_cmd(0xAF);
}

/* ============================================================================
 *                              畫圖 API（只動 framebuffer）
 * ============================================================================ */

/**
 * 清除 framebuffer（不推到螢幕，需自己呼叫 update_screen）
 */
void ssd1306_clear(void)
{
    for (uint16_t i = 0; i < 1024; i++) {
        framebuffer[i] = 0x00;
    }
}

/**
 * 整片填滿（不推到螢幕，需自己呼叫 update_screen）
 */
void ssd1306_fill(void)
{
    for (uint16_t i = 0; i < 1024; i++) {
        framebuffer[i] = 0xFF;
    }
}

/**
 * 畫單一 pixel
 *   x: 0~127
 *   y: 0~63
 *   color: 1=亮 0=暗
 */
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= 128 || y >= 64) return;        // 邊界檢查

    uint16_t index = (y / 8) * 128 + x;
    uint8_t  bit   = y % 8;

    if (color)
        framebuffer[index] |=  (1 << bit);
    else
        framebuffer[index] &= ~(1 << bit);
}

/**
 * 畫單一字元（通用版，y 可為任意值）
 *   x: 0~122（字寬 5+1 = 6）
 *   y: 0~56
 */
void ssd1306_draw_char(uint8_t x, uint8_t y, char c)
{
    if (c < 0x20 || c > 0x7E) c = '?';

    const uint8_t *glyph = font5x8[c - 0x20];

    for (uint8_t col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0; row < 8; row++) {
            uint8_t pixel = (bits >> row) & 0x1;
            ssd1306_draw_pixel(x + col, y + row, pixel);
        }
    }
}

/**
 * 畫字串
 *   遇到 \0 結束
 *   超出右邊界自動截斷
 */
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *s)
{
    while (*s) {
        if (x > 122) break;
        ssd1306_draw_char(x, y, *s);
        x += 6;                              // 5 pixel 字寬 + 1 pixel 間距
        s++;
    }
}

/**
 * 畫整數（含負數）
 *   不依賴 stdio.h 的 itoa / sprintf
 */
void ssd1306_draw_int(uint8_t x, uint8_t y, int32_t n)
{
    char buf[12];                            // -2147483648 + '\0' = 12 chars
    int  i   = 0;
    int  neg = 0;

    if (n < 0) {
        neg = 1;
        n   = -n;
    }

    if (n == 0) {
        buf[i++] = '0';
    } else {
        while (n > 0) {
            buf[i++] = '0' + (n % 10);
            n /= 10;
        }
    }
    if (neg) buf[i++] = '-';
    buf[i] = '\0';

    // 反轉字串
    for (int a = 0, b = i - 1; a < b; a++, b--) {
        char t = buf[a];
        buf[a] = buf[b];
        buf[b] = t;
    }

    ssd1306_draw_string(x, y, buf);
}

/* ============================================================================
 *                              把 framebuffer 推到 OLED
 *
 *  init 設定的 horizontal addressing mode (0x20, 0x00) 讓我們可以一次推 1024
 *  bytes，硬體自動換 col / 換 page。
 * ============================================================================ */

void ssd1306_update_screen(void)
{
    /* 設定寫入範圍：col 0~127, page 0~7 */
    ssd1306_send_cmd(0x21);
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(127);

    ssd1306_send_cmd(0x22);
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(7);

    /* 進入 data mode，準備一口氣推 1024 bytes */
    OLED_DC_HIGH();
    OLED_CS_LOW();

    /* 逾時時回 -1，但 OLED 少畫一幀不影響記錄，刻意忽略；
     * 重點是 spi1_wait_done 現在有上限，不會抱著 spi1_lock 空轉。 */
    (void)spi1_send_buffer(framebuffer, 1024);

    OLED_CS_HIGH();
}

void ssd1306_update_screen_async(void)
{
    /* 設定寫入範圍 */
    ssd1306_send_cmd(0x21);
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(127);

    ssd1306_send_cmd(0x22);
    ssd1306_send_cmd(0);
    ssd1306_send_cmd(7);

    /* 切到 data mode、拉低 CS、啟動 DMA 就 return */
    OLED_DC_HIGH();
    OLED_CS_LOW();
    spi1_send_buffer_async(framebuffer, 1024);
    /* CS 不在這拉高！等 wait 函式裡才拉 */
}

void ssd1306_wait_update_done(void)
{
    (void)spi1_wait_done();  // 等 DMA + SPI 完成（逾時回 -1，同上刻意忽略）
    OLED_CS_HIGH();          // 才拉高 CS
}

int ssd1306_is_updating(void)
{
    return spi1_is_busy();
}
