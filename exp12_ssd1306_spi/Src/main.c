#include "systick.h"
#include "driver_spi1.h"
#include "driver_i2c1.h"
#include "driver_usart1.h"
#include "device_ssd1306.h"
#include "device_mpu6050.h"

static void uart_print_hex(uint8_t v)
{
    const char *hex = "0123456789ABCDEF";
    char s[6];
    s[0] = '0';
    s[1] = 'x';
    s[2] = hex[(v >> 4) & 0xF];
    s[3] = hex[v & 0xF];
    s[4] = '\r';
    s[5] = '\n';

    /* usart1_send_string 需要 null-terminated，這裡逐 byte 送 */
    for (int i = 0; i < 6; i++) {
        usart1_send_byte((uint8_t)s[i]);
    }
}

int main(void)
{
    systick_init(16000);

    spi1_init();
    i2c1_init();

    ssd1306_init();

    /* ---- 開機畫面 ---- */
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "Boot...");
    ssd1306_update_screen();
    delay_ms(500);

    /* ---- 初始化 USART1 設備 ---- */
    usart1_init(1200);
    usart1_send_string("Boot...\r\n");

    /* ---- I2C bus scan ---- */
    usart1_send_string("Scanning I2C bus...\r\n");
    uint8_t addrs[8];
    int n = i2c1_scan(addrs, 8);

    if (n == 0) {
        usart1_send_string("No I2C device found\r\n");
    } else {
        usart1_send_string("Found device(s):\r\n");
        for (int i = 0; i < n; i++) {
            uart_print_hex(addrs[i]);
        }
    }

    /* ---- 初始化 MPU6050 ---- */
    int mpu_st = mpu6050_init();
    if (mpu_st != 0) {
        ssd1306_clear();
        if (mpu_st == -2) {
            /* bus 卡住（SDA/SCL stuck），driver 已嘗試 recovery */
            ssd1306_draw_string(0, 0,  "I2C BUS STUCK");
            ssd1306_draw_string(0, 16, "Recovery tried");
        } else {
            /* NACK / WHO_AM_I 錯：sensor 沒接好 */
            ssd1306_draw_string(0, 0,  "MPU NOT FOUND");
            ssd1306_draw_string(0, 16, "Check wiring");
        }
        ssd1306_update_screen();
        while (1);
    }

    ssd1306_clear();
    ssd1306_draw_string(0, 0, "MPU6050 ready");
    ssd1306_update_screen();
    delay_ms(500);

    /* ---- 主迴圈：讀 + 顯示 ---- */
    mpu6050_axes_t acc = {0};
    mpu6050_read_accel(&acc);

    while (1) {
        /* 1. 準備新一幀 framebuffer */
        ssd1306_clear();
        ssd1306_draw_string(0, 0,  "Accel raw:");

        ssd1306_draw_string(0, 16, "X:");
        ssd1306_draw_int   (18, 16, acc.x);

        ssd1306_draw_string(0, 32, "Y:");
        ssd1306_draw_int   (18, 32, acc.y);

        ssd1306_draw_string(0, 48, "Z:");
        ssd1306_draw_int   (18, 48, acc.z);

        /* 2. 啟動 DMA 推送畫面（背景跑）*/
        ssd1306_update_screen_async();

        /* 3. 同時讀下一筆 MPU6050（CPU 跟 DMA 平行）*/
        mpu6050_read_accel(&acc);

        /* 4. 等 DMA 完成，下次 loop 才能再改 framebuffer */
        ssd1306_wait_update_done();

        delay_ms(50);
    }
}
