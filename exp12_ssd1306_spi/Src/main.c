#include "systick.h"
#include "driver_spi1.h"
#include "driver_i2c1.h"
#include "device_ssd1306.h"
#include "device_mpu6050.h"

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

    /* ---- 初始化 MPU6050 ---- */
    if (mpu6050_init() != 0) {
        ssd1306_clear();
        ssd1306_draw_string(0, 0,  "MPU NOT FOUND");
        ssd1306_draw_string(0, 16, "Check wiring");
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
