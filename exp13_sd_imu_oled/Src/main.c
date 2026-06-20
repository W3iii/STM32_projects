/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IMU datalogger — bare-metal 整合主程式
 *
 *  MPU6050(I2C1) → 100Hz 取樣 → FatFs/CSV → SD(SPI1)
 *  OLED(SPI1) 顯示狀態、UART(USART1) 提供 start/stop/status CLI。
 *
 *  主迴圈是 co-operative scheduler（SPEC §6.5），之後可平移到 FreeRTOS。
 ******************************************************************************
 */

#include <stdint.h>
#include "systick.h"
#include "driver_spi1.h"
#include "driver_i2c1.h"
#include "driver_usart1.h"
#include "device_ssd1306.h"
#include "device_sdcard.h"
#include "app_datalogger.h"
#include "app_cli.h"
#include "app_button.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define DISPLAY_PERIOD_MS  100   /* OLED 更新 ~10Hz */

/* 更新 OLED 狀態畫面。用同步更新（內部等 DMA 完成）才不會和 SD
 * 在共用的 SPI1 上撞車（bare-metal 階段最簡單的做法）。 */
static void update_oled(void)
{
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "IMU LOGGER");

    if (app_datalogger_state() == DL_LOGGING) {
        ssd1306_draw_string(0, 16, "* REC");
        ssd1306_draw_int(0, 32, (int32_t)app_datalogger_count());
    } else {
        ssd1306_draw_string(0, 16, "READY");
    }
    ssd1306_update_screen();
}

/* 按鈕按下 → 切換記錄狀態。OLED 由 update_oled() 自動反映，這裡補 UART 回饋。 */
static void toggle_logging(void)
{
    if (app_datalogger_state() == DL_LOGGING) {
        app_datalogger_stop();
        usart1_send_string("\r\n[BTN] stopped\r\n> ");
    } else if (app_datalogger_start() == 0) {
        usart1_send_string("\r\n[BTN] started: ");
        usart1_send_string(app_datalogger_filename());
        usart1_send_string("\r\n> ");
    } else {
        usart1_send_string("\r\n[BTN] start failed\r\n> ");
    }
}

int main(void)
{
    /* ---- 匯流排 / 周邊初始化 ---- */
    systick_init(16000);          // 16MHz HSI → 1ms tick
    usart1_init(1200);            // UART console（CH340 不穩就 1200）
    spi1_init();                  // SPI1（OLED + SD 共用）
    i2c1_init();                  // I2C1（MPU6050）
    sd_cs_init();                 // SD CS = PB10
    ssd1306_init();               // OLED
    delay_ms(10);

    usart1_send_string("\r\n=== IMU datalogger ===\r\n");

    /* ---- Application 初始化（MPU6050 + 掛載 FAT32）---- */
    int rc = app_datalogger_init();
    if (rc == -1) {
        usart1_send_string("[FATAL] MPU6050 not found\r\n");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "MPU FAIL");
        ssd1306_update_screen();
        for (;;);
    } else if (rc == -2) {
        usart1_send_string("[FATAL] FAT32 mount failed\r\n");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "FS FAIL");
        ssd1306_update_screen();
        for (;;);
    }
    usart1_send_string("[OK]   sensors + FAT32 ready\r\n");

    app_cli_init();
    app_button_init();            // PA0 user button：按一下切換 start/stop

    /* ---- co-operative 主迴圈 ---- */
    uint32_t t_display = 0;
    for (;;) {
        uint32_t now = systick_get_ms();

        app_datalogger_tick(now);      // 100Hz 取樣 + 週期 sync（內部自己看時間）
        app_cli_poll();                // 處理 UART 命令
        if (app_button_poll(now)) {    // PA0 去抖 + 邊緣偵測
            toggle_logging();          // 切換交給 main
        }

        if (now - t_display >= DISPLAY_PERIOD_MS) {
            t_display = now;
            update_oled();
        }
    }
}
