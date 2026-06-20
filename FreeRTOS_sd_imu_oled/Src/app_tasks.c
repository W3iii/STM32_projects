/*
 * app_tasks.c
 *
 *  FreeRTOS 四 task 架構（SPEC §7）。把 Phase 1 的單一 super-loop 拆成：
 *
 *    Sensor (prio4, 100Hz, vTaskDelayUntil)
 *        └ 只在 LOGGING 時讀 MPU6050 → xQueueSend(qSamples)
 *           佇列滿 → 計 overrun（不阻塞，保住取樣節奏，SPEC §7.2-2）
 *    Logger (prio3, 事件驅動)
 *        └ xQueueReceive → app_datalogger_write_sample()；每 1s sync
 *           唯一主要寫卡者；FatFs 操作走 bus_lock（SPI1 互斥）
 *    Display(prio1, 10Hz)
 *        └ 取狀態快照 → bus_lock → 畫 OLED → bus_unlock
 *    Console(prio2, ~10ms)
 *        └ app_cli_poll() + app_button_poll()；按鈕 toggle start/stop
 *
 *  共用資源與同步：
 *    qSamples : Sensor→Logger 佇列（producer/consumer 解耦，抗 SD stall）
 *    mBus     : SPI1 匯流排互斥鎖（OLED 與 SD/FatFs 共用，SPEC §7.2-1）
 *    （狀態 mutex 在 app_datalogger.c，保護 state/count/filename）
 *
 *  Created on: Jun 20, 2026
 *      Author: xushu
 */

#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "systick.h"
#include "driver_spi1.h"
#include "driver_i2c1.h"
#include "driver_usart1.h"
#include "device_ssd1306.h"
#include "device_sdcard.h"
#include "device_mpu6050.h"
#include "app_datalogger.h"
#include "app_cli.h"
#include "app_button.h"

/* ---- task 參數 ---- */
#define SENSOR_PERIOD_MS     10          /* 100Hz */
#define DISPLAY_PERIOD_MS    100         /* ~10Hz */
#define CONSOLE_PERIOD_MS    10          /* CLI 取字 + 按鈕去抖輪詢 */
#define LOGGER_SYNC_MS       1000        /* 每秒 f_sync */

#define QSAMPLES_LEN         64          /* 64 × 16B ≈ 640ms 緩衝 */

#define PRIO_SENSOR          4
#define PRIO_LOGGER          3
#define PRIO_CONSOLE         2
#define PRIO_DISPLAY         1
#define PRIO_BOOTSTRAP       (configMAX_PRIORITIES - 1)   /* 先跑完初始化 */

#define STACK_SENSOR         256
#define STACK_LOGGER         1024        /* FatFs + f_printf 吃 stack */
#define STACK_DISPLAY        256
#define STACK_CONSOLE        1024        /* ls/dump/clear 走 FatFs */
#define STACK_BOOTSTRAP      1024        /* f_mount 走 FatFs */

/* ---- 同步原語 ---- */
static QueueHandle_t     s_qSamples;
static SemaphoreHandle_t s_mBus;
static volatile uint32_t s_overruns;

void bus_lock(void)   { xSemaphoreTake(s_mBus, portMAX_DELAY); }
void bus_unlock(void) { xSemaphoreGive(s_mBus); }

uint32_t app_tasks_overruns(void) { return s_overruns; }

/* ───────────────── Sensor task ───────────────── */
static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

        if (app_datalogger_state() != DL_LOGGING) {
            continue;                       /* 沒在記錄就不取樣（省 I2C/功耗）*/
        }

        imu_sample_t s;
        s.t_ms = systick_get_ms();
        mpu6050_read_all(&s.acc, &s.gyro);

        if (xQueueSend(s_qSamples, &s, 0) != pdTRUE) {
            s_overruns++;                   /* 佇列滿：丟一筆，但不拖慢取樣 */
        }
    }
}

/* ───────────────── Logger task ───────────────── */
static void logger_task(void *arg)
{
    (void)arg;
    TickType_t last_sync = xTaskGetTickCount();
    for (;;) {
        imu_sample_t s;
        /* 等樣本；timeout 讓我們也能定期 sync、不會永遠卡住 */
        if (xQueueReceive(s_qSamples, &s, pdMS_TO_TICKS(100)) == pdTRUE) {
            app_datalogger_write_sample(s.t_ms, &s.acc, &s.gyro);
        }

        if ((xTaskGetTickCount() - last_sync) >= pdMS_TO_TICKS(LOGGER_SYNC_MS)) {
            last_sync = xTaskGetTickCount();
            app_datalogger_sync();          /* 內部只在 LOGGING 時做事 */
        }
    }
}

/* ───────────────── Display task ───────────────── */
static void display_task(void *arg)
{
    (void)arg;
    for (;;) {
        dl_status_t st;
        app_datalogger_get_status(&st);

        bus_lock();
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "IMU LOGGER");
        if (st.state == DL_LOGGING) {
            ssd1306_draw_string(0, 16, "* REC");
            ssd1306_draw_int(0, 32, (int32_t)st.count);
        } else {
            ssd1306_draw_string(0, 16, "READY");
        }
        ssd1306_update_screen();            /* 同步更新，鎖內完成 */
        bus_unlock();

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* ───────────────── Console task ───────────────── */
/* 按鈕按下 → 切換記錄狀態。OLED 由 Display task 反映，這裡補 UART 回饋。 */
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

static void console_task(void *arg)
{
    (void)arg;
    for (;;) {
        app_cli_poll();                     /* 取 UART、執行指令（內部自鎖 FatFs）*/
        if (app_button_poll(systick_get_ms())) {
            toggle_logging();
        }
        vTaskDelay(pdMS_TO_TICKS(CONSOLE_PERIOD_MS));
    }
}

/* ───────────────── Bootstrap task ─────────────────
 * 初始化集中在這裡：跑在排程器啟動後（delay_ms = vTaskDelay 才有效），
 * 完成後建立四個工作 task，再把自己刪掉。 */
static void bootstrap_task(void *arg)
{
    (void)arg;

    /* ---- 匯流排 / 周邊初始化 ---- */
    usart1_init(1200);            // UART console（CH340 不穩就 1200）
    spi1_init();                  // SPI1（OLED + SD 共用）
    i2c1_init();                  // I2C1（MPU6050）
    sd_cs_init();                 // SD CS = PB10
    ssd1306_init();               // OLED
    delay_ms(10);

    usart1_send_string("\r\n=== IMU datalogger (FreeRTOS, 4 tasks) ===\r\n");

    /* ---- Application 初始化（MPU6050 + 掛載 FAT32）---- */
    int rc = app_datalogger_init();
    if (rc == -1) {
        usart1_send_string("[FATAL] MPU6050 not found\r\n");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "MPU FAIL");
        ssd1306_update_screen();
        vTaskSuspend(NULL);
    } else if (rc == -2) {
        usart1_send_string("[FATAL] FAT32 mount failed\r\n");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "FS FAIL");
        ssd1306_update_screen();
        vTaskSuspend(NULL);
    } else if (rc != 0) {
        usart1_send_string("[FATAL] init error\r\n");
        vTaskSuspend(NULL);
    }
    usart1_send_string("[OK]   sensors + FAT32 ready\r\n");

    app_cli_init();
    app_button_init();            // PA0 user button：按一下切換 start/stop

    /* ---- 建立四個工作 task ---- */
    xTaskCreate(sensor_task,  "sensor",  STACK_SENSOR,  NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(logger_task,  "logger",  STACK_LOGGER,  NULL, PRIO_LOGGER,  NULL);
    xTaskCreate(display_task, "display", STACK_DISPLAY, NULL, PRIO_DISPLAY, NULL);
    xTaskCreate(console_task, "console", STACK_CONSOLE, NULL, PRIO_CONSOLE, NULL);

    vTaskDelete(NULL);            // 初始化完成，bootstrap 退場
}

void app_tasks_start(void)
{
    s_qSamples = xQueueCreate(QSAMPLES_LEN, sizeof(imu_sample_t));
    s_mBus     = xSemaphoreCreateMutex();
    configASSERT(s_qSamples != NULL && s_mBus != NULL);

    xTaskCreate(bootstrap_task, "boot", STACK_BOOTSTRAP, NULL, PRIO_BOOTSTRAP, NULL);
}
