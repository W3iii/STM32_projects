/*
 * app_tasks.c
 *
 *  FreeRTOS task 架構。把 Phase 1 的單一 super-loop 拆成事件驅動的多 task：
 *
 *    Sensor (prio4, 事件驅動 + 100Hz cadence)
 *        └ 平時 block 在 ulTaskNotifyTake；start 喚醒 → 進 vTaskDelayUntil 100Hz
 *           迴圈讀 MPU6050 → xQueueSend(qSamples)；stop 後退回睡死（IDLE 時 0% CPU）
 *           佇列滿 → 計 overrun（不阻塞，保住取樣節奏）
 *    Logger (prio3, 事件驅動)
 *        └ xQueueReceive → app_datalogger_write_sample()；每 1s sync
 *           唯一主要寫卡者；FatFs 操作走 spi1_lock（SPI1 互斥）
 *    Display(prio1, 10Hz)
 *        └ 取狀態快照 → spi1_lock → 畫 OLED → spi1_unlock
 *    Console(prio2, 事件驅動)
 *        └ block 在 UART RX 通知（USART IDLE / DMA HT-TC）；醒來 app_cli_poll()
 *    Button(prio2, 事件驅動)
 *        └ block 在 EXTI0 邊緣通知；去抖（延遲後重讀）確認 → toggle start/stop
 *
 *  事件接線：UART/EXTI 的 ISR 在「優先權 6（>= MAX_SYSCALL）」呼叫 ...FromISR()
 *  喚醒對應 task；start() 用 xTaskNotifyGive 喚醒 sensor。狀態（DL_IDLE/LOGGING）
 *  只剩「被查詢」職責（Display/CLI status），不再被任何 task 輪詢。
 *
 *  共用資源與同步：
 *    qSamples : Sensor→Logger 佇列（producer/consumer 解耦，抗 SD stall）
 *    匯流排鎖 : SPI1 互斥（OLED 與 SD/FatFs 共用，SPEC §7.2-1）—— 實作在
 *               bus_port_freertos.c，本檔只呼叫 bus_port_init()/spi1_lock()
 *    （狀態 mutex 在 app_datalogger.c，保護 state/count/filename）
 *
 *  Created on: Jun 20, 2026
 *      Author: xushu
 */

#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bus_port.h"      /* spi1_lock / spi1_unlock / bus_port_init */

#include "systick.h"
#include "driver_spi1.h"
#include "driver_i2c1.h"
#include "driver_usart1.h"
#include "device_ssd1306.h"
#include "device_sdcard.h"
#include "device_mpu6050.h"
#include "device_tempsensor.h"
#include "app_datalogger.h"
#include "app_cli.h"
#include "app_button.h"

/* ---- task 參數 ---- */
#define SENSOR_PERIOD_MS     10          /* 100Hz */
#define DISPLAY_PERIOD_MS    100         /* ~10Hz */
#define LOGGER_SYNC_MS       1000        /* 每秒 f_sync */
#define BTN_DEBOUNCE_MS      20          /* EXTI 喚醒後等接點沉澱，再重讀電位確認 */

#define QSAMPLES_LEN         64          /* 64 × 16B ≈ 640ms 緩衝 */

#define PRIO_SENSOR          4
#define PRIO_LOGGER          3
#define PRIO_CONSOLE         2
#define PRIO_BUTTON          2          /* 與 console 同級；按鈕事件不需搶先 sensor/logger */
#define PRIO_DISPLAY         1
#define PRIO_BOOTSTRAP       (configMAX_PRIORITIES - 1)   /* 先跑完初始化 */

#define STACK_SENSOR         256
#define STACK_LOGGER         1024        /* FatFs + f_printf 吃 stack */
#define STACK_DISPLAY        256
#define STACK_CONSOLE        1024        /* ls/dump/clear 經 dl_* 走 FatFs */
#define STACK_BUTTON         768         /* toggle→start/stop 會走 FatFs；保守值，可用 high-water-mark 量測後縮小 */
#define STACK_BOOTSTRAP      1024        /* f_mount 走 FatFs */

/* ---- 同步原語 ---- */
static QueueHandle_t     s_qSamples;
static volatile uint32_t s_overruns;

/* spi1_lock()/spi1_unlock() 的實作已搬到 bus_port_freertos.c（匯流排鎖屬於
 * bus 自己，見 bus_port.h）；這裡只在 app_tasks_start() 呼叫 bus_port_init()。 */

uint32_t app_tasks_overruns(void) { return s_overruns; }

/* Sensor 每輪更新的最新晶片溫度（×10，整數）。給 Display task 顯示用。
 * 單一 writer(Sensor) / 單一 reader(Display)、32-bit 對齊存取，讀到舊值無害，
 * 不需 mutex。DL_IDLE 時 Sensor 不跑，值停在最後一次量測。 */
static volatile int32_t s_last_temp_c10;

/* ---- task handles（ISR 與 datalogger 的通知目標）---- */
static TaskHandle_t s_sensor;     /* start 成功時被喚醒，進 100Hz 取樣迴圈 */
static TaskHandle_t s_console;    /* UART RX(IDLE / DMA HT-TC) 事件喚醒 */
static TaskHandle_t s_button;     /* EXTI0 邊緣喚醒 */

/* ---- ISR → task 通知 callback（跑在 ISR 脈絡）----
 * 注入給 driver（usart1 / app_button）。承載這些 callback 的 IRQ 優先權已在各自
 * driver 的 init 設為 6（>= configMAX_SYSCALL_INTERRUPT_PRIORITY），故可安全
 * 呼叫 ...FromISR()。woken=pdTRUE 時 portYIELD_FROM_ISR 會在離開 ISR 時直接
 * 切到被喚醒的高優先 task，省掉一次排程延遲。 */
static void on_uart_rx_isr(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_console, &woken);
    portYIELD_FROM_ISR(woken);
}

static void on_button_isr(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_button, &woken);
    portYIELD_FROM_ISR(woken);
}

/* ───────────────── Sensor task ─────────────────
 * 事件驅動：平時 block 在 ulTaskNotifyTake（IDLE 時 0% CPU，0 次取樣）。
 * app_datalogger_start() 成功時 xTaskNotifyGive 把它喚醒 → 進 100Hz 內層迴圈；
 * stop 把狀態翻回 IDLE，內層迴圈下次檢查就退出、回頂端睡死。
 * 不變量：IDLE ⟺ sensor 卡在頂端的 ulTaskNotifyTake，所以 start 的通知一定接得到。 */
static void sensor_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     /* 等 start 通知；睡死 */

        TickType_t next = xTaskGetTickCount();       /* 進迴圈才取基準 → 節奏乾淨 */
        while (app_datalogger_state() == DL_LOGGING) {
            vTaskDelayUntil(&next, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
            if (app_datalogger_state() != DL_LOGGING) {
                break;                               /* stop 落在這次等待中：不取這筆 */
            }

            imu_sample_t s;
            s.t_ms = systick_get_ms();
            i2c1_lock();                             /* I2C1 互斥：CLI 的 i2cscan / 之後的 INA219 也會上這條 bus */
            mpu6050_read_all(&s.acc, &s.gyro);       /* 單次 14-byte burst，鎖住的時間 ~1.5ms */
            i2c1_unlock();
            s.temp_c = tempsensor_read_c();          /* 同一時間點的晶片溫度（ADC，不走 I2C）*/
            s_last_temp_c10 = (int32_t)(s.temp_c * 10.0f);   /* 給 Display 顯示 */

            if (xQueueSend(s_qSamples, &s, 0) != pdTRUE) {
                s_overruns++;                        /* 佇列滿：丟一筆，但不拖慢取樣 */
            }
        }
        /* 迴圈退出 = 已 stop，回頂端 ulTaskNotifyTake 睡死 */
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
            app_datalogger_write_sample(s.t_ms, &s.acc, &s.gyro, s.temp_c);
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

        spi1_lock();
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "IMU LOGGER");
        if (st.state == DL_LOGGING) {
            ssd1306_draw_string(0, 16, "* REC");
            ssd1306_draw_int(0, 32, (int32_t)st.count);
        } else {
            ssd1306_draw_string(0, 16, "READY");
        }
        /* 第 4 行：晶片溫度 "T:35.0C"（整數位 + 一位小數，自組字串）。 */
        int32_t t10 = s_last_temp_c10;
        char    tline[10];
        int     ti = 0;
        tline[ti++] = 'T';
        tline[ti++] = ':';
        if (t10 < 0) { tline[ti++] = '-'; t10 = -t10; }
        int32_t whole = t10 / 10;
        if (whole >= 100) { tline[ti++] = (char)('0' + (whole / 100) % 10); }
        if (whole >= 10)  { tline[ti++] = (char)('0' + (whole / 10)  % 10); }
        tline[ti++] = (char)('0' + whole % 10);
        tline[ti++] = '.';
        tline[ti++] = (char)('0' + t10 % 10);
        tline[ti++] = 'C';
        tline[ti]   = '\0';
        ssd1306_draw_string(0, 48, tline);
        ssd1306_update_screen();            /* 同步更新，鎖內完成 */
        spi1_unlock();

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* ───────────────── Console / Button tasks ───────────────── */
/* start/stop 切換（CLI 與按鈕共用）。OLED 由 Display task 反映，這裡補 UART 回饋。 */
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
        /* 事件驅動：睡到有 UART 事件（USART IDLE 或 DMA RX 半滿/全滿）才醒，
         * 不再每 10ms 輪詢。醒來一次把 ring buffer 抽乾、跑完所有完整指令。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        app_cli_poll();                     /* 內部委派 dl_* 走 FatFs，CLI 不自鎖 */
    }
}

/* ───────────────── Button task ─────────────────
 * 事件驅動 + 去抖。EXTI0 邊緣（on_button_isr）喚醒；平時 block，0% CPU。
 * 去抖放在這裡而非 ISR：等接點沉澱後重讀電位，仍按著才採信 → 同時濾掉按下
 * 與放開的彈跳。toggle_logging 會走 FatFs（start/stop），在 task 脈絡才安全。 */
static void button_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     /* 等 EXTI 邊緣 */
        vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));  /* 讓接點彈跳沉澱 */
        if (app_button_is_down()) {                  /* 沉澱後仍按著才是真的按下 */
            toggle_logging();
        }
        ulTaskNotifyTake(pdTRUE, 0);                 /* 清掉彈跳/放開期間累積的通知 */
    }
}

/* ───────────────── Bootstrap task ─────────────────
 * 初始化集中在這裡：跑在排程器啟動後（delay_ms = vTaskDelay 才有效），
 * 完成後建立五個工作 task、接好通知接線，再把自己刪掉。 */
static void bootstrap_task(void *arg)
{
    (void)arg;

    /* ---- 匯流排 / 周邊初始化 ---- */
    usart1_init(115200);          // UART console（PCLK2=84MHz：BRR=729，誤差 0.02%；CP2102 可穩定跑此速率）
    spi1_init();                  // SPI1（OLED + SD 共用）
    i2c1_init();                  // I2C1（MPU6050）
    sd_cs_init();                 // SD CS = PB10
    ssd1306_init();               // OLED
    tempsensor_init();            // ADC1 內部 ch16：晶片內建溫度感測器
    delay_ms(10);

    usart1_send_string(
        "\r\n"
        "  +--------------------------------------------+\r\n"
        "  |  IMU Datalogger   STM32F407 / FreeRTOS     |\r\n"
        "  |  MPU6050 100Hz -> CSV/SD   OLED   UART CLI  |\r\n"
        "  +--------------------------------------------+\r\n"
        "\r\n");

    /* ---- Application 初始化（MPU6050 + 掛載 FAT32）---- */
    int rc = app_datalogger_init();
    if (rc == -1) {
        usart1_send_string("[FATAL] MPU6050 not found\r\n");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "MPU FAIL");
        ssd1306_update_screen();
        vTaskSuspend(NULL);
    } else if (rc == -2) {
        int mr = app_datalogger_mount_result();   /* FRESULT: 13=非FAT32, 1/3=SPI/卡片 */
        usart1_send_string("[FATAL] FAT32 mount failed, FRESULT=");
        char num[4] = { (char)('0' + (mr / 10) % 10), (char)('0' + mr % 10), '\r', '\n' };
        usart1_send((const uint8_t *)num, 4);
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "FS FAIL");
        ssd1306_draw_string(0, 16, "FRES:");
        ssd1306_draw_int(48, 16, mr);
        ssd1306_draw_string(0, 32, "SDINIT:");      /* 卡在哪一步：1=CMD0 3=ACMD41 */
        ssd1306_draw_int(64, 32, sd_last_init_rc);
        ssd1306_update_screen();
        vTaskSuspend(NULL);
    } else if (rc != 0) {
        usart1_send_string("[FATAL] init error\r\n");
        vTaskSuspend(NULL);
    }
    usart1_send_string("  [ok] sensors + FAT32 ready\r\n\r\n");

    app_cli_init();
    app_button_init();            // PA0 user button → EXTI0（rising edge）：按一下切換 start/stop

    /* ---- 建立工作 task（接出 handle 給 ISR / datalogger 當通知目標）---- */
    xTaskCreate(sensor_task,  "sensor",  STACK_SENSOR,  NULL, PRIO_SENSOR,  &s_sensor);
    xTaskCreate(logger_task,  "logger",  STACK_LOGGER,  NULL, PRIO_LOGGER,  NULL);
    xTaskCreate(display_task, "display", STACK_DISPLAY, NULL, PRIO_DISPLAY, NULL);
    xTaskCreate(console_task, "console", STACK_CONSOLE, NULL, PRIO_CONSOLE, &s_console);
    xTaskCreate(button_task,  "button",  STACK_BUTTON,  NULL, PRIO_BUTTON,  &s_button);

    /* ---- 接上 ISR / 狀態機的通知接線（必須在 task 建立後）----
     * bootstrap 跑在最高優先權，會一路執行到 vTaskDelete 才讓出 CPU；因此在這裡
     * 完成註冊前，console/button/sensor 都還沒機會跑、也沒有人能觸發 start，
     * 不會發生「通知一個還沒接好的目標」。註冊前若有 UART/EXTI 中斷觸發，
     * driver 內的 callback 指標還是 NULL，只會清旗標/pending，安全。 */
    usart1_set_rx_callback(on_uart_rx_isr);   // UART RX 事件（IDLE / DMA HT-TC）→ 喚醒 console
    app_button_set_callback(on_button_isr);   // EXTI0 邊緣 → 喚醒 button
    app_datalogger_set_producer(s_sensor);    // start 成功 → 喚醒 sensor 進取樣迴圈

    vTaskDelete(NULL);            // 初始化完成，bootstrap 退場
}

void app_tasks_start(void)
{
    s_qSamples = xQueueCreate(QSAMPLES_LEN, sizeof(imu_sample_t));
    int bus_rc = bus_port_init();   /* 獨立敘述：configASSERT 被停用時仍會建立 mutex */
    configASSERT(s_qSamples != NULL && bus_rc == 0);

    xTaskCreate(bootstrap_task, "boot", STACK_BOOTSTRAP, NULL, PRIO_BOOTSTRAP, NULL);
}