/*
 * app_datalogger.h
 *
 *  IMU 資料記錄器 — Application 層核心（FreeRTOS 版）。
 *  狀態機 IDLE/LOGGING；「取樣」與「寫入」已拆開：
 *    - Sensor task 讀 MPU6050 → 丟 queue
 *    - Logger task 從 queue 取 → app_datalogger_write_sample() 寫 CSV/SD
 *  本模組負責 FIL/狀態的擁有權與鎖：
 *    - 所有 FatFs/SD 操作包在 spi1_lock()（SPI1 匯流排互斥，見 bus_port.h）
 *    - 狀態快照（state/count/filename）由自有的 mState 保護
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#ifndef APP_DATALOGGER_H_
#define APP_DATALOGGER_H_

#include <stdint.h>
#include "device_mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"       /* TaskHandle_t（sensor 喚醒用）*/

typedef enum {
    DL_IDLE = 0,   /* 待命，沒在記錄 */
    DL_LOGGING,    /* 記錄中 */
    DL_ERROR       /* 寫入失敗，本次 session 已作廢（見下方說明）*/
} dl_state_t;

/* 為什麼要有 DL_ERROR：FatFs 在任何一次寫入失敗時會把錯誤鎖進
 * fp->err（ff.c 的 ABORT 巨集），之後每一次 f_write 都在函式開頭直接
 * 返回、連 SD 卡都不碰。原本的程式碼只是「成功才 count++」，失敗時
 * 什麼都不做，於是 state 停在 LOGGING、CLI 說在錄、OLED 有畫面、
 * CPU 用量完全正常，但一筆都沒寫進去 —— 實際踩過一次（供電不穩導致
 * SD 寫入逾時）。既然錯誤是鎖住的，第一次失敗就等於整個 session 作廢，
 * 所以直接翻到 DL_ERROR：Sensor 會停止取樣、Logger 不再嘗試寫入，
 * 使用者從 status / OLED 立刻看得到。 */

/* 狀態快照：給 Display / CLI status 一次取走，避免讀到半更新的欄位。 */
typedef struct {
    dl_state_t state;
    uint32_t   count;        /* 本次 session 已記錄筆數 */
    char       filename[16]; /* 目前/上一個檔名 */
    uint8_t    fres;         /* 失敗時的 FatFs FRESULT（fp->err）；0 = 沒錯 */
    uint32_t   err_count;    /* 累計寫入失敗次數（跨 session 不清）*/
} dl_status_t;

/* 初始化：建立狀態 mutex、初始化 MPU6050 + 掛載 FAT32。
 * 回傳 0=OK / -1=MPU6050 沒找到 / -2=掛載失敗。
 * 注意：I2C1/SPI1/SD CS 等匯流排要先初始化好；需在排程器啟動後呼叫。 */
int app_datalogger_init(void);

/* 診斷：回傳最後一次 f_mount 的 FRESULT（掛載失敗時看原因）。
 *   1=FR_DISK_ERR / 3=FR_NOT_READY → SPI/卡片；13=FR_NO_FILESYSTEM → 不是 FAT32。 */
int app_datalogger_mount_result(void);

/* 開新檔 log_NNN.csv、寫表頭、進 LOGGING。回傳 0=OK / <0=失敗。 */
int app_datalogger_start(void);

/* f_sync + f_close、回 IDLE。回傳 0。 */
int app_datalogger_stop(void);

/* 寫一筆樣本（Logger task 呼叫）。LOGGING 時才真的寫；回傳 FatFs rc（0=OK）。
 * temp_c：同一時間點的晶片溫度（°C），與 IMU 一起寫進 CSV 最後一欄。 */
int app_datalogger_write_sample(uint32_t t_ms,
                                const mpu6050_axes_t *acc,
                                const mpu6050_axes_t *gyro,
                                float temp_c);

/* 週期性把 FAT/目錄落地（Logger task 每秒呼叫）。LOGGING 時才做。 */
void app_datalogger_sync(void);

/* 一次取走狀態快照（mState 保護）。 */
void app_datalogger_get_status(dl_status_t *out);

dl_state_t  app_datalogger_state(void);   /* 便捷讀取（內部取 mState）*/
uint32_t    app_datalogger_count(void);
/* 目前/上一個檔名。只有 Console（start）會寫，Console 自己讀安全；
 * 跨 task 讀請改用 app_datalogger_get_status()。 */
const char *app_datalogger_filename(void);

/* ── sensor 喚醒注入 ──
 * 把 sensor task 的 handle 交給本模組。app_datalogger_start() 成功從 IDLE
 * 轉成 LOGGING 時，會用它 xTaskNotifyGive() 喚醒 sensor 進取樣迴圈；
 * sensor 平時 block 在 ulTaskNotifyTake()，不再輪詢狀態旗標。
 * stop 不需通知：sensor 內層迴圈下次檢查狀態就自行退出。 */
void app_datalogger_set_producer(TaskHandle_t producer);

/* ── 檔案層操作（給 CLI / console 用）──
 * 把所有 FatFs/SD 存取與 spi1_lock 收進本模組，CLI 不再直接碰 ff.h / spi1_lock。
 *
 * 重要：emit 類 callback 會在「本模組持有 spi1_lock 期間」被回呼。callback 內
 *       只能做 UART（usart1_*）等「不在 SPI bus 上」的事，絕對不可再碰 SD 或
 *       OLED（共用同一支 spi1_lock）→ 會重入死鎖。
 */

/* 列出根目錄所有檔案，每個檔呼叫一次 emit(name,size)；free_kb_out 取回剩餘 KB
 * （傳 NULL 可不取）。回傳檔案數 / <0=opendir 失敗。 */
int dl_list(void (*emit)(const char *name, uint32_t size), uint32_t *free_kb_out);

/* 把 name 這個檔讀回來，每一行呼叫一次 emit_line(line)。回傳 0=OK / <0=open 失敗。
 * 注意：記錄中的 session 檔正開著寫，dump 它請先 stop（呼叫端自行判斷）。 */
int dl_dump(const char *name, void (*emit_line)(const char *line));

/* 刪掉所有 log_NNN.csv（NNN=000..999）。deleted_out 取回刪除數（可 NULL）。回傳 0。 */
int dl_clear(uint32_t *deleted_out);

#endif /* APP_DATALOGGER_H_ */