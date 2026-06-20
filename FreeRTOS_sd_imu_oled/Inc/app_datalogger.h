/*
 * app_datalogger.h
 *
 *  IMU 資料記錄器 — Application 層核心（FreeRTOS 版）。
 *  狀態機 IDLE/LOGGING；「取樣」與「寫入」已拆開：
 *    - Sensor task 讀 MPU6050 → 丟 queue
 *    - Logger task 從 queue 取 → app_datalogger_write_sample() 寫 CSV/SD
 *  本模組負責 FIL/狀態的擁有權與鎖：
 *    - 所有 FatFs/SD 操作包在 bus_lock()（SPI1 匯流排互斥，見 app_tasks.h）
 *    - 狀態快照（state/count/filename）由自有的 mState 保護
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#ifndef APP_DATALOGGER_H_
#define APP_DATALOGGER_H_

#include <stdint.h>
#include "device_mpu6050.h"

typedef enum {
    DL_IDLE = 0,   /* 待命，沒在記錄 */
    DL_LOGGING     /* 記錄中 */
} dl_state_t;

/* 狀態快照：給 Display / CLI status 一次取走，避免讀到半更新的欄位。 */
typedef struct {
    dl_state_t state;
    uint32_t   count;        /* 本次 session 已記錄筆數 */
    char       filename[16]; /* 目前/上一個檔名 */
} dl_status_t;

/* 初始化：建立狀態 mutex、初始化 MPU6050 + 掛載 FAT32。
 * 回傳 0=OK / -1=MPU6050 沒找到 / -2=掛載失敗。
 * 注意：I2C1/SPI1/SD CS 等匯流排要先初始化好；需在排程器啟動後呼叫。 */
int app_datalogger_init(void);

/* 開新檔 log_NNN.csv、寫表頭、進 LOGGING。回傳 0=OK / <0=失敗。 */
int app_datalogger_start(void);

/* f_sync + f_close、回 IDLE。回傳 0。 */
int app_datalogger_stop(void);

/* 寫一筆樣本（Logger task 呼叫）。LOGGING 時才真的寫；回傳 FatFs rc（0=OK）。 */
int app_datalogger_write_sample(uint32_t t_ms,
                                const mpu6050_axes_t *acc,
                                const mpu6050_axes_t *gyro);

/* 週期性把 FAT/目錄落地（Logger task 每秒呼叫）。LOGGING 時才做。 */
void app_datalogger_sync(void);

/* 一次取走狀態快照（mState 保護）。 */
void app_datalogger_get_status(dl_status_t *out);

dl_state_t  app_datalogger_state(void);   /* 便捷讀取（內部取 mState）*/
uint32_t    app_datalogger_count(void);
/* 目前/上一個檔名。只有 Console（start）會寫，Console 自己讀安全；
 * 跨 task 讀請改用 app_datalogger_get_status()。 */
const char *app_datalogger_filename(void);

#endif /* APP_DATALOGGER_H_ */
