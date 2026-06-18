/*
 * app_datalogger.h
 *
 *  IMU 資料記錄器 — Application 層核心。
 *  狀態機 IDLE/LOGGING + 100Hz 取樣 MPU6050 → CSV(FatFs) → SD。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#ifndef APP_DATALOGGER_H_
#define APP_DATALOGGER_H_

#include <stdint.h>

typedef enum {
    DL_IDLE = 0,   /* 待命，沒在記錄 */
    DL_LOGGING     /* 記錄中 */
} dl_state_t;

/* 初始化：初始化 MPU6050 + 掛載 FAT32。
 * 回傳 0=OK / -1=MPU6050 沒找到 / -2=掛載失敗。
 * 注意：I2C1/SPI1/SD CS 等匯流排要先在 main 初始化好。 */
int app_datalogger_init(void);

/* 開新檔 log_NNN.csv、寫表頭、進 LOGGING。回傳 0=OK / <0=失敗。 */
int app_datalogger_start(void);

/* f_sync + f_close、回 IDLE。回傳 0。 */
int app_datalogger_stop(void);

/* 主迴圈每圈呼叫；LOGGING 時依時間取樣 + 週期性 f_sync。 */
void app_datalogger_tick(uint32_t now_ms);

dl_state_t  app_datalogger_state(void);
uint32_t    app_datalogger_count(void);     /* 本次 session 已記錄筆數 */
const char *app_datalogger_filename(void);  /* 目前/上一個檔名 */

#endif /* APP_DATALOGGER_H_ */
