/*
 * app_i2c1_supervisor.h
 *
 *  I2C1 匯流排管理員：何時動匯流排、何時動晶片，集中在這裡決定。
 *  ─────────────────────────────────────────────
 *  以前 MPU6050（Sensor task）和 INA219（Display task）各寫一套：連續失敗
 *  N 次就 bus recovery + 重新 init 自己這顆，不看錯誤種類；mpu6050_init()
 *  裡面還會自己 bus recovery。結果拔掉 MPU（全是 NACK）也每 50ms 搖一次
 *  匯流排，匯流排卡住時兩個 task 可能前後各搖一次。
 *
 *  現在依錯誤種類分兩層，各自只處理自己那一種：
 *
 *    NACK              → 晶片層（每顆一份，同一套規則）
 *                        連續 I2C1_SUP_NACK_LIMIT 次 → 離線，不再讀它
 *                        離線時每 probe_ms 用 init 重探，過了就回線上
 *                        永遠不動匯流排
 *
 *    TIMEOUT/BUS/SCL   → 匯流排層（只有一份，所有晶片的錯誤都算進來）
 *                        連續 I2C1_SUP_BUS_ERR_LIMIT 次 → bus recovery（限速）
 *                        之後每顆晶片重新 init（熱插拔回來的是 power-on 預設值）
 *
 *  兩層放在同一個模組是刻意的：bus recovery 之後要重新 init 這條匯流排上的
 *  每顆晶片，晶片的重探又可能撞到匯流排錯誤，兩層互相牽動，拆開只會變成
 *  兩個檔案互相呼叫。真正動手的程式不在這裡 —— 搖 clock 是 driver_i2c1 的
 *  i2c1_bus_recovery()，重新設定是各晶片 driver 的 init；這裡只決定時機。
 *
 *  本模組不認得任何一顆晶片：清單由 app_tasks.c（知道板子接了什麼的地方）
 *  在開機時用 i2c1_sup_register() 登記。加一顆晶片只要多登記一行。
 *
 *  在線偵測也統一：一顆晶片超過 probe_ms 沒有成功交易（離線，或在線但沒人
 *  在讀它，例如 IDLE 時的 MPU）就用 init 探一次。所以讀取端只要「在線才讀、
 *  讀完回報」，重探由 Display 每輪呼叫 i2c1_sup_tick() 推動。
 *
 *  鎖：report / probe 的呼叫端必須持有 i2c1_lock（它們本來就剛做完交易）；
 *  tick 只在真的要探時自己拿鎖。online 與統計可以不上鎖讀。
 *
 *  Created on: Oct 27, 2026
 *      Author: xushu
 */

#ifndef APP_I2C1_SUPERVISOR_H_
#define APP_I2C1_SUPERVISOR_H_

#include <stdint.h>

/* 最多登記幾顆晶片。I2C1 上目前是 MPU6050 + INA219。 */
#define I2C1_SUP_MAX_DEVS   4

/* 登記時拿到的編號。-1 = 沒登記成功（表滿），其餘函式收到 -1 一律當成離線。 */
typedef int i2c1_dev_t;

/* 登記一顆晶片。只在建立工作 task 之前呼叫（bootstrap），之後清單固定，
 * 所以讀清單的地方不必上鎖。
 *   name      給 CLI status 印
 *   init      0 = OK，其餘 i2c_status_t；兼在線偵測，必須可以重複呼叫
 *   probe_ms  超過這麼久沒有成功交易就用 init 探一次
 * 登記後先當離線，用 i2c1_sup_probe() 給初值。 */
i2c1_dev_t i2c1_sup_register(const char *name, int (*init)(void), uint32_t probe_ms);

/* 回報一次讀取的最終結果（呼叫端自己的重試做完之後）。
 * st = 裝置函式的回傳值：0 = OK，其餘是 i2c_status_t。持有 i2c1_lock。 */
void i2c1_sup_report(i2c1_dev_t dev, int st);

/* 立刻用 init 探一次（失敗重試一次再下結論），更新在線狀態。
 * 回傳 1 = 在線。持有 i2c1_lock。 */
int i2c1_sup_probe(i2c1_dev_t dev);

/* Display 每輪呼叫：該探的晶片（見檔頭）探一次。自己拿 i2c1_lock。 */
void i2c1_sup_tick(void);

/* 在線與否（1/0）。不上鎖讀，讀到舊值只是晚一輪才停 / 才恢復。 */
int i2c1_sup_online(i2c1_dev_t dev);

/* ---- 給 CLI status ---- */
typedef struct {
    uint32_t recoveries;        /* bus recovery 次數 */
    uint8_t  last_rc;           /* 最後一次 recovery 的結果（i2c_status_t）*/
} i2c1_sup_bus_stats_t;

typedef struct {
    const char *name;
    uint8_t     online;
    uint32_t    probe_ms;       /* 重探週期 */
    uint32_t    went_offline;   /* 掉線次數（累計）*/
} i2c1_sup_dev_stats_t;

int  i2c1_sup_dev_count(void);  /* 已登記幾顆；編號是 0 .. count-1 */
void i2c1_sup_bus_stats(i2c1_sup_bus_stats_t *out);
void i2c1_sup_dev_stats(i2c1_dev_t dev, i2c1_sup_dev_stats_t *out);

#endif /* APP_I2C1_SUPERVISOR_H_ */
