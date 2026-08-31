/*
 * app_tasks.h
 *
 *  FreeRTOS task 與同步原語的家（SPEC §7.1 的四 task 架構）。
 *  ─────────────────────────────────────────────
 *    Sensor(prio4,100Hz) --qSamples--> Logger(prio3) 寫 CSV/SD
 *    Display(prio1,10Hz)  畫 OLED
 *    Console(prio2,~10ms) UART CLI + 按鈕
 *
 *  對外只暴露啟動進入點 app_tasks_start()。共用的 SPI1 匯流排鎖
 *  spi1_lock()/spi1_unlock() 已移到 bus_port.h（RTOS 中立的 porting 介面，
 *  OLED 與 SD/FatFs 共用同一支鎖）。
 *
 *  Created on: Jun 20, 2026
 *      Author: xushu
 */

#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include <stdint.h>
#include "device_mpu6050.h"
#include "device_ina219.h"

/* Sensor → Logger 佇列的元素：一筆 IMU 樣本（含時間戳 + 晶片溫度）。 */
typedef struct {
    uint32_t       t_ms;
    mpu6050_axes_t acc;
    mpu6050_axes_t gyro;
    float          temp_c;     /* 晶片內建溫度感測器（°C），與同時間點 IMU 一起記錄 */
} imu_sample_t;

/* 建立 queue/mutex 與 bootstrap task，再由 caller 啟動排程器。
 * 在 main() 內、vTaskStartScheduler() 之前呼叫。 */
void app_tasks_start(void);

/* Sensor 因佇列滿而丟棄的樣本數（理想為 0；給 CLI status 觀察用）。 */
uint32_t app_tasks_overruns(void);

/* Sensor 因連續 I2C 讀取失敗而觸發 bus recovery 的累計次數（理想為 0）。
 * 非 0 代表 I2C 匯流排曾經卡住 —— 通常是供電、接觸或拉電阻的問題。 */
uint32_t app_tasks_i2c_recovers(void);

/* IDLE task 要不要用 WFI 睡覺（1 = 睡，0 = 空轉）。CLI 的 wfi 指令直接改它，
 * 實作與完整說明在 app_tasks.c 的 vApplicationIdleHook。 */
extern volatile uint8_t g_wfi_enabled;

/* 最近一次 INA219 量測的快照（由 Display task 以 10Hz 更新）。
 * INA219 不獨立開 task：它只是「被顯示 / 被查詢」的慢速資料，掛在既有的
 * Display task 上量就好，省一個 task 的 stack 與排程成本。
 *
 * out 可為 NULL（只想問在不在線時）。
 * 回傳：1 = 資料有效，0 = INA219 不在線或還沒量到（out 不會被寫）。 */
int app_tasks_get_power(ina219_data_t *out);

/* 最近 10 筆量測（10Hz → 約 1 秒）的平均，四個欄位各自平均。
 * 單筆讀數偶爾會有離群值（I2C 雜訊、剛好撞上 SD/OLED 的湧浪電流），
 * 要「看系統平均耗電」時用這支，CLI 的 power 指令就是走這條。
 *
 * out 可為 NULL；n_used 可為 NULL，非 NULL 時回填實際平均了幾筆
 * （開機後前 1 秒還沒累積滿，會少於 10）。
 * 回傳：1 = 有效，0 = INA219 不在線或最新一次讀取失敗（out/n_used 不會被寫）。*/
int app_tasks_get_power_avg(ina219_data_t *out, uint32_t *n_used);

#endif /* APP_TASKS_H_ */
