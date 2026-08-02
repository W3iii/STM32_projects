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
 *  bus_lock()/bus_unlock() 已移到 bus_port.h（RTOS 中立的 porting 介面，
 *  OLED 與 SD/FatFs 共用同一支鎖）。
 *
 *  Created on: Jun 20, 2026
 *      Author: xushu
 */

#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include <stdint.h>
#include "device_mpu6050.h"

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

#endif /* APP_TASKS_H_ */
