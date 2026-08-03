/*
 * bus_port.h
 *
 *  匯流排互斥的 porting 介面 —— RTOS 中立。
 *  ─────────────────────────────────────────────
 *  仿 FatFs ff_mutex_* 的風格：這裡只宣告「具名函式」，不 include 任何
 *  RTOS 標頭。實作由各專案自己的 porting 檔提供（link-time 綁定）：
 *    - FreeRTOS 版：bus_port_freertos.c，每條匯流排一支 mutex。
 *    - bare-metal 版：可寫成 no-op（單執行緒）或關中斷（若 ISR 也上匯流排）。
 *
 *  這樣底層 bus driver（SD / OLED / MPU6050 / INA219）與上層模組
 *  （app_datalogger 等）都只需 #include "bus_port.h" 並呼叫 xxx_lock()/
 *  xxx_unlock()，不必碰 semphr.h，即維持可移植（bare-metal 與 FreeRTOS 都能編）。
 *
 *  兩條匯流排各有獨立的鎖，名字直接寫明鎖的是誰 —— 兩者互不阻擋
 *  （Sensor 讀 I2C 時 Logger 照樣能寫 SD）：
 *
 *    SPI1  OLED(SSD1306) + SD/FatFs
 *    I2C1  MPU6050 + INA219
 */

#ifndef BUS_PORT_H_
#define BUS_PORT_H_

/* 建立底層同步原語（兩支鎖一起）。在使用任何 lock 之前呼叫一次
 * （通常在啟動排程器前）。回傳 0 成功、非 0 失敗。bare-metal no-op 版直接回 0。 */
int  bus_port_init(void);

/* SPI1 匯流排互斥鎖：包住「整個」SPI1 transaction（OLED 整次更新，或一個
 * FatFs 操作）。app_datalogger.c / Display task 共用。non-recursive，
 * 不可巢狀對自己再 lock。 */
void spi1_lock(void);
void spi1_unlock(void);

/* I2C1 匯流排互斥鎖：包住「整個」I2C1 transaction 序列。單筆 read_regs /
 * write_reg 固然要包，但「寫 reg 再讀回來」這種多筆組合更是要整段包起來，
 * 中間被別的 task 插進一次 START 就會讀到別人的資料。
 * Sensor task（MPU6050）/ CLI（i2cscan）/ 之後的 INA219 共用。non-recursive。 */
void i2c1_lock(void);
void i2c1_unlock(void);

#endif /* BUS_PORT_H_ */
