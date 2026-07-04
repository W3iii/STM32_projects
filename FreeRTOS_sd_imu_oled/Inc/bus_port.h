/*
 * bus_port.h
 *
 *  SPI1 匯流排互斥的 porting 介面 —— RTOS 中立。
 *  ─────────────────────────────────────────────
 *  仿 FatFs ff_mutex_* 的風格：這裡只宣告「具名函式」，不 include 任何
 *  RTOS 標頭。實作由各專案自己的 porting 檔提供（link-time 綁定）：
 *    - FreeRTOS 版：bus_port_freertos.c，用一支 mutex 實作。
 *    - bare-metal 版：可寫成 no-op（單執行緒）或關中斷（若 ISR 也上匯流排）。
 *
 *  這樣底層 bus driver（SD / OLED）與上層模組（app_datalogger 等）都只需
 *  #include "bus_port.h" 並呼叫 bus_lock()/bus_unlock()，不必碰 semphr.h，
 *  即維持可移植（bare-metal 與 FreeRTOS 都能編）。
 */

#ifndef BUS_PORT_H_
#define BUS_PORT_H_

/* 建立底層同步原語。在使用任何 bus_lock() 之前呼叫一次（通常在啟動排程器前）。
 * 回傳 0 成功、非 0 失敗。bare-metal no-op 版直接回 0。 */
int  bus_port_init(void);

/* SPI1 匯流排互斥鎖：包住「整個」SPI1 transaction（OLED 整次更新，或一個
 * FatFs 操作）。app_datalogger.c / app_cli.c / Display task 共用。non-recursive，
 * 不可巢狀對自己再 lock。 */
void bus_lock(void);
void bus_unlock(void);

#endif /* BUS_PORT_H_ */
