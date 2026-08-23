/*
 * isr_port_config.h
 *
 *  本專案要怎麼實作 ISR 計時掛勾 —— 對應 FreeRTOSConfig.h 之於 FreeRTOS.h。
 *  ─────────────────────────────────────────────
 *  isr_port.h 會用 __has_include 找這支檔案；找到就用這裡的定義，
 *  找不到（例如 bare-metal 的 exp13）巨集就維持空的，那幾行呼叫整個消失。
 *
 *  這一層的存在意義：driver 只認 isr_port.h 的巨集名字，把「掛到誰身上」
 *  的決定推到專案層。要換成別的統計實作、或臨時關掉 ISR 記帳（把下面兩行
 *  改成 ((void)0)），都不必動 driver 一個字。
 *
 *  Created on: Aug 20, 2026
 *      Author: xushu
 */

#ifndef ISR_PORT_CONFIG_H_
#define ISR_PORT_CONFIG_H_

/* 實作在 app_cpu_stats.c。這裡只宣告，不 include app_cpu_stats.h ——
 * driver 是透過 isr_port.h 間接引進本檔的，不該因此看到統計模組的型別。 */
void cpustats_isr_enter(void);
void cpustats_isr_exit(void);

#define ISR_PORT_ENTER()    cpustats_isr_enter()
#define ISR_PORT_EXIT()     cpustats_isr_exit()

#endif /* ISR_PORT_CONFIG_H_ */
