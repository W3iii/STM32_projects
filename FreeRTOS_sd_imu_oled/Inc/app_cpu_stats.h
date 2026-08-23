/*
 * app_cpu_stats.h
 *
 *  自建的 CPU 佔用統計（取代直接用 uxTaskGetSystemState 算百分比）。
 *  ─────────────────────────────────────────────
 *  為什麼不用 kernel 內建的 run-time stats？兩個結構性缺陷：
 *
 *    1) ISR 時間被算到「剛好被打斷的那支 task」頭上。FreeRTOS 只在
 *       context switch 時結算（tasks.c vTaskSwitchContext），所以 100Hz
 *       取樣、UART DMA、EXTI 這些 ISR 的成本是隱形的，而且亂噴。
 *
 *    2) 當下執行中那支 task 的「未結算切片」永遠漏掉：
 *       uxTaskGetSystemState 當場讀計時器當分母，但各 task 的累加值停在
 *       上一次切換，導致 Σ(各 task %) < 100%，而且呼叫者系統性低報自己。
 *
 *  本模組直接掛 traceTASK_SWITCHED_IN/OUT 與 ISR 進出點，自己維護一張
 *  累加表，把時間拆成三類：各 task / [ISR] / [SCHED]（排程器本身的開銷），
 *  三者相加 = 牆鐘時間。計時源共用 app_runtime_stats.c 設定的 TIM2 @100kHz。
 *
 *  另外提供 10Hz 取樣器（掛在既有的 Display task 上，不新增 task）：
 *  保留短期歷史與歷史峰值。這是為了抓 logger 每秒 f_sync 造成的尖峰——
 *  SD 卡 FAT 更新遇到 block erase 可以停上百毫秒，任何秒級平均都會把它抹平。
 *
 *  Created on: Aug 20, 2026
 *      Author: xushu
 */

#ifndef APP_CPU_STATS_H_
#define APP_CPU_STATS_H_

#include <stdint.h>

/* slot 配置：前 CPUSTATS_TASK_SLOTS 個給 task，後面兩個是固定的統計桶。 */
#define CPUSTATS_TASK_SLOTS   12
#define CPUSTATS_SLOT_ISR     (CPUSTATS_TASK_SLOTS)       /* 所有 ISR 的總和 */
#define CPUSTATS_SLOT_SCHED   (CPUSTATS_TASK_SLOTS + 1)   /* vTaskSwitchContext 本身 */
#define CPUSTATS_SLOTS        (CPUSTATS_TASK_SLOTS + 2)

/* 歷史深度：64 × 100ms = 6.4 秒。挑 64 是為了讓 `tasks hist` 的
 * 「名字欄 + 每格一個字元」剛好塞得進 80 欄終端機。 */
#define CPUSTATS_HIST_LEN     64

/* 取樣週期，需與 Display task 的迴圈週期一致（app_tasks.c DISPLAY_PERIOD_MS）。 */
#define CPUSTATS_SAMPLE_MS    100

typedef struct {
    const char *name;        /* task 名稱，或 "[ISR]" / "[SCHED]" */
    void       *handle;      /* TaskHandle_t；ISR/SCHED 兩個 slot 為 NULL */
    uint8_t     slot;        /* 本列對應的 slot 索引，餵給 cpustats_hist() */
    uint16_t    cur_pm;      /* 最近 1 秒的平均佔用（千分比，0..1000）*/
    uint16_t    peak_pm;     /* 單一 100ms 取樣的歷史最大值（千分比）*/
    uint16_t    avg_pm;      /* 開機至今的平均佔用（千分比）*/
} cpustats_slot_t;

/* ---- 給 Display task 呼叫（task 脈絡）---- */

/* 取一次樣：計算距上次取樣的區間佔用，寫進歷史環形緩衝並更新峰值。
 * 每 CPUSTATS_SAMPLE_MS 呼叫一次。 */
void cpustats_sample(void);

/* ---- 給 CLI 呼叫（task 脈絡）---- */

/* 填入目前所有 slot 的統計；回傳實際填入筆數（含 ISR/SCHED）。 */
uint32_t cpustats_get(cpustats_slot_t *out, uint32_t max);

/* 取某個 slot 的歷史（千分比，由舊到新）；回傳實際筆數。 */
uint32_t cpustats_hist(uint32_t slot, uint16_t *out, uint32_t max);

/* 清掉峰值與歷史（重新開始觀察一段負載）。 */
void cpustats_reset(void);

/* ---- hot path hook：由 FreeRTOSConfig.h 的 trace macro 呼叫 ----
 * 跑在 PendSV 脈絡，不可呼叫任何會阻塞的東西。
 *
 * ISR 那一對掛勾不在這裡：driver 層不該認得本模組，所以它們走 isr_port.h
 * 那支中性介面（本檔的 .c 提供實作），跟 bus_port.h 同一個模式。 */
void cpustats_switched_in(void *tcb);
void cpustats_switched_out(void);

#endif /* APP_CPU_STATS_H_ */
