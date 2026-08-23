/*
 * app_cpu_stats.c
 *
 *  CPU 佔用統計的實作。設計理由見 app_cpu_stats.h 檔頭。
 *  ─────────────────────────────────────────────
 *  時間怎麼被拆開（TIM2 @100kHz，計時源由 app_runtime_stats.c 設定）：
 *
 *      SWITCHED_IN(t0) ────── task 執行 ────── SWITCHED_OUT(t1)
 *          └ 其間若有 ISR，時間記進 s_isr_in_slice，結算時從 task 扣掉
 *
 *      SWITCHED_OUT(t1) ── vTaskSwitchContext 選 task ── SWITCHED_IN(t2)
 *          └ (t2 - t1) 記進 [SCHED]
 *
 *  所以 Σ(各 task) + [ISR] + [SCHED] = 牆鐘時間，百分比會加總到 100%。
 *
 *  兩個已知的量測邊界（刻意接受，換取零額外 task 與極短的 hot path）：
 *    - PendSV 的組語 context save/restore 沒被括起來（它在 trace hook 之外），
 *      那幾十個 cycle 會落在 task 的切片裡。
 *    - ISR 巢狀只認最外層那一次進出（見 cpustats_isr_enter 的註解）。
 *
 *  Created on: Aug 20, 2026
 *      Author: xushu
 */

#include <string.h>

#include "stm32f407xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_cpu_stats.h"
#include "isr_port_config.h" /* 本檔提供 cpustats_isr_enter/exit 的實作 */

#define CPUSTATS_NAME_LEN   configMAX_TASK_NAME_LEN

/* 單一 ISR 的合理上限（TIM2 @100kHz → 10000 = 100ms）。超過就當成計數器
 * 狀態被打亂，整筆丟棄：累加器是單調遞增的，混進一筆垃圾會永久污染。 */
#define CPUSTATS_ISR_MAX    10000U

/* 直接讀 TIM2->CNT。不呼叫 ulGetRunTimeCounterValue()：這條路徑每次
 * context switch 都會走，省掉一次函式呼叫。TIM2 的設定歸 app_runtime_stats.c 管。 */
static inline uint32_t cpustats_cnt(void)
{
    return TIM2->CNT;
}

/* ───────────── hot path 狀態（PendSV / ISR 脈絡更新）─────────────
 * 這些只在「中斷被遮蔽」的情境下被改：trace hook 跑在 vTaskSwitchContext
 * 內（basepri 已擋掉 prio >= 5 的 IRQ，本專案的 app IRQ 都是 6），
 * ISR hook 跑在同優先權的 ISR 裡（彼此不會互相搶）。故不需額外上鎖。 */
static volatile uint32_t s_acc[CPUSTATS_SLOTS];          /* 各 slot 累計 counter */
static void    *volatile s_handle[CPUSTATS_TASK_SLOTS];
static char              s_name[CPUSTATS_TASK_SLOTS][CPUSTATS_NAME_LEN];
static volatile uint32_t s_ntasks;

static volatile uint32_t s_cur;          /* 目前執行中 task 的 slot */
static volatile uint8_t  s_cur_valid;    /* 0 = 還不知道（BSS 歸零即為初始值）*/
static volatile uint32_t s_t_in;         /* 最近一次 SWITCHED_IN 的 TIM2 值 */
static volatile uint32_t s_t_out;        /* 最近一次 SWITCHED_OUT 的 TIM2 值 */
static volatile uint8_t  s_have_out;
static volatile uint32_t s_t_start;      /* 第一次 SWITCHED_IN（= 統計起點）*/
static volatile uint8_t  s_started;

static volatile uint32_t s_isr_in_slice; /* 本切片內累積的 ISR 時間 */
static volatile uint8_t  s_isr_active;
static volatile uint32_t s_isr_t0;

/* ───────────── 取樣器狀態（只在 task 脈絡碰）─────────────
 * Display(prio1) 寫、Console(prio2) 讀，後者會搶前者，故用
 * vTaskSuspendAll() 互斥——不必遮中斷，比 critical section 溫和。 */
static uint32_t s_prev[CPUSTATS_SLOTS];
static uint32_t s_prev_t;
static uint8_t  s_prev_valid;
static uint16_t s_hist[CPUSTATS_SLOTS][CPUSTATS_HIST_LEN];   /* 千分比，環形 */
static uint32_t s_hist_head;             /* 下一個要寫的位置 */
static uint32_t s_hist_count;
static uint16_t s_peak[CPUSTATS_SLOTS];

/* ═════════════════════ hot path hooks ═════════════════════ */

void cpustats_switched_out(void)
{
    uint32_t now = cpustats_cnt();

    if (s_cur_valid) {
        /* 減法用 uint32 模運算，TIM2 繞回（約 11.9 小時）自動正確。 */
        uint32_t slice = now - s_t_in;
        uint32_t isr   = s_isr_in_slice;
        if (isr > slice) {
            isr = slice;              /* 理論上不會發生；夾住免得下溢 */
        }
        s_acc[s_cur] += slice - isr;
    }

    /* 把「未結算區間的起點」推到現在。正常流程下 switched_in 馬上會再覆寫它，
     * 這行只是維持 s_t_in 的不變式（= 尚未計入 s_acc 的那段從哪開始），
     * 免得日後有人在 OUT→IN 之間取快照時把同一段時間算兩次。 */
    s_t_in     = now;
    s_isr_in_slice = 0;

    s_t_out    = now;
    s_have_out = 1;
}

void cpustats_switched_in(void *tcb)
{
    uint32_t now = cpustats_cnt();
    uint32_t idx = CPUSTATS_TASK_SLOTS;   /* = 沒找到 */

    if (!s_started) {
        s_started = 1;
        s_t_start = now;                  /* 統計起點：排程器啟動的那一刻 */
    } else if (s_have_out) {
        /* OUT → IN 之間就是 vTaskSwitchContext 選 task 的成本。 */
        s_acc[CPUSTATS_SLOT_SCHED] += now - s_t_out;
    }
    s_have_out = 0;

    for (uint32_t i = 0; i < s_ntasks; i++) {
        if (s_handle[i] == tcb) {
            idx = i;
            break;
        }
    }

    /* 第一次見到這支 task：登記，並且「當場複製名字」。不能只存指標之後再查——
     * task 被刪除後 TCB 會還給 heap，屆時 pcTaskGetName 就是 use-after-free
     * （本專案的 bootstrap task 就會自我刪除）。整支程式最多只走進來 12 次。 */
    if ((idx == CPUSTATS_TASK_SLOTS) && (s_ntasks < CPUSTATS_TASK_SLOTS)) {
        const char *nm = pcTaskGetName((TaskHandle_t)tcb);
        uint32_t    k  = 0;

        idx = s_ntasks;
        s_handle[idx] = tcb;
        while ((k < (CPUSTATS_NAME_LEN - 1U)) && (nm[k] != '\0')) {
            s_name[idx][k] = nm[k];
            k++;
        }
        s_name[idx][k] = '\0';
        s_ntasks = idx + 1U;
    }

    if (idx < CPUSTATS_TASK_SLOTS) {
        s_cur       = idx;
        s_cur_valid = 1;
    } else {
        s_cur_valid = 0;                  /* slot 用完了，這支不計 */
    }

    s_t_in         = now;
    s_isr_in_slice = 0;
}

/*
 * ISR 進出 —— ISR_PORT_ENTER/EXIT 巨集的實作，由 isr_port_config.h 接上。
 * driver 只認巨集，不認本模組；沒有 config 檔的專案巨集為空，這兩支不會被連結。
 *
 * 用「一個 active 旗標」而不是巢狀深度計數，是為了容忍不成對呼叫：
 * portYIELD_FROM_ISR() 內含 traceISR_EXIT()，而本專案的 ISR 是透過 callback
 * 間接呼叫它的（app_tasks.c 的 on_uart_rx_isr / on_button_isr），而且好幾支
 * ISR 有「flag 沒設就直接 return」的早退路徑。深度計數只要漏掉一次配對就會
 * 永久失衡；旗標版本則是進多了不算、出多了忽略，自我修復。
 *
 * 代價：巢狀時只認最外層那一次進入到最內層那一次離開。本專案唯一的巢狀情境是
 * app IRQ(prio 6) 打斷 SysTick(prio 15)，此時 SysTick 的尾巴會被算進 task。
 * 量級是幾微秒，可接受。
 */
void cpustats_isr_enter(void)
{
    if (!s_isr_active) {
        s_isr_active = 1;
        s_isr_t0     = cpustats_cnt();
    }
}

void cpustats_isr_exit(void)
{
    uint32_t d;

    if (!s_isr_active) {
        return;                           /* 沒配對的 exit，忽略 */
    }
    s_isr_active = 0;

    d = cpustats_cnt() - s_isr_t0;
    if (d > CPUSTATS_ISR_MAX) {
        return;
    }

    s_isr_in_slice           += d;        /* 結算切片時要從 task 扣掉 */
    s_acc[CPUSTATS_SLOT_ISR] += d;
}

/* ═════════════════════ 取樣與查詢（task 脈絡）═════════════════════ */

/*
 * 把累加表複製出來，並且「虛擬結算」當下這支 task 的未完成切片。
 * 少了這一步，呼叫者（Display 或 Console）永遠低報自己，而且各 slot
 * 加總不到 100%——這正是 kernel 內建 run-time stats 的老問題。
 * 只加在複製出來的副本上，不動 s_acc，所以不影響下一次的差值計算。
 */
static void cpustats_snapshot(uint32_t *dst, uint32_t *p_now, uint32_t *p_ntasks)
{
    taskENTER_CRITICAL();
    {
        uint32_t now = cpustats_cnt();

        for (uint32_t i = 0; i < CPUSTATS_SLOTS; i++) {
            dst[i] = s_acc[i];
        }
        if (s_cur_valid) {
            uint32_t slice = now - s_t_in;
            uint32_t isr   = s_isr_in_slice;
            if (isr > slice) {
                isr = slice;
            }
            dst[s_cur] += slice - isr;
        }
        *p_now = now;
        /* s_ntasks 一併在鎖內讀：新 task 是在 PendSV 脈絡註冊的，鎖外讀會
         * 拿到跟 dst[] 不同一刻的數量。 */
        if (p_ntasks != NULL) {
            *p_ntasks = s_ntasks;
        }
    }
    taskEXIT_CRITICAL();
}

void cpustats_sample(void)
{
    uint32_t cur[CPUSTATS_SLOTS];
    uint32_t now;

    cpustats_snapshot(cur, &now, NULL);

    vTaskSuspendAll();
    {
        if (!s_prev_valid) {
            memcpy(s_prev, cur, sizeof(s_prev));
            s_prev_t     = now;
            s_prev_valid = 1;
        } else {
            uint32_t dt = now - s_prev_t;

            if (dt != 0U) {
                for (uint32_t i = 0; i < CPUSTATS_SLOTS; i++) {
                    uint32_t d  = cur[i] - s_prev[i];
                    /* 先乘後除、用 64-bit：區間 100ms 時分母才 10000，
                     * 兩段式整數除法的量化誤差會吃掉整個小數位。 */
                    uint32_t pm = (uint32_t)(((uint64_t)d * 1000U) / dt);

                    if (pm > 1000U) {
                        pm = 1000U;       /* 邊界誤差夾住 */
                    }
                    s_hist[i][s_hist_head] = (uint16_t)pm;
                    if ((uint16_t)pm > s_peak[i]) {
                        s_peak[i] = (uint16_t)pm;
                    }
                    s_prev[i] = cur[i];
                }
                s_prev_t = now;

                s_hist_head = (s_hist_head + 1U) % CPUSTATS_HIST_LEN;
                if (s_hist_count < CPUSTATS_HIST_LEN) {
                    s_hist_count++;
                }
            }
        }
    }
    (void)xTaskResumeAll();
}

/* 最近 n 筆取樣的平均（千分比）。呼叫端已持有 vTaskSuspendAll()。 */
static uint16_t cpustats_recent_avg(uint32_t slot, uint32_t n)
{
    uint32_t sum = 0;
    uint32_t idx = s_hist_head;

    if (s_hist_count == 0U) {
        return 0;
    }
    if (n > s_hist_count) {
        n = s_hist_count;
    }
    for (uint32_t k = 0; k < n; k++) {
        idx = (idx + CPUSTATS_HIST_LEN - 1U) % CPUSTATS_HIST_LEN;
        sum += s_hist[slot][idx];
    }
    return (uint16_t)(sum / n);
}

uint32_t cpustats_get(cpustats_slot_t *out, uint32_t max)
{
    uint32_t cur[CPUSTATS_SLOTS];
    uint32_t now;
    uint32_t elapsed;
    uint32_t ntasks;
    uint32_t n;

    cpustats_snapshot(cur, &now, &ntasks);
    elapsed = now - s_t_start;

    n = ntasks + 2U;                      /* task + [ISR] + [SCHED] */
    if (n > max) {
        n = max;
    }

    vTaskSuspendAll();
    {
        for (uint32_t i = 0; i < n; i++) {
            uint32_t slot;

            if (i < ntasks) {
                slot          = i;
                out[i].name   = s_name[i];
                out[i].handle = s_handle[i];
            } else {
                slot          = CPUSTATS_TASK_SLOTS + (i - ntasks);
                out[i].name   = (slot == CPUSTATS_SLOT_ISR) ? "[ISR]" : "[SCHED]";
                out[i].handle = NULL;
            }

            /* CPU%：最近 1 秒（10 筆 100ms 取樣）的平均。單看最後一筆會
             * 太跳，看開機至今又會被啟動後的閒置稀釋掉。 */
            out[i].slot    = (uint8_t)slot;
            out[i].cur_pm  = cpustats_recent_avg(slot, 1000U / CPUSTATS_SAMPLE_MS);
            out[i].peak_pm = s_peak[slot];
            out[i].avg_pm  = (elapsed != 0U)
                           ? (uint16_t)(((uint64_t)cur[slot] * 1000U) / elapsed)
                           : (uint16_t)0U;
        }
    }
    (void)xTaskResumeAll();

    return n;
}

uint32_t cpustats_hist(uint32_t slot, uint16_t *out, uint32_t max)
{
    uint32_t n;

    if (slot >= CPUSTATS_SLOTS) {
        return 0;
    }

    vTaskSuspendAll();
    {
        n = (s_hist_count < max) ? s_hist_count : max;
        /* 由舊到新：head 是下一個要寫的位置，往回推 n 格就是最舊的一筆。 */
        for (uint32_t k = 0; k < n; k++) {
            uint32_t idx = (s_hist_head + CPUSTATS_HIST_LEN - n + k) % CPUSTATS_HIST_LEN;
            out[k] = s_hist[slot][idx];
        }
    }
    (void)xTaskResumeAll();

    return n;
}

void cpustats_reset(void)
{
    vTaskSuspendAll();
    {
        memset(s_peak, 0, sizeof(s_peak));
        memset(s_hist, 0, sizeof(s_hist));
        s_hist_head  = 0;
        s_hist_count = 0;
        s_prev_valid = 0;                 /* 下一次取樣重新建立基準 */
    }
    (void)xTaskResumeAll();
}
