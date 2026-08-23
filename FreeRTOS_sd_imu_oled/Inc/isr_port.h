/*
 * isr_port.h
 *
 *  ISR 計時掛勾 —— 預設展開成空的巨集介面。
 *  ─────────────────────────────────────────────
 *  仿 FreeRTOS 的 traceXXX 巨集：
 *
 *      FreeRTOS.h:64     #include "FreeRTOSConfig.h"      ← 專案提供
 *      FreeRTOS.h:617    #ifndef traceTASK_SWITCHED_IN
 *                        #define traceTASK_SWITCHED_IN()  ← 空的預設值
 *                        #endif
 *      tasks.c:39        #include "FreeRTOS.h"            ← kernel 照樣要 include
 *
 *  重點不在「driver 不 include 東西」（不 include 的話巨集根本不存在），
 *  而在於：**預設展開成空，不產生任何呼叫、不需要任何 symbol**。所以
 *
 *    - bare-metal 的 exp13：什麼都不用做。沒有 isr_port_config.h → 巨集為空
 *      → 那幾行 ISR_PORT_ENTER() 完全消失，零指令、零連結相依。driver 原封
 *      不動搬過去就能編。
 *    - 這個 FreeRTOS 專案：Inc/ 下放一支 isr_port_config.h 把巨集接到
 *      app_cpu_stats.c 的實作。driver 不必改一個字。
 *
 *  ── 為什麼 driver 非得留這個洞不可 ──
 *  Cortex-M 沒有「通用的中斷進入 hook」，NVIC 直接跳進 vector table 指的
 *  handler。想量 ISR 佔掉多少 CPU，就只能在每支 handler 的頭尾各插一次
 *  時間戳；而 ISR 時間量不到的話，整份 CPU 統計都是歪的 —— FreeRTOS 內建的
 *  run-time stats 就是因為沒有這個洞，才會把 ISR 時間算到剛好被打斷的那支
 *  task 頭上。
 *
 *  ── 使用約定 ──
 *  在 IRQ handler 的最開頭放 ISR_PORT_ENTER()、最後面放 ISR_PORT_EXIT()，
 *  所有 return 路徑都要走到 EXIT。實作端必須容忍不成對呼叫（進多了不算、
 *  出多了忽略），因為 FreeRTOS 的 portYIELD_FROM_ISR() 內含 traceISR_EXIT()，
 *  會產生額外的 exit —— 詳見 app_cpu_stats.c 的說明。
 *
 *  Created on: Aug 20, 2026
 *      Author: xushu
 */

#ifndef ISR_PORT_H_
#define ISR_PORT_H_

/* 專案若在 include path 上提供 isr_port_config.h，就用它定義的實作。
 * __has_include 是 C23 標準（GCC 5 以後都支援），沒有這支檔案時不會報錯。 */
#if defined(__has_include)
    #if __has_include("isr_port_config.h")
        #include "isr_port_config.h"
    #endif
#endif

#ifndef ISR_PORT_ENTER
    #define ISR_PORT_ENTER()    ( ( void ) 0 )
#endif

#ifndef ISR_PORT_EXIT
    #define ISR_PORT_EXIT()     ( ( void ) 0 )
#endif

#endif /* ISR_PORT_H_ */
