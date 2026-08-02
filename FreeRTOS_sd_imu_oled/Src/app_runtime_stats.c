/*
 * app_runtime_stats.c
 *
 *  FreeRTOS run-time stats 的計時源（給 `tasks` CLI 算 CPU%）。
 *  ─────────────────────────────────────────────
 *  FreeRTOS 每次切換 task 都會把「離開的 task 執行了多久」累加起來，
 *  但 tick(1ms) 太粗，量不準短命的 task。這裡拿一顆閒置的 TIM2 當高解析度
 *  自由計數器：port 透過 FreeRTOSConfig.h 的兩個 macro
 *      portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() → vConfigureTimerForRunTimeStats()
 *      portGET_RUN_TIME_COUNTER_VALUE()         → ulGetRunTimeCounterValue()
 *  取用本檔。
 *
 *  頻率選 ~100kHz（= tick 1kHz 的 100 倍，遠高於 spec 要求的 10 倍），
 *  32-bit counter 約 11.9 小時才繞回，對 demo 綽綽有餘。TIM2 不開中斷、
 *  不走 DMA，純唯讀 CNT，零 ISR 負擔。
 *
 *  TIM2 掛在 APB1：當 APB1 prescaler != 1 時，timer clock = 2 × PCLK1
 *  （RM0090 §7.2 圖）。目前 PCLK1=42MHz → TIM2 clk = 84MHz，由 rcc_pclk1_hz()
 *  即時推導，改 PLL 設定不用動這裡。
 *
 *  Created on: Aug 2, 2026
 *      Author: xushu
 */

#include "stm32f407xx.h"
#include "driver_rcc.h"

/* stm32f407xx.h 沒提供 TIM bit macro，這裡自己定義（同 i2c driver 風格）。 */
#define TIM_CR1_CEN     (1U << 0)    /* Counter enable */
#define TIM_EGR_UG      (1U << 0)    /* Update generation（立刻載入 PSC/ARR）*/

#define RTSTATS_HZ      100000U      /* run-time 計數頻率：100kHz */

void vConfigureTimerForRunTimeStats(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* APB1 prescaler != 1 → timer clock = 2 × PCLK1（= 84MHz）。 */
    uint32_t timer_clk = rcc_pclk1_hz() * 2U;

    TIM2->PSC = (uint16_t)(timer_clk / RTSTATS_HZ - 1U);   /* 84MHz/840 = 100kHz */
    TIM2->ARR = 0xFFFFFFFFU;                               /* 32-bit 自由跑 */
    TIM2->EGR = TIM_EGR_UG;                                /* 立刻把 PSC 載進去 */
    TIM2->CR1 = TIM_CR1_CEN;                               /* 開始計數 */
}

unsigned long ulGetRunTimeCounterValue(void)
{
    return (unsigned long)TIM2->CNT;
}
