/*
 * systick.c
 *
 *  FreeRTOS 版：時間基準改由 RTOS tick 提供。
 *  ─────────────────────────────────────────────
 *  bare-metal 時這裡自己管 SysTick；上了 FreeRTOS 後，SysTick 由 port
 *  接管（SysTick_Handler 在 port.c，透過 FreeRTOSConfig.h 的 alias），
 *  所以這裡：
 *    - 不再定義 SysTick_Handler（否則和 port 撞符號）
 *    - systick_init() 變 no-op（port 在 vTaskStartScheduler() 設好 SysTick）
 *    - systick_get_ms() / delay_ms() 改接 RTOS tick，保留原本 API，
 *      上層 device_* / app_* 一行都不用改。
 *
 *  configTICK_RATE_HZ = 1000 → 1 tick = 1ms，和原本的語意一致。
 *
 *  注意：delay_ms() 現在只能在排程器啟動後（task 內）呼叫——所有周邊
 *  初始化都已移進 bootstrap task，所以成立。
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"

void systick_init(uint32_t ticks)
{
    (void)ticks;                /* SysTick 由 FreeRTOS port 設定，這裡不動 */
}

void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t systick_get_ms(void)
{
    return (uint32_t)xTaskGetTickCount();   /* TICK_RATE=1000 → tick 即 ms */
}
