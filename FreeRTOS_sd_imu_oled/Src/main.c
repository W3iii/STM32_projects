/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IMU datalogger — FreeRTOS 入口（Phase 2：四 task 架構）
 *
 *  MPU6050(I2C1) → 100Hz 取樣 → FatFs/CSV → SD(SPI1)
 *  OLED(SPI1) 顯示狀態、UART(USART1) 提供 start/stop/status CLI。
 *
 *  實際的 task 切分（Sensor/Logger/Display/Console）、佇列與 mutex 都在
 *  app_tasks.c。main() 只負責建立同步原語 + bootstrap task，然後啟動排程器。
 *  周邊與應用初始化跑在 bootstrap task 內（排程器啟動後 delay_ms 才有效）。
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "driver_rcc.h"
#include "driver_usart1.h"

int main(void)
{
    /* 先把 SYSCLK 拉到 168MHz（HSE 8MHz→PLL，失敗 fallback HSI，見 driver_rcc.c）。
     * 必須在排程器之前：port 用 configCPU_CLOCK_HZ(=168e6) 算 SysTick reload，硬體得先到位。 */
    rcc_clock_init_168mhz();

    app_tasks_start();        // 建 queue/mutex + bootstrap task
    vTaskStartScheduler();

    /* 跑到這代表 heap 不夠建 idle/timer task —— 不該發生 */
    for (;;);
}

/* ---- 故障輸出（panic）─────────────────────────────────────────────
 * crash hook / configASSERT 共用。只用 polling UART（usart1_send_*，poll TXE，
 * 不靠中斷/DMA/RTOS），所以即使排程已死、taskDISABLE_INTERRUPTS() 後或在 ISR
 * 脈絡都還能把訊息送出去。TXE 是硬體旗標，中斷關掉照樣會 set。
 * 注意：這些 hook 多半在 bootstrap task 內（xTaskCreate 配不到記憶體時）或
 * 執行期觸發，此時 usart1_init() 已跑過，UART 是通的。若在 usart1_init 之前就
 * 觸發，送字只是無聲（USART 沒 enable），不會卡死。 */
static void panic_print_u32(uint32_t v)
{
    char b[11];
    int  i = 10;
    b[i] = '\0';
    if (v == 0) { usart1_send_string("0"); return; }
    while (v && i > 0) { b[--i] = (char)('0' + v % 10); v /= 10; }
    usart1_send_string(&b[i]);
}

/* ---- FreeRTOS hook（config 有打開就必須提供，否則 link 不過）---- */

/* heap_4 malloc 失敗：通常是 xTaskCreate/xQueueCreate 配不到記憶體。
 * 印出剩餘 heap 與歷史最低值，方便決定 configTOTAL_HEAP_SIZE 要加多少。 */
void vApplicationMallocFailedHook(void)
{
    usart1_send_string("\r\n[PANIC] malloc failed (heap exhausted)\r\n  freeNow=");
    panic_print_u32((uint32_t)xPortGetFreeHeapSize());
    usart1_send_string(" B  minEver=");
    panic_print_u32((uint32_t)xPortGetMinimumEverFreeHeapSize());
    usart1_send_string(" B\r\n  -> raise configTOTAL_HEAP_SIZE\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* task stack 溢位（configCHECK_FOR_STACK_OVERFLOW=2）：印出是哪個 task 爆，
 * 直接對著它把 STACK_xxx 調大。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    usart1_send_string("\r\n[PANIC] stack overflow in task: ");
    usart1_send_string(pcTaskName ? pcTaskName : "?");
    usart1_send_string("\r\n  -> raise that task's stack depth\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* configASSERT 失敗：印出觸發的檔案與行號（FreeRTOS 內部與我們的 configASSERT
 * 都會走到這）。配 FreeRTOSConfig.h 的 configASSERT 巨集。 */
void vAssertCalled(const char *file, unsigned long line)
{
    usart1_send_string("\r\n[ASSERT] ");
    usart1_send_string(file ? file : "?");
    usart1_send_string(":");
    panic_print_u32((uint32_t)line);
    usart1_send_string("\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}
