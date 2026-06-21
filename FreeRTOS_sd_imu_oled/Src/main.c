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

/* ---- FreeRTOS hook（config 有打開就必須提供，否則 link 不過）---- */

/* heap_4 malloc 失敗：bring-up 期直接卡住，debugger 一看就知道 heap 不夠 */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* task stack 溢位（configCHECK_FOR_STACK_OVERFLOW=2）：同樣卡住定位，
 * pxTask / pcTaskName 可在 debugger 看是哪個 task 爆 stack。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;);
}
