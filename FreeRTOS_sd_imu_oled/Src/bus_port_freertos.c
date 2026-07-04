/*
 * bus_port_freertos.c
 *
 *  bus_port.h 的 FreeRTOS 實作：SPI1 匯流排鎖用一支 non-recursive mutex。
 *  匯流排鎖的擁有權（mutex 本體）收在這裡，不再散落於 app_tasks.c —— 上層
 *  只透過 bus_lock()/bus_unlock() 這對具名函式使用，換到 bare-metal 時
 *  只要改連 bus_port_baremetal.c（no-op / 關中斷版）即可，其餘檔案不動。
 */

#include "bus_port.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t s_mBus;

int bus_port_init(void)
{
    s_mBus = xSemaphoreCreateMutex();
    return (s_mBus != NULL) ? 0 : -1;
}

void bus_lock(void)   { xSemaphoreTake(s_mBus, portMAX_DELAY); }
void bus_unlock(void) { xSemaphoreGive(s_mBus); }
