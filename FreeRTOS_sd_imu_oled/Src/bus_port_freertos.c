/*
 * bus_port_freertos.c
 *
 *  bus_port.h 的 FreeRTOS 實作：每條匯流排一支 non-recursive mutex。
 *  匯流排鎖的擁有權（mutex 本體）收在這裡，不再散落於 app_tasks.c —— 上層
 *  只透過 spi1_lock()/i2c1_lock() 這些具名函式使用，換到 bare-metal 時
 *  只要改連 bus_port_baremetal.c（no-op / 關中斷版）即可，其餘檔案不動。
 *
 *  兩支鎖彼此獨立，沒有任何路徑會同時持有兩支（SPI 的使用者不碰 I2C，
 *  反之亦然），所以不需要定義鎖序、也不會 ABBA 死鎖。
 */

#include "bus_port.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t s_mSpi1;   /* OLED + SD/FatFs */
static SemaphoreHandle_t s_mI2c1;   /* MPU6050 + INA219 */

int bus_port_init(void)
{
    s_mSpi1 = xSemaphoreCreateMutex();
    s_mI2c1 = xSemaphoreCreateMutex();
    return (s_mSpi1 != NULL && s_mI2c1 != NULL) ? 0 : -1;
}

void spi1_lock(void)   { xSemaphoreTake(s_mSpi1, portMAX_DELAY); }
void spi1_unlock(void) { xSemaphoreGive(s_mSpi1); }

void i2c1_lock(void)   { xSemaphoreTake(s_mI2c1, portMAX_DELAY); }
void i2c1_unlock(void) { xSemaphoreGive(s_mI2c1); }
