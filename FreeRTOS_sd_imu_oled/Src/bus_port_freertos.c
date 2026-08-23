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

/* I2C1 非同步交易完成的號誌。用 binary semaphore 而不是 task notification：
 * ulTaskNotifyTake 預設吃 index 0，而 sensor / console / button 都已經拿
 * index 0 接自己的事件（start 通知、UART RX、EXTI），撞在一起會互相吃掉
 * 對方的通知。專用號誌沒這問題，代價只有約 80 bytes heap。
 *
 * binary semaphore 會 latch：ISR 若在 task 走到 wait 之前就 give，稍後的
 * take 會立刻返回，不會 lost wakeup。 */
static SemaphoreHandle_t s_sI2c1Xfer;

/* SPI1 DMA 傳輸完成的號誌。理由與 s_sI2c1Xfer 相同：專用 binary semaphore，
 * 不跟 task notification 的 index 0 打架，而且 give 會 latch，ISR 若在 task
 * 走到 wait 之前就完成也不會 lost wakeup。 */
static SemaphoreHandle_t s_sSpi1Xfer;

/* 等待上限。交易本身在 400kHz 下約 400µs，給 50ms 是純粹的保險 ——
 * 狀態機若因為 bus stuck 卡住，Sensor task 要能自己脫身，不能死等。 */
#define I2C1_XFER_TIMEOUT_MS   50U

/* OLED 一次 1024 bytes 在 5.25MHz 下約 1.6ms、SD 降速到 2.6MHz 時約 3.1ms，
 * 50ms 有十幾倍餘裕。逾時只代表這一幀沒畫成，driver 會把 DMA 收乾淨。 */
#define SPI1_XFER_TIMEOUT_MS   50U

int bus_port_init(void)
{
    s_mSpi1      = xSemaphoreCreateMutex();
    s_mI2c1      = xSemaphoreCreateMutex();
    s_sI2c1Xfer  = xSemaphoreCreateBinary();
    s_sSpi1Xfer  = xSemaphoreCreateBinary();
    return (s_mSpi1 != NULL && s_mI2c1 != NULL &&
            s_sI2c1Xfer != NULL && s_sSpi1Xfer != NULL) ? 0 : -1;
}

void spi1_lock(void)   { xSemaphoreTake(s_mSpi1, portMAX_DELAY); }
void spi1_unlock(void) { xSemaphoreGive(s_mSpi1); }

/* 呼叫端一定持有 spi1_lock，所以同時只有一支 task 在等這支號誌。 */
int spi1_xfer_wait(void)
{
    return (xSemaphoreTake(s_sSpi1Xfer,
                           pdMS_TO_TICKS(SPI1_XFER_TIMEOUT_MS)) == pdTRUE) ? 0 : -1;
}

void spi1_xfer_signal(void)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_sSpi1Xfer, &woken);
    portYIELD_FROM_ISR(woken);
}

void spi1_xfer_reset(void)
{
    (void)xSemaphoreTake(s_sSpi1Xfer, 0);
}

void i2c1_lock(void)   { xSemaphoreTake(s_mI2c1, portMAX_DELAY); }
void i2c1_unlock(void) { xSemaphoreGive(s_mI2c1); }

/* 呼叫端一定持有 i2c1_lock，所以同一時間只有一支 task 在等這支號誌。
 * 發起交易前先把可能殘留的 give 清掉（上一筆逾時後 ISR 才姍姍來遲的情況），
 * 否則這次的 take 會立刻假性返回。 */
int i2c1_xfer_wait(void)
{
    return (xSemaphoreTake(s_sI2c1Xfer,
                           pdMS_TO_TICKS(I2C1_XFER_TIMEOUT_MS)) == pdTRUE) ? 0 : -1;
}

void i2c1_xfer_signal(void)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_sI2c1Xfer, &woken);
    /* Sensor 是 prio 4，比被打斷的那支高，直接在離開 ISR 時切過去，
     * 省掉一次排程延遲。 */
    portYIELD_FROM_ISR(woken);
}

/* 發起交易前把殘留的 give 清乾淨（見 i2c1_xfer_wait 的說明）。 */
void i2c1_xfer_reset(void)
{
    (void)xSemaphoreTake(s_sI2c1Xfer, 0);
}
