/*
 * app_datalogger.c
 *
 *  IMU 資料記錄器 — Application 層核心（FreeRTOS 版）。
 *  ─────────────────────────────────────────────
 *  與 bare-metal 版的差別：
 *    - 取樣（讀 MPU）移到 Sensor task；本模組只負責「寫入 + 狀態」。
 *    - 所有 FatFs/SD 操作包在 bus_lock()/bus_unlock()（SPI1 匯流排互斥，
 *      和 OLED 共用同一支 mBus，見 app_tasks.c）。
 *    - 狀態（state/count/filename）由自有的 s_state_mtx 保護，給 Display /
 *      CLI status 用 get_status() 一次取走。
 *
 *  鎖序：需要時 bus_lock 在外、s_state_mtx 在內（只有 writer 會巢狀），
 *  讀者只取 s_state_mtx，故不會死鎖。
 *
 *  緩衝策略不變：FatFs 的 FIL 內建 512B sector 緩衝，f_printf 先累積，
 *  滿一個 sector 才真的寫 SD；另外每秒 f_sync 把 FAT/目錄落地（SPEC §6.3）。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "app_datalogger.h"
#include "app_tasks.h"      /* bus_lock / bus_unlock */
#include "ff.h"
#include "device_mpu6050.h"
#include "FreeRTOS.h"
#include "semphr.h"

static FATFS      s_fs;
static FIL        s_fil;
static dl_state_t s_state = DL_IDLE;
static uint32_t   s_count;
static char       s_filename[16];   /* "log_NNN.csv" + '\0' */
static uint8_t    s_mounted;

static SemaphoreHandle_t s_state_mtx;   /* 保護 s_state / s_count / s_filename */

/* 診斷：保留最後一次 f_mount 的 FRESULT，方便 bootstrap 顯示掛載失敗原因
 *   1=FR_DISK_ERR、3=FR_NOT_READY → SPI/卡片硬體；13=FR_NO_FILESYSTEM → 不是 FAT32 */
static volatile FRESULT s_mount_res = FR_OK;

static inline void state_lock(void)   { xSemaphoreTake(s_state_mtx, portMAX_DELAY); }
static inline void state_unlock(void) { xSemaphoreGive(s_state_mtx); }

/* 找第一個還沒被用的 log_NNN.csv（NNN = 000..999）。 */
static void dl_make_filename(void)
{
    FILINFO fno;
    for (int i = 0; i < 1000; i++) {
        s_filename[0] = 'l';
        s_filename[1] = 'o';
        s_filename[2] = 'g';
        s_filename[3] = '_';
        s_filename[4] = (char)('0' + (i / 100));
        s_filename[5] = (char)('0' + (i / 10) % 10);
        s_filename[6] = (char)('0' + (i % 10));
        s_filename[7] = '.';
        s_filename[8] = 'c';
        s_filename[9] = 's';
        s_filename[10] = 'v';
        s_filename[11] = '\0';
        if (f_stat(s_filename, &fno) == FR_NO_FILE) {
            return;                 /* 這個沒人用，就用它 */
        }
    }
    /* 000..999 都被用了：沿用迴圈最後設定的 log_999.csv（會被覆蓋）*/
}

int app_datalogger_init(void)
{
    s_state_mtx = xSemaphoreCreateMutex();
    if (s_state_mtx == NULL) {
        return -3;
    }
    if (mpu6050_init() != 0) {
        return -1;                  /* WHO_AM_I 不對 */
    }
    s_mount_res = f_mount(&s_fs, "", 1);
    if (s_mount_res != FR_OK) {
        return -2;                  /* 掛載失敗：FRESULT 存在 s_mount_res */
    }
    s_mounted = 1;
    s_state = DL_IDLE;
    return 0;
}

int app_datalogger_mount_result(void)
{
    return (int)s_mount_res;        /* 診斷用：最後一次 f_mount 的 FRESULT */
}

int app_datalogger_start(void)
{
    if (!s_mounted) {
        return -1;
    }
    if (app_datalogger_state() == DL_LOGGING) {
        return 0;                   /* 已在記錄 */
    }

    int rc = 0;
    bus_lock();
    dl_make_filename();             /* 寫 s_filename（在 bus 內、稍後 state 內公布）*/
    if (f_open(&s_fil, s_filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        rc = -2;
    } else {
        f_puts("t_ms,ax,ay,az,gx,gy,gz\n", &s_fil);
    }
    bus_unlock();

    if (rc != 0) {
        return rc;
    }

    state_lock();
    s_count = 0;
    s_state = DL_LOGGING;
    state_unlock();
    return 0;
}

int app_datalogger_stop(void)
{
    if (app_datalogger_state() != DL_LOGGING) {
        return 0;
    }

    /* 先翻狀態 → Sensor 立刻停止取樣、Logger 不再寫；再關檔。 */
    state_lock();
    s_state = DL_IDLE;
    state_unlock();

    bus_lock();
    f_sync(&s_fil);
    f_close(&s_fil);
    bus_unlock();
    return 0;
}

int app_datalogger_write_sample(uint32_t t_ms,
                                const mpu6050_axes_t *acc,
                                const mpu6050_axes_t *gyro)
{
    int rc = 0;

    bus_lock();
    if (s_state == DL_LOGGING) {
        /* 先記 raw int16（scaling 留待下一步，避免開 float printf）*/
        rc = f_printf(&s_fil, "%u,%d,%d,%d,%d,%d,%d\n",
                      (unsigned)t_ms,
                      acc->x, acc->y, acc->z,
                      gyro->x, gyro->y, gyro->z);
        if (rc >= 0) {
            state_lock();
            s_count++;
            state_unlock();
            rc = 0;
        }
    }
    bus_unlock();
    return rc;
}

void app_datalogger_sync(void)
{
    bus_lock();
    if (s_state == DL_LOGGING) {
        f_sync(&s_fil);
    }
    bus_unlock();
}

void app_datalogger_get_status(dl_status_t *out)
{
    state_lock();
    out->state = s_state;
    out->count = s_count;
    for (int i = 0; i < (int)sizeof(s_filename); i++) {
        out->filename[i] = s_filename[i];
    }
    state_unlock();
}

dl_state_t app_datalogger_state(void)
{
    dl_state_t s;
    state_lock();
    s = s_state;
    state_unlock();
    return s;
}

uint32_t app_datalogger_count(void)
{
    uint32_t c;
    state_lock();
    c = s_count;
    state_unlock();
    return c;
}

const char *app_datalogger_filename(void)
{
    return s_filename;
}
