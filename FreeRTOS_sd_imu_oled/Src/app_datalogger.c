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
#include "task.h"       /* TaskHandle_t / xTaskNotifyGive */

static FATFS      s_fs;
static FIL        s_fil;
static dl_state_t s_state = DL_IDLE;
static uint32_t   s_count;
static char       s_filename[16];   /* "log_NNN.csv" + '\0' */
static uint8_t    s_mounted;

static SemaphoreHandle_t s_state_mtx;   /* 保護 s_state / s_count / s_filename */

static TaskHandle_t s_producer;         /* sensor task handle；start 成功時喚醒它 */

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

    /* 事件驅動：喚醒 sensor 進 100Hz 取樣迴圈。只在 IDLE→LOGGING 成功時發。
     * 不變量：IDLE ⟺ sensor 卡在 ulTaskNotifyTake()，所以這個通知一定接得到、
     * 不會掉，也不會在 sensor 跑迴圈時被 latch 成假喚醒。stop 不發通知。 */
    if (s_producer != NULL) {
        xTaskNotifyGive(s_producer);
    }
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

/* ───────────────── sensor 喚醒注入 ───────────────── */
void app_datalogger_set_producer(TaskHandle_t producer)
{
    s_producer = producer;
}

/* ───────────────── 檔案層操作（給 CLI 用）─────────────────
 * 全部 FatFs/SD 存取 + bus_lock 收在這裡，emit callback 在 bus_lock 內被回呼，
 * callback 內只能碰 UART、不可碰 SD/OLED（同一支 bus_lock）。 */

int dl_list(void (*emit)(const char *name, uint32_t size), uint32_t *free_kb_out)
{
    static DIR dir;             /* static：DIR 別放 stack */
    FILINFO    fno;
    int        n = 0;

    bus_lock();                 /* 整段目錄掃描 + getfree 與寫卡/OLED 互斥 */
    if (f_opendir(&dir, "/") != FR_OK) {
        bus_unlock();
        return -1;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fattrib & AM_DIR) {
            continue;           /* 只列檔案 */
        }
        if (emit) {
            emit(fno.fname, (uint32_t)fno.fsize);
        }
        n++;
    }
    f_closedir(&dir);

    if (free_kb_out) {
        /* 剩餘空間：free clusters × 每 cluster 幾個 sector × 512 / 1024 = KB */
        DWORD  fre_clust;
        FATFS *fsp;
        if (f_getfree("", &fre_clust, &fsp) == FR_OK) {
            *free_kb_out = (uint32_t)((uint64_t)fre_clust * fsp->csize / 2);
        } else {
            *free_kb_out = 0;
        }
    }
    bus_unlock();
    return n;
}

int dl_dump(const char *name, void (*emit_line)(const char *line))
{
    static FIL f;               /* static：FIL 很大，別放 stack */
    char       line[64];
    int        rc = 0;

    /* 整段 FatFs 讀取包在 bus_lock：和 Logger 寫卡、Display 畫 OLED 互斥
     * （dump 期間 OLED 會短暫凍結，可接受）。 */
    bus_lock();
    if (f_open(&f, name, FA_READ) != FR_OK) {
        rc = -1;
    } else {
        while (f_gets(line, sizeof line, &f) != 0) {
            if (emit_line) {
                emit_line(line);
            }
        }
        f_close(&f);
    }
    bus_unlock();
    return rc;
}

int dl_clear(uint32_t *deleted_out)
{
    char name[16];
    int  n = 0;

    /* 直接掃 000..999 unlink，避免「邊 readdir 邊 unlink」可能漏項的問題。 */
    bus_lock();
    for (int i = 0; i < 1000; i++) {
        name[0] = 'l'; name[1] = 'o'; name[2] = 'g'; name[3] = '_';
        name[4] = (char)('0' + (i / 100));
        name[5] = (char)('0' + (i / 10) % 10);
        name[6] = (char)('0' + (i % 10));
        name[7] = '.'; name[8] = 'c'; name[9] = 's'; name[10] = 'v';
        name[11] = '\0';
        if (f_unlink(name) == FR_OK) {
            n++;
        }
    }
    bus_unlock();
    if (deleted_out) {
        *deleted_out = (uint32_t)n;
    }
    return 0;
}