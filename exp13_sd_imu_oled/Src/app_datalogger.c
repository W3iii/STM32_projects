/*
 * app_datalogger.c
 *
 *  IMU 資料記錄器 — Application 層核心。
 *  ─────────────────────────────────────────────
 *  流程：
 *    start  → 找未用的 log_NNN.csv、開檔、寫 CSV 表頭、進 LOGGING
 *    tick   → 每 10ms(100Hz) 讀 MPU6050、append 一行；每 1s f_sync
 *    stop   → f_sync + f_close、回 IDLE
 *  ─────────────────────────────────────────────
 *  緩衝策略：FatFs 的 FIL 內建 512B sector 緩衝，f_write/f_printf 會先
 *  累積在那裡，滿一個 sector 才真的寫 SD；我們只額外每秒做一次 f_sync
 *  把 FAT/目錄落地，避免每筆都 sync 太慢（見 SPEC §6.3）。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "app_datalogger.h"
#include "ff.h"
#include "device_mpu6050.h"
#include "systick.h"

#define DL_SAMPLE_PERIOD_MS  10     /* 100Hz 取樣 */
#define DL_SYNC_PERIOD_MS    1000   /* 每秒 f_sync 一次 */

static FATFS      s_fs;
static FIL        s_fil;
static dl_state_t s_state = DL_IDLE;
static uint32_t   s_count;
static uint32_t   s_t_sample;
static uint32_t   s_t_sync;
static char       s_filename[16];   /* "log_NNN.csv" + '\0' */
static uint8_t    s_mounted;

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
    if (mpu6050_init() != 0) {
        return -1;                  /* WHO_AM_I 不對 */
    }
    if (f_mount(&s_fs, "", 1) != FR_OK) {
        return -2;                  /* 掛載失敗（卡沒 FAT32？）*/
    }
    s_mounted = 1;
    s_state = DL_IDLE;
    return 0;
}

int app_datalogger_start(void)
{
    if (s_state == DL_LOGGING) {
        return 0;                   /* 已在記錄 */
    }
    if (!s_mounted) {
        return -1;
    }

    dl_make_filename();
    if (f_open(&s_fil, s_filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        return -2;
    }
    f_puts("t_ms,ax,ay,az,gx,gy,gz\n", &s_fil);

    s_count    = 0;
    s_t_sample = systick_get_ms();
    s_t_sync   = s_t_sample;
    s_state    = DL_LOGGING;
    return 0;
}

int app_datalogger_stop(void)
{
    if (s_state != DL_LOGGING) {
        return 0;
    }
    f_sync(&s_fil);
    f_close(&s_fil);
    s_state = DL_IDLE;
    return 0;
}

void app_datalogger_tick(uint32_t now_ms)
{
    if (s_state != DL_LOGGING) {
        return;
    }

    /* 100Hz 取樣 + 寫一行 */
    if (now_ms - s_t_sample >= DL_SAMPLE_PERIOD_MS) {
        s_t_sample = now_ms;

        mpu6050_axes_t acc, gyro;
        mpu6050_read_all(&acc, &gyro);

        /* 先記 raw int16（scaling 留待下一步，避免開 float printf）*/
        f_printf(&s_fil, "%u,%d,%d,%d,%d,%d,%d\n",
                 (unsigned)now_ms,
                 acc.x, acc.y, acc.z,
                 gyro.x, gyro.y, gyro.z);
        s_count++;
    }

    /* 週期性把 FAT/目錄落地（不是每筆都 sync）*/
    if (now_ms - s_t_sync >= DL_SYNC_PERIOD_MS) {
        s_t_sync = now_ms;
        f_sync(&s_fil);
    }
}

dl_state_t app_datalogger_state(void)
{
    return s_state;
}

uint32_t app_datalogger_count(void)
{
    return s_count;
}

const char *app_datalogger_filename(void)
{
    return s_filename;
}
