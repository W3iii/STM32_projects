/*
 * diskio.c
 *
 *  FatFs 媒體存取層 (glue) — 把 FatFs 的 disk_* 介面接到
 *  device_sdcard 的 single-block 讀寫上。
 *
 *  FatFs 只透過這 5 個函式碰硬體：
 *    disk_status / disk_initialize / disk_read / disk_write / disk_ioctl
 *  我們把它們轉呼叫 sd_init / sd_read_block / sd_write_block。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#include "ff.h"          /* FatFs 型別 (DSTATUS/DRESULT/LBA_t...) */
#include "diskio.h"      /* FatFs 媒體層原型 */
#include "device_sdcard.h"

#define DEV_SD   0       /* 只有一個磁碟：SD 卡 = pdrv 0 */

/* 目前磁碟狀態。FatFs 靠 STA_NOINIT 判斷有沒有初始化過。 */
static volatile DSTATUS s_stat = STA_NOINIT;

/* ---- 狀態查詢 ---- */
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != DEV_SD) {
        return STA_NOINIT;
    }
    return s_stat;
}

/* ---- 初始化（FatFs 在第一次掛載/開檔時呼叫）---- */
DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != DEV_SD) {
        return STA_NOINIT;
    }
    sd_last_init_rc = sd_init();     // 診斷：保留卡在哪一步（SD_ERR_*）
    if (sd_last_init_rc == SD_OK) {
        s_stat &= ~STA_NOINIT;       // 清 NOINIT = ready
    } else {
        s_stat = STA_NOINIT;
    }
    return s_stat;
}

/* ---- 讀 count 個 512B sector ---- */
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != DEV_SD) {
        return RES_PARERR;
    }
    if (s_stat & STA_NOINIT) {
        return RES_NOTRDY;
    }
    for (UINT i = 0; i < count; i++) {
        if (sd_read_block((uint32_t)sector + i, buff + i * SD_BLOCK_SIZE) != SD_OK) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

/* ---- 寫 count 個 512B sector ---- */
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != DEV_SD) {
        return RES_PARERR;
    }
    if (s_stat & STA_NOINIT) {
        return RES_NOTRDY;
    }
    for (UINT i = 0; i < count; i++) {
        if (sd_write_block((uint32_t)sector + i, buff + i * SD_BLOCK_SIZE) != SD_OK) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

/* ---- 其他控制 ---- */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)buff;
    if (pdrv != DEV_SD) {
        return RES_PARERR;
    }
    switch (cmd) {
        case CTRL_SYNC:
            /* sd_write_block 本身已經等到卡 busy 結束才回來，
             * 資料已經落地，這裡不用再做事。 */
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

/* FatFs 拿來蓋檔案時間戳。FF_FS_NORTC == 1 時其實不會被呼叫，
 * 但 NORTC == 0 時需要它，所以一律提供一個固定值。
 *   bit31:25 = year-1980, 24:21 = month, 20:16 = day,
 *   bit15:11 = hour,      10:5  = min,   4:0   = sec/2 */
DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25)
         | ((DWORD)6  << 21)
         | ((DWORD)18 << 16)
         | ((DWORD)0  << 11)
         | ((DWORD)0  << 5)
         | ((DWORD)0  >> 1);
}
