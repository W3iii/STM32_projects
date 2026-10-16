/*
 * device_sdcard.h
 *
 *  SD 卡 (SPI 模式) driver — exp13（init + CMD17 讀 / CMD24 寫 block）
 *  ─────────────────────────────────────────────
 *  接線：
 *    SCK  = PA5   (SPI1, AF5)
 *    MISO = PA6   (SPI1, AF5)
 *    MOSI = PA7   (SPI1, AF5)
 *    CS   = PB10  (軟體控制的 GPIO output)
 *  ─────────────────────────────────────────────
 *
 *  Created on: Jun 16, 2026
 *      Author: xushu
 */

#ifndef DEVICE_SDCARD_H_
#define DEVICE_SDCARD_H_

#include <stdint.h>

/* 回傳碼 */
#define SD_OK            0
#define SD_ERR_CMD0      1   /* CMD0 沒回 0x01（沒進 idle state）       */
#define SD_ERR_CMD8      2   /* CMD8 電壓/pattern 不符或非預期回應      */
#define SD_ERR_ACMD41    3   /* ACMD41 逾時，卡沒離開 idle              */
#define SD_ERR_CMD58     4   /* CMD58 讀 OCR 失敗                       */
#define SD_ERR_READ      5   /* CMD17 讀失敗（R1 非 0 或沒等到 data token）*/
#define SD_ERR_WRITE     6   /* CMD24 寫失敗（R1 非 0 / 拒絕 / busy 逾時） */

/* 一個 block 固定 512 byte */
#define SD_BLOCK_SIZE    512

/* 卡片類型 */
#define SD_TYPE_UNKNOWN  0
#define SD_TYPE_V1       1   /* SD v1.x / MMC（不認得 CMD8）           */
#define SD_TYPE_V2       2   /* SD v2.0+ 標準容量 SDSC（byte 定址）    */
#define SD_TYPE_SDHC     3   /* SDHC/SDXC（block 定址，CCS=1）         */

/* ---- 讀寫路徑的失敗現場 ----
 * init 路徑有 sd_last_init_rc / sd_last_r1 / sd_r7 / sd_ocr 可查，但 I/O
 * 路徑以前什麼都沒留：7 種失敗全被壓成 SD_ERR_READ / SD_ERR_WRITE，再壓成
 * diskio 的 RES_ERROR、FatFs 的 FR_DISK_ERR，到 status 只剩一個數字。
 * 「卡被拔掉」「供電不穩」「buffer 誤放 CCM」「線路雜訊」的處理方式天差地別，
 * 但呈現出來一模一樣 —— 所以在最細的那層留一份快照。 */
#define SD_IO_OK           0
#define SD_IO_RD_CMD_R1    1   /* CMD17 的 R1 != 0（byte = 該 R1）        */
#define SD_IO_RD_TOKEN_TMO 2   /* 等 data token 逾時（SD_READ_TIMEOUT_MS）*/
#define SD_IO_RD_ERR_TOKEN 3   /* 收到 error token（byte = 該 token）     */
#define SD_IO_WR_CMD_R1    4   /* CMD24 的 R1 != 0（byte = 該 R1）        */
#define SD_IO_WR_DMA_MEM   5   /* buffer DMA 碰不到（典型是 CCM 上的 stack）*/
#define SD_IO_WR_DMA_TMO   6   /* DMA/SPI 傳輸逾時                        */
#define SD_IO_WR_RESP      7   /* data response 被拒（byte = 該 resp）    */
#define SD_IO_WR_BUSY_TMO  8   /* 等 busy 結束逾時（SD_WRITE_TIMEOUT_MS） */

/* debug 用全域（數值意義見 .c） */
extern volatile uint8_t sd_last_r1;       /* 最後一次命令的原始 R1     */
extern volatile uint8_t sd_last_init_rc;  /* 最後一次 sd_init() 回傳碼  */
extern volatile uint8_t sd_last_cmd55_r1; /* 最後一次 CMD55 的 R1      */
extern volatile uint8_t sd_card_type;     /* SD_TYPE_*                 */
extern volatile uint8_t sd_r7[4];       /* CMD8 的 4-byte trailing */
extern volatile uint8_t sd_ocr[4];      /* CMD58 讀回的 OCR        */

/* 讀寫路徑「最後一次失敗」的現場（SD_IO_*）。刻意不在成功時清掉 —— 失敗後
 * 通常還會有成千上萬次成功寫入，清掉就查不到了。要判斷這份快照是否還有意義，
 * 看 dl_status 的 err_count：為 0 就代表從沒失敗過，這三個值無意義。 */
extern volatile uint8_t  sd_last_io_err;   /* SD_IO_*                      */
extern volatile uint8_t  sd_last_io_byte;  /* 當時讀到的 R1 / token / resp */
extern volatile uint32_t sd_last_io_blk;   /* 出事的 block 號              */

/* SD_IO_* 的短字串（給 CLI 印）。未知值回 "?"。 */
const char *sd_io_err_str(uint8_t code);

/* CS 腳 GPIO 初始化（PB10 推挽輸出，預設拉高）。
 * spi1_init() 要先呼叫過，這裡只負責 CS。 */
void sd_cs_init(void);

/* 完整開卡：CMD0 → CMD8 → ACMD41 → CMD58。
 * 成功時 sd_card_type 會被設成 SD_TYPE_V1 / SD_TYPE_V2 / SD_TYPE_SDHC。
 * 回傳 SD_OK 或對應的 SD_ERR_*。 */
uint8_t sd_init(void);

/* CMD17：讀單一 block（512 byte）到 buf。
 * block = 邏輯 block 號（SDHC 直接用；SDSC 內部換算成 byte 位址）。
 * 回傳 SD_OK 或 SD_ERR_READ。 */
uint8_t sd_read_block(uint32_t block, uint8_t *buf);

/* CMD24：把 buf（512 byte）寫進單一 block。
 * 回傳 SD_OK 或 SD_ERR_WRITE。 */
uint8_t sd_write_block(uint32_t block, const uint8_t *buf);

#endif /* DEVICE_SDCARD_H_ */
