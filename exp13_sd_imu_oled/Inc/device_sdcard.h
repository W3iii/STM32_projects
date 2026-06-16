/*
 * device_sdcard.h
 *
 *  SD 卡 (SPI 模式) driver — exp13 第一步：CMD0 初始化
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
#define SD_ERR_CMD0      1   /* CMD0 沒回 0x01（沒進 idle state） */

/* 最後一次 CMD0 收到的原始 R1（debug 用），數值意義見 .c */
extern volatile uint8_t sd_last_r1;

/* CS 腳 GPIO 初始化（PB10 推挽輸出，預設拉高）。
 * spi1_init() 要先呼叫過，這裡只負責 CS。 */
void sd_cs_init(void);

/* 第一步：送 74+ 個 dummy clock 喚醒卡，再送 CMD0 進 idle state。
 * 回傳 SD_OK 或 SD_ERR_CMD0。 */
uint8_t sd_init(void);

#endif /* DEVICE_SDCARD_H_ */
