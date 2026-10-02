/*
 * spi_driver.h
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include <stdint.h>

/* 初始化 */
void spi1_init(void);

/* 同步：等到完成才 return
 * spi1_send_buffer 回傳 0 = 正常，-1 = 逾時 / TEIF / buf 不在 DMA 可存取區
 * （Flash 或主 SRAM；task stack 在 CCM，local 陣列不行，見 dma_mem.h）。 */
void spi1_send_byte  (uint8_t b);
int  spi1_send_buffer(const uint8_t *buf, uint32_t len);

/* 全雙工：送一個 byte 同時收回 MISO（SD 卡用） */
uint8_t spi1_transfer_byte(uint8_t b);

/* 切鮑率：br = CR1[5:3]，PCLK2=84MHz：7=/256(328kHz, SD init), 4=/32(2.6MHz) */
void spi1_set_baud(uint8_t br);

/* 非同步：啟動 DMA 就 return。回傳 0 = 已啟動，-1 = buf 不在 DMA 可存取區、
 * 沒有啟動（之後的 spi1_wait_done() 也會回 -1，忽略這裡的回傳值也不會漏）。 */
int  spi1_send_buffer_async(const uint8_t *buf, uint32_t len);

/* 狀態查詢 */
int  spi1_is_busy(void);
/* 等 DMA + SPI 傳完。回傳 0 = 正常，-1 = 逾時。
 * 逾時時 DMA stream 已被關掉、busy 旗標已放掉，SPI1 可以交給下一個使用者 ——
 * 呼叫端就算忽略回傳值，也不會像以前那樣抱著 spi1_lock 永久空轉。 */
int spi1_wait_done(void);

#endif /* SPI_DRIVER_H_ */
