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

/* 同步：等到完成才 return */
void spi1_send_byte  (uint8_t b);
void spi1_send_buffer(const uint8_t *buf, uint32_t len);

/* 全雙工：送一個 byte 同時收回 MISO（SD 卡用） */
uint8_t spi1_transfer_byte(uint8_t b);

/* 切鮑率：br = CR1[5:3]，5=/64(250kHz, SD init), 2=/8(2MHz) */
void spi1_set_baud(uint8_t br);

/* 非同步：啟動 DMA 就 return */
void spi1_send_buffer_async(const uint8_t *buf, uint32_t len);

/* 狀態查詢 */
int  spi1_is_busy(void);
void spi1_wait_done(void);

#endif /* SPI_DRIVER_H_ */
