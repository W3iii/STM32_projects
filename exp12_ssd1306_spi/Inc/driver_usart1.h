/*
 * driver_usart1.h
 *
 * USART1 driver for STM32F407
 *   - PA9  = TX (AF7)
 *   - PA10 = RX (AF7)
 *   - 預期接 CP2102 或 CH340 USB-UART → Tera Term
 *
 * 提供兩組 API：
 *   1. Polling (blocking)  - 適合 init/debug print
 *   2. DMA TX  (non-blocking) - 適合大量資料、log
 *
 * NOTE:
 *   目前系統 clock = 16MHz HSI（RCC->CFGR = 0 驗證）
 *   USART1 在 APB2 bus，PCLK2 = 16MHz
 *
 *   CH340 5V 介面下，9600+ baud 偶有訊號完整性問題，
 *   建議 baudrate=1200 測試。換 CP2102 後可用 115200。
 *
 *  Created on: May 19, 2026
 *      Author: xushu
 */

#ifndef DRIVER_USART1_H_
#define DRIVER_USART1_H_

#include <stdint.h>

/* ========================================================================= */
/*                          Status codes                                     */
/* ========================================================================= */
typedef enum {
    UART_OK = 0,
    UART_ERR_TIMEOUT,
    UART_ERR_BUSY,         /* DMA 還在傳，不能發新的 */
    UART_ERR_PARAM,        /* buf == NULL 或 len == 0 */
} uart_status_t;


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

/*
 * 初始化 USART1
 *   - 開 GPIOA / USART1 / DMA2 clock
 *   - PA9/PA10 設為 AF7
 *   - 設定 baudrate（自動計算 BRR）
 *   - 開啟 TE + RE + UE
 *   - 設定 DMA2 Stream7 Channel 4 給 USART1_TX（但不啟動）
 *
 * 常用 baudrate: 1200, 9600, 115200
 */
void uart_init(uint32_t baudrate);


/* ========================================================================= */
/*                          Polling API (blocking)                           */
/* ========================================================================= */

/*
 * 送一個 byte
 *   等到 TXE = 1 才寫 DR
 *   有 timeout 保護（避免硬體異常卡死）
 */
uart_status_t uart_send_byte(uint8_t b);

/*
 * 送一個 null-terminated 字串
 *   不會自動加換行
 */
uart_status_t uart_send_string(const char *s);

/*
 * 收一個 byte
 *   等到 RXNE = 1 才讀 DR
 *   有 timeout 保護
 */
uart_status_t uart_recv_byte(uint8_t *b);


/* ========================================================================= */
/*                          DMA TX API (non-blocking)                        */
/* ========================================================================= */

/*
 * 非同步送 buf[0..len-1]
 *   - 啟動 DMA2 Stream7 → USART1_DR
 *   - 立刻 return，CPU 不等
 *   - 如果上一筆還在傳，回傳 UART_ERR_BUSY
 *
 * IMPORTANT:
 *   傳輸期間 buf 內容不可改！
 *   不要傳 stack 上的區域變數（除非確定 caller 會等完）。
 */
uart_status_t uart_send_dma(const uint8_t *buf, uint16_t len);

/*
 * 查詢 DMA 是否還在傳
 *   1 = busy（還在傳）
 *   0 = idle（可以發下一筆）
 */
int uart_send_busy(void);

#endif /* DRIVER_USART1_H_ */
