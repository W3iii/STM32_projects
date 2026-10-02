/*
 * driver_usart1.h
 *
 * USART1 driver for STM32F407
 *   - PB6 = TX (AF7)   ← 從 PA9 改來：Discovery PA9/PA10 被 USB-OTG 佔用會亂碼
 *   - PB7 = RX (AF7)   ← 從 PA10 改來
 *   - 預期接 CP2102 USB-UART → Tera Term（115200）
 *
 * TX 有兩種送法：
 *   1. Polling (blocking)     - usart1_send_byte/string，poll TXE，適合 init/debug print
 *   2. DMA      (non-blocking) - usart1_try_send/send，DMA2 Stream7，適合大量資料、log
 *
 * RX 只有一條路徑：DMA2 Stream2 以 circular 模式持續把 USART1->DR 寫進
 * 內部 ring buffer（永遠在跑，不用啟動）。所有讀取 API 都從這個 buffer 取資料，
 * 差別只在 buffer 空時的行為：
 *   - usart1_read_byte      (blocking)     空就等到 timeout
 *   - usart1_try_read_byte  (non-blocking) 空就立刻回 ERR_EMPTY
 *   - usart1_try_read_line  (non-blocking) 沒完整一行就回 ERR_EMPTY
 * USART IDLE line interrupt 會在一筆傳輸結束時設旗標，main loop 可用
 * usart1_rx_idle_pending() 得知「有新資料可 parse」，不必一直 polling。
 *
 * NOTE:
 *   系統 clock = PLL 168MHz（HSE 8MHz 源，見 driver_rcc.c）
 *   USART1 在 APB2 bus，PCLK2 = HCLK/2 = 84MHz
 *
 *   目前 console 用 115200（PCLK2=84MHz 下 BRR=729，誤差 0.02%）。
 *   若 CH340 5V 介面訊號完整性不佳要降速，注意 16-bit BRR 上限讓最低
 *   baud ≈ 1283——1200 已做不到，最近的有效低速是 2400。
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
    USART1_OK = 0,
    USART1_ERR_TIMEOUT,
    USART1_ERR_BUSY,         /* DMA 還在傳，不能發新的 */
    USART1_ERR_PARAM,        /* buf == NULL、len == 0，或 buf 不在 DMA 可存取區 */
    USART1_ERR_EMPTY,        /* RX ring buffer 為空     */
} usart1_status_t;


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

/*
 * 初始化 USART1
 *   - 開 GPIOB / USART1 / DMA2 clock
 *   - PB6/PB7 設為 AF7
 *   - 設定 baudrate（自動計算 BRR）
 *   - 開啟 UE + TE + RE + IDLEIE
 *   - DMA2 Stream7 Channel 4 給 USART1_TX（只設固定參數，不啟動）
 *   - DMA2 Stream2 Channel 4 給 USART1_RX（circular，立刻啟動，永遠跑）
 *   - 開 NVIC：DMA2 Stream2(RX half/full)、Stream7(TX done)、USART1(IDLE)
 *
 * 可重入：二次呼叫（如換 baudrate）會把 RX ring buffer 狀態歸零。
 *
 * 常用 baudrate: 1200, 9600, 115200
 */
void usart1_init(uint32_t baudrate);


/* ========================================================================= */
/*                          Blocking TX API (polling TXE)                    */
/* ========================================================================= */

/*
 * 送一個 byte
 *   等到 TXE = 1 才寫 DR
 *   有 timeout 保護（避免硬體異常卡死）
 */
usart1_status_t usart1_send_byte(uint8_t b);

/*
 * 送一個 null-terminated 字串
 *   不會自動加換行
 */
usart1_status_t usart1_send_string(const char *s);


/* ========================================================================= */
/*                          Non-Blocking TX API (DMA)                        */
/* ========================================================================= */

/*
 * 非同步送 buf[0..len-1]
 *   - 啟動 DMA2 Stream7 → USART1_DR
 *   - 立刻 return，CPU 不等
 *   - 如果上一筆還在傳，回傳 USART1_ERR_BUSY
 *
 * IMPORTANT:
 *   傳輸期間 buf 內容不可改！
 *   buf 必須在 Flash 或主 SRAM（static / 全域 / 字串常數）。task stack 在 CCM，
 *   DMA 讀不到 —— local 陣列一律回 USART1_ERR_PARAM，就算 caller 會等完也不行。
 */
usart1_status_t usart1_try_send(const uint8_t *buf, uint16_t len);

/*
 * 等上一筆傳完（含 timeout），再送新的
 * 適合大多數情況：不需要真正 async，只是不想 polling TXE
 */
usart1_status_t usart1_send(const uint8_t *buf, uint16_t len);

/*
 * 查詢 DMA 是否還在傳
 *   1 = busy（還在傳）
 *   0 = idle（可以發下一筆）
 */
int usart1_send_busy(void);


/* ========================================================================= */
/*                          RX Ring Buffer API (DMA circular)                */
/* ========================================================================= */
/*
 * 以下所有讀取都從 DMA2 Stream2 的 circular buffer 取資料（見檔首說明）。
 * head 由 DMA 的 NDTR 推算，tail 只有 caller 推進。
 *
 * 注意：DMA 不認識 tail，buffer 滿了仍會無聲覆蓋最舊資料。正確性靠
 * 「buffer 夠大 + 每次 IDLE 都盡快 drain」保證，不要長時間不讀。
 */

/*
 * 收一個 byte（阻塞）
 *   ring buffer 空就等到有資料，最多等 USART1_TIMEOUT_MS
 *   有 timeout 保護（避免硬體異常卡死）
 *   USART1_ERR_TIMEOUT = 等到 timeout 仍無資料
 */
usart1_status_t usart1_read_byte(uint8_t *b);

/*
 * 查詢 USART IDLE 事件是否觸發（= 一次資料傳輸結束）
 *   1 = 有；0 = 無。呼叫即清除（read-and-clear）。
 *
 *   if (usart1_rx_idle_pending()) { usart1_try_read_line(...); }
 */
int usart1_rx_idle_pending(void);

/*
 * 註冊 RX 通知 callback（事件驅動用）。
 *   USART IDLE 與 DMA RX(HT/TC) 中斷觸發時，會在 ISR 脈絡呼叫此 callback。
 *   driver 本身不依賴 RTOS：RTOS 版可在 callback 內呼叫 vTaskNotifyGiveFromISR()
 *   喚醒 console task，讓 console 不必每 10ms 輪詢 ring buffer。
 *   傳 NULL 取消註冊。
 *
 *   注意：callback 跑在 ISR 內，必須極短；且承載此 callback 的兩個 IRQ
 *   （USART1 / DMA2_Stream2）優先權已在 usart1_init() 設為 6（>= MAX_SYSCALL），
 *   callback 內才能安全呼叫 FreeRTOS 的 ...FromISR() API。
 */
void usart1_set_rx_callback(void (*cb)(void));

/*
 * 查詢 RX ring buffer 有幾個 byte 可讀
 */
uint16_t usart1_rx_available(void);

/*
 * 取出一個 byte（非阻塞）
 *   buffer 空就立刻回傳，不等待
 *   USART1_ERR_EMPTY = buffer 為空
 */
usart1_status_t usart1_try_read_byte(uint8_t *b);

/*
 * 讀一整行（直到 '\n' 或 buf 滿為止）
 *   結尾會加 '\0'，不包含 '\n'
 *   USART1_ERR_EMPTY  = buffer 還沒有完整一行（不會消耗任何 byte）
 *   USART1_OK         = 成功，*len_out 為字元數（不含 '\0'）
 */
usart1_status_t usart1_try_read_line(char *buf, uint16_t buf_size, uint16_t *len_out);

/*
 * 清空 RX ring buffer
 */
void usart1_rx_flush(void);

#endif /* DRIVER_USART1_H_ */