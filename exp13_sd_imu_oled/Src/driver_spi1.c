/*
 * spi_driver.c
 *
 * *  SPI1 + DMA2 Stream3 Channel3
 *  ─────────────────────────────────────────────
 *  Architecture:
 *    Public:
 *      spi1_init                 — 全部硬體初始化
 *      spi1_send_byte            — 短資料 polling
 *      spi1_send_buffer          — 大資料 DMA 同步
 *      spi1_send_buffer_async    — 大資料 DMA 非同步
 *      spi1_is_busy / wait_done  — 狀態查詢
 *    Private (static):
 *      spi1_dma_configure        — 設定 DMA 參數
 *      spi1_dma_start            — 啟動 DMA stream
 *      spi1_dma_clear_flags      — 清掉所有 stream 3 flag
 *  ─────────────────────────────────────────────
 * 
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#include "driver_spi1.h"
#include "stm32f407xx.h"

/* DMA2 Stream3 的 LISR/LIFCR bit 位置（RM0090 §10.5.5）
 *   bit 27: TCIF3     bit 22: FEIF3
 *   bit 26: HTIF3
 *   bit 25: TEIF3
 *   bit 24: DMEIF3
 */
#define DMA_FLAG_TCIF3   (1 << 27)
#define DMA_FLAG_HTIF3   (1 << 26)
#define DMA_FLAG_TEIF3   (1 << 25)
#define DMA_FLAG_DMEIF3  (1 << 24)
#define DMA_FLAG_FEIF3   (1 << 22)
#define DMA_FLAG_ALL_S3  (DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | \
                          DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3)

/* DMA 完成 flag：DMA ISR 設、wait_done() 讀
 * volatile 是因為一個寫（ISR）一個讀（main），編譯器不能優化掉 */
static volatile uint8_t s_dma_done = 1;   // 初始 = 1（沒在傳）

/* ============================================================================
 *                              初始化
 * ============================================================================ */

void spi1_init(void)
{
    /* ---- 1. GPIO: PA5/PA6/PA7 AF5 ---- */
    RCC->AHB1ENR |= (1 << 0);                       // GPIOA clock
    GPIOA->MODER &= ~((0x3 << (5*2)) | (0x3 << (6*2)) | (0x3 << (7*2)));
    GPIOA->MODER |=  ((0x2 << (5*2)) | (0x2 << (6*2)) | (0x2 << (7*2))); // AF
    GPIOA->AFRL &= ~((0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    GPIOA->AFRL |=  ((0x5 << (5*4)) | (0x5 << (6*4)) | (0x5 << (7*4))); // AF5

    // PA6 MISO 加上拉
    GPIOA->PUPDR &= ~(0x3U << (6 * 2));
    GPIOA->PUPDR |=  (0x1U << (6 * 2));   // pull-up

    /* ---- 2. SPI1 ---- */
    RCC->APB2ENR |= (1 << 12);                      // SPI1 clock
    SPI1->CR1 = 0;
    SPI1->CR1 |= (1 << 2);                          // MSTR
    SPI1->CR1 |= (1 << 9) | (1 << 8);               // SSM + SSI（軟體 NSS）
    SPI1->CR1 |= (3 << 3);                          // BR = fPCLK/16
    SPI1->CR2 = 0;
    SPI1->CR1 |= (1 << 6);                          // SPE

    /* ---- 3. DMA2 Stream3 Channel3 ---- */
    RCC->AHB1ENR |= (1 << 22);                      // DMA2 clock

    DMA2_Stream3->CR = 0;                           // 確保 disabled
    while (DMA2_Stream3->CR & (1 << 0));            // 等真的關掉

    /* 固定設定（每次 transfer 不會變的） */
    DMA2_Stream3->CR |= (3 << 25);                  // CHSEL = 3
    DMA2_Stream3->CR |= (0 << 13);                  // MSIZE = 8-bit
    DMA2_Stream3->CR |= (0 << 11);                  // PSIZE = 8-bit
    DMA2_Stream3->CR |= (1 << 10);                  // MINC = 1
    DMA2_Stream3->CR |= (0 << 9);                   // PINC = 0
    DMA2_Stream3->CR |= (0 << 8);                   // CIRC = 0
    DMA2_Stream3->CR |= (1 << 6);                   // DIR = M2P
    DMA2_Stream3->CR |= (1 << 4);                   // TCIE = 1

    /* PAR 永遠是 SPI1->DR */
    DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;

    /* SPI 啟用 DMA TX 請求 */
    SPI1->CR2 |= (1 << 1);                          // TXDMAEN

    /* NVIC 開啟 DMA2 Stream3 IRQ (#59) */
    NVIC->ISER[59 / 32] |= (1U << (59 % 32));
}

/* ============================================================================
 *                              Private helpers
 * ============================================================================ */

static void spi1_dma_clear_flags(void)
{
    DMA2->LIFCR = DMA_FLAG_ALL_S3;
}

static void spi1_dma_configure(const uint8_t *buf, uint32_t len)
{
    /* 改 NDTR / M0AR 前必須 disabled */
    DMA2_Stream3->CR &= ~(1 << 0);
    while (DMA2_Stream3->CR & (1 << 0));

    spi1_dma_clear_flags();

    DMA2_Stream3->M0AR = (uint32_t)buf;
    DMA2_Stream3->NDTR = len;
}

static void spi1_dma_start(void)
{
    s_dma_done = 0;
    DMA2_Stream3->CR |= (1 << 0);                   // EN
}

/* ============================================================================
 *                              Public API
 * ============================================================================ */

void spi1_send_byte(uint8_t data)
{
    SPI1->DR = data;                     // 1. 先把 data 塞進去

    while (!(SPI1->SR & (1U << 1)));     // 2. 等 TXE = 1（DR 又空了）
    while (SPI1->SR & (1U << 7));        // 3. 等 BSY = 0（SPI 真的傳完）

    (void)SPI1->DR;                      // 4. 讀 DR 清 RXNE
}

/* 全雙工：送一個 byte，同時收回 MISO 上的一個 byte
 * SD 卡都是「送命令 / 收回應」成對動作，所以一定要讀 MISO。
 * 寫 DR 觸發 8 clock → RXNE=1 代表一個 frame 收完 → 讀 DR 取回收到的值 */
uint8_t spi1_transfer_byte(uint8_t data)
{
    SPI1->DR = data;                     // 1. 寫 DR 啟動 8 clock
    while (!(SPI1->SR & (1U << 0)));     // 2. 等 RXNE = 1（收到一個 byte）
    return (uint8_t)SPI1->DR;            // 3. 讀 DR：取回 MISO 值 + 清 RXNE
}

/* 切換 SPI 鮑率（BR = CR1[5:3]）
 * SD 卡初始化規定 ≤ 400kHz，本系統 PCLK2 = 16MHz：
 *   br = 5 → /64  = 250kHz（init 用）
 *   br = 2 → /8   = 2MHz  （init 完成後可加速）
 * 改 BR 必須在 SPI 沒在傳的時候，這裡先關 SPE 再改再開最保險。 */
void spi1_set_baud(uint8_t br)
{
    while (SPI1->SR & (1U << 7));        // 等 BSY = 0
    SPI1->CR1 &= ~(1U << 6);             // 關 SPE
    SPI1->CR1 &= ~(0x7 << 3);            // 清掉舊 BR
    SPI1->CR1 |= ((br & 0x7) << 3);      // 設新 BR
    SPI1->CR1 |= (1U << 6);              // 開 SPE
}

void spi1_send_buffer(const uint8_t *buf, uint32_t len)
{
    spi1_dma_configure(buf, len);
    spi1_dma_start();
    spi1_wait_done();                                // 內部呼叫 wait_done
}

void spi1_send_buffer_async(const uint8_t *buf, uint32_t len)
{
    spi1_dma_configure(buf, len);
    spi1_dma_start();
    /* 立刻 return，使用者自己 wait_done */
}

int spi1_is_busy(void)
{
    return !s_dma_done;
}

void spi1_wait_done(void)
{
    while (!s_dma_done);                             // ← 你問的那個 polling
    while (SPI1->SR & (1 << 7));                     // 確保 SPI 真的空了
}

/* ============================================================================
 *                              ISR
 * ============================================================================ */

void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA_FLAG_TCIF3) {
        DMA2->LIFCR = DMA_FLAG_TCIF3;               // 清 flag
        s_dma_done = 1;                              // 通知 wait_done
    }
}