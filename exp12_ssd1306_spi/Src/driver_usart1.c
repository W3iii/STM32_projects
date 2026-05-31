/*
 * driver_usart1.c
 *
 * USART1 driver implementation
 *
 *  Created on: May 19, 2026
 *      Author: xushu
 */

#include "driver_usart1.h"
#include "stm32f407xx.h"
#include "systick.h"

/* ========================================================================= */
/*                          Private constants                                */
/* ========================================================================= */

/* 目前系統 clock 假設值（HSI 16MHz）
 * USART1 在 APB2，APB2 prescaler = 1 → PCLK2 = HCLK = 16MHz
 * 之後切到 PLL 168MHz 時要改這裡（PCLK2 會變成 84MHz）
 */
#define PCLK2_HZ        16000000U

/* Timeout 設定（ms）
 * 與 I2C driver 一樣使用 SysTick 做 timeout，避免 busy-loop 次數
 * 隨 compiler optimization 或系統 clock 改變而失準。
 */
#define USART1_TIMEOUT_MS    100U


/* ========================================================================= */
/*                          Private state                                    */
/* ========================================================================= */

/* DMA 傳輸狀態旗標
 * 1 = idle（可以發下一筆）
 * 0 = busy（DMA 正在傳）
 *
 * volatile 因為會被 ISR 修改
 */
static volatile int dma_idle = 1;


/* ========================================================================= */
/*                          Wait helpers                                     */
/* ========================================================================= */

static usart1_status_t wait_sr_flag(uint32_t flag)
{
    uint32_t start = systick_get_ms();

    while (!(USART1->SR & flag)) {
        if ((systick_get_ms() - start) >= USART1_TIMEOUT_MS) {
            return USART1_ERR_TIMEOUT;
        }
    }

    return USART1_OK;
}


/* ========================================================================= */
/*                          DMA ISR                                          */
/* ========================================================================= */

/*
 * DMA2 Stream7 中斷 handler
 *   - 對應 IRQ_DMA2_Stream7 (IRQ 70)
 *   - 只處理 Transfer Complete (TCIF)
 *   - TODO: 之後可加 TEIF/DMEIF error handling
 *
 * 函式名必須跟 startup file 的 vector table entry 完全一致，
 * 不然 linker 不會把這個 handler 填進中斷向量表。
 */
void DMA2_Stream7_IRQHandler(void)
{
    /* Stream7 的 TCIF 在 HISR bit 27 */
    if (DMA2->HISR & DMA_TCIF_STREAM7) {
        /* 清除 flag（寫 1 清除，跟一般 read-modify-write 反過來）*/
        DMA2->HIFCR = DMA_TCIF_STREAM7;
        dma_idle = 1;
    }
}


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

void usart1_init(uint32_t baudrate)
{
    /* --- 開 clock --- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* GPIOA for PA9/PA10 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;     /* DMA2 for TX */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   /* USART1 */

    /* --- 設定 PA9 (TX), PA10 (RX) 為 AF7 --- */
    /* PA9 MODER */
    GPIOA->MODER &= ~(0x3U << (9 * 2));
    GPIOA->MODER |=  (GPIO_MODER_AF << (9 * 2));
    /* PA10 MODER */
    GPIOA->MODER &= ~(0x3U << (10 * 2));
    GPIOA->MODER |=  (GPIO_MODER_AF << (10 * 2));

    /* PA9 AF7 (AFRH[3:0] for pin 9) */
    GPIOA->AFRH &= ~(0xFU << ((9 - 8) * 4));
    GPIOA->AFRH |=  (GPIO_AF7_USART1 << ((9 - 8) * 4));
    /* PA10 AF7 (AFRH[7:4] for pin 10) */
    GPIOA->AFRH &= ~(0xFU << ((10 - 8) * 4));
    GPIOA->AFRH |=  (GPIO_AF7_USART1 << ((10 - 8) * 4));

    /* --- 計算並設定 BRR ---
     *
     * USARTDIV = PCLK / (16 × baud)
     * BRR     = USARTDIV × 16 = PCLK / baud
     *
     * 加 baudrate/2 是讓整數除法做四捨五入而非捨去，
     * 提高 baud rate 精準度。
     *
     * 例：baud=9600, PCLK=16M
     *     (16000000 + 4800) / 9600 = 1667 = 0x683
     *     拆解：MANTISSA=104, FRACTION=3
     *     實際 baud = 16M / (16 × 104.1875) = 9600.6 ✓
     */
    USART1->CR1 = 0;    /* 確保 disable 才能改 BRR */
    USART1->BRR = (PCLK2_HZ + baudrate / 2) / baudrate;

    /* --- 設定 DMA2 Stream7 給 USART1_TX ---
     * 只設定固定參數（PAR、CR 大部分位元），
     * NDTR 和 M0AR 在每次 uart_send_dma 時才填。
     *
     * 為什麼這裡先設 PAR：
     *   PAR 永遠指向 USART1->DR，不會變，
     *   一次設好可省每次 dma 呼叫的開銷。
     */
    DMA2_Stream7->CR  = 0;                          /* 先 disable + 清零 */
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;      /* peripheral 固定 */

    /* DMA CR 的非變動位元先設好：
     *   Channel 4 (USART1_TX)
     *   Memory → Peripheral
     *   Memory increment
     *   Transfer complete interrupt enable
     * EN bit 不在這裡開，留到 uart_send_dma 啟動
     */

    /* --- 開啟 USART1 ---
     * UE = USART enable
     * TE = TX enable
     * RE = RX enable
     * CR3 DMAT = DMA transmitter enable
     */
    USART1->CR3 = USART_CR3_DMAT;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* --- 開 NVIC ---
     * DMA2_Stream7 IRQ = 70
     * NVIC->ISER[70/32] |= (1 << (70%32))
     * 用 stm32f407xx.h 提供的 macro 更清楚
     */
    NVIC_EnableIRQ(IRQ_DMA2_Stream7);

    /* 初始狀態：DMA 沒在傳 */
    dma_idle = 1;
}


/* ========================================================================= */
/*                          Polling API                                      */
/* ========================================================================= */

usart1_status_t usart1_send_byte(uint8_t b)
{
    /* 等 TXE = 1 (TX register empty)，有 SysTick timeout 保護 */
    usart1_status_t st = wait_sr_flag(USART_SR_TXE);
    if (st != USART1_OK) {
        return st;
    }

    USART1->DR = b;
    return USART1_OK;
}


usart1_status_t usart1_send_string(const char *s)
{
    if (s == 0) {
        return USART1_ERR_PARAM;
    }

    while (*s) {
        usart1_status_t st = usart1_send_byte((uint8_t)*s);
        if (st != USART1_OK) {
            return st;      /* timeout → 提早結束，告訴上層 */
        }
        s++;
    }

    return USART1_OK;
}


usart1_status_t usart1_recv_byte(uint8_t *b)
{
    if (b == 0) {
        return USART1_ERR_PARAM;
    }

    /* 等 RXNE = 1 (RX register not empty)，有 SysTick timeout 保護 */
    usart1_status_t st = wait_sr_flag(USART_SR_RXNE);
    if (st != USART1_OK) {
        return st;
    }

    *b = (uint8_t)(USART1->DR & 0xFF);
    return USART1_OK;
}


/* ========================================================================= */
/*                          DMA TX API                                       */
/* ========================================================================= */

usart1_status_t usart1_send_dma(const uint8_t *buf, uint16_t len)
{
    if (buf == 0 || len == 0) {
        return USART1_ERR_PARAM;
    }

    /* 如果上一筆還沒傳完，拒絕（讓上層自己決定要等或要跳過）*/
    if (!dma_idle) {
        return USART1_ERR_BUSY;
    }

    /* 標記 busy，必須在啟動 DMA 之前
     * 否則：先啟動 → DMA 立刻完成 → ISR set dma_idle=1 → 這裡才 set =0
     *      結果 dma_idle 永遠是 0 (busy)
     */
    dma_idle = 0;

    /* 設定 stream，必須先 disable 才能改 NDTR/M0AR */
    DMA2_Stream7->CR &= ~DMA_CR_EN;
    /* 等真的 disable（硬體 DMA 不是立刻停的），有 SysTick timeout 保護 */
    uint32_t start = systick_get_ms();
    while (DMA2_Stream7->CR & DMA_CR_EN) {
        if ((systick_get_ms() - start) >= USART1_TIMEOUT_MS) {
            dma_idle = 1;
            return USART1_ERR_TIMEOUT;
        }
    }

    /* 清除所有 stream7 flag（不只 TCIF，避免之前的 error flag 殘留）*/
    DMA2->HIFCR = DMA_FLAGS_STREAM7;

    /* 設定本次傳輸 */
    DMA2_Stream7->NDTR = len;
    DMA2_Stream7->M0AR = (uint32_t)buf;

    /* 啟動 DMA：一次寫好整個 CR 比 |= 安全（避免 read-modify-write 競爭）
     *   Channel 4 (USART1_TX)
     *   Direction: Memory → Peripheral
     *   Memory increment
     *   PSIZE/MSIZE = 8-bit (預設 0，不用設)
     *   Transfer complete interrupt enable
     *   Enable
     */
    DMA2_Stream7->CR = DMA_CR_CHSEL(4)
                     | DMA_CR_DIR_M2P
                     | DMA_CR_MINC
                     | DMA_CR_TCIE
                     | DMA_CR_EN;

    return USART1_OK;
}


int usart1_send_busy(void)
{
    return !dma_idle;
}
