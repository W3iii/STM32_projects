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
#include "isr_port.h"          /* ISR 計時掛勾；沒有 isr_port_config.h 時展開成空 */
#include "systick.h"           /* systick_get_ms()：DMA 等待的逾時上限 */
#include "bus_port.h"          /* DMA 傳輸的等待/喚醒（RTOS 中立）*/
#include "dma_mem.h"           /* dma_mem_ok()：buffer 在 CCM 的話 DMA 讀不到 */

/* DMA2 Stream3 的 LISR/LIFCR bit 位置（RM0090 §10.5.5）
 *   bit 27: TCIF3     bit 22: FEIF3
 *   bit 26: HTIF3
 *   bit 25: TEIF3
 *   bit 24: DMEIF3
 */
/* DMA/SPI 等待的逾時上限（ms）。見 spi1_wait_done 的說明。 */
#define SPI1_DMA_TIMEOUT_MS   100U

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
static volatile uint8_t s_dma_err;        // 1 = 這次傳輸 DMA 回報 TEIF

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

    /* SCK(PA5)/MOSI(PA7) 輸出速度設 Medium(01)：預設 Low 在 fast 階段(2.6MHz)
     * 邊緣會被麵包板電容拖圓。但長線無端接不要拉到 Very High，邊緣太快會
     * ringing/串擾到 MISO，反而更糟——Medium 是折衷。
     * MISO(PA6) 是輸入，OSPEEDR 對它無效，不設。 */
    GPIOA->OSPEEDR &= ~((0x3U << (5*2)) | (0x3U << (7*2)));
    GPIOA->OSPEEDR |=  ((0x1U << (5*2)) | (0x1U << (7*2)));   // 01 = Medium

    // PA6 MISO 加上拉
    GPIOA->PUPDR &= ~(0x3U << (6 * 2));
    GPIOA->PUPDR |=  (0x1U << (6 * 2));   // pull-up

    /* ---- 2. SPI1 ---- */
    RCC->APB2ENR |= (1 << 12);                      // SPI1 clock
    SPI1->CR1 = 0;
    SPI1->CR1 |= (1 << 2);                          // MSTR
    SPI1->CR1 |= (1 << 9) | (1 << 8);               // SSM + SSI（軟體 NSS）
    SPI1->CR1 |= (3 << 3);                          // BR = fPCLK/16 = 84M/16 = 5.25MHz（OLED 用，SD init 會再降速）
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
    DMA2_Stream3->CR |= (1 << 4);                   // TCIE = 1（傳輸完成）
    /* TEIE：傳輸錯誤也要進 ISR。沒開的話錯誤發生時 TC 永遠不來，
     * 等待端只能耗完 50ms 逾時才發現，白等 50ms。 */
    DMA2_Stream3->CR |= (1 << 2);                   // TEIE = 1（傳輸錯誤）

    /* PAR 永遠是 SPI1->DR */
    DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;

    /* SPI 啟用 DMA TX 請求 */
    SPI1->CR2 |= (1 << 1);                          // TXDMAEN

    /* --- NVIC：DMA2 Stream3 (#59) ---
     * 優先權必須設 6（>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5）。
     * 原本沒設，吃 NVIC 預設值 0 —— 數值比 MAX_SYSCALL 小代表邏輯優先權更高，
     * 這種 ISR 能打斷 taskENTER_CRITICAL() 與其他 prio-6 的 ISR。本 handler
     * 加了 ISR_PORT_ENTER/EXIT 之後就不能再這樣：cpustats 的
     * s_acc[ISR] += d 是 read-modify-write，被打斷會 lost update；也會讓
     * I2C1 的 ISR 延遲而觸發 OVR。
     * 用 NVIC_SetPriority/EnableIRQ 取代原本手寫的 ISER 位元運算，與
     * driver_usart1 / app_button 的寫法一致。 */
    NVIC_SetPriority(IRQ_DMA2_Stream3, 6);
    NVIC_EnableIRQ(IRQ_DMA2_Stream3);
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
    /* 清掉上一筆殘留的完成通知（前一次逾時、ISR 事後才送達的情況），
     * 否則這次的 spi1_xfer_wait() 會立刻假性返回。必須在 EN 之前做，
     * 不然可能把這一筆自己的通知清掉。 */
    spi1_xfer_reset();

    s_dma_err  = 0;
    s_dma_done = 0;
    DMA2_Stream3->CR |= (1 << 0);                   // EN
}

/* 檢查 buffer → 設定 → 啟動。sync / async 共用的唯一入口。
 *
 * buffer 不在 DMA 碰得到的區域（典型是 task 的 local 陣列，在 CCM）就不
 * 啟動，並把 s_dma_err 設起來：s_dma_done 維持 1，之後的 spi1_wait_done()
 * 不會去等號誌，直接因 s_dma_err 回 -1 —— async 呼叫端就算沒看這裡的
 * 回傳值，wait_done 那一步照樣拿得到失敗。 */
static int spi1_dma_begin(const uint8_t *buf, uint32_t len)
{
    if (!dma_mem_ok(buf, len)) {
        s_dma_err = 1;
        return -1;
    }
    spi1_dma_configure(buf, len);
    spi1_dma_start();
    return 0;
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
 * SD 卡初始化規定 ≤ 400kHz，本系統 PCLK2 = 84MHz（PLL 168MHz）：
 *   br = 7 → /256 = 328kHz（init 用，≤400kHz）
 *   br = 4 → /32  = 2.6MHz（init 完成後加速）
 * 改 BR 必須在 SPI 沒在傳的時候，這裡先關 SPE 再改再開最保險。 */
void spi1_set_baud(uint8_t br)
{
    while (SPI1->SR & (1U << 7));        // 等 BSY = 0
    SPI1->CR1 &= ~(1U << 6);             // 關 SPE
    SPI1->CR1 &= ~(0x7 << 3);            // 清掉舊 BR
    SPI1->CR1 |= ((br & 0x7) << 3);      // 設新 BR
    SPI1->CR1 |= (1U << 6);              // 開 SPE
}

int spi1_send_buffer(const uint8_t *buf, uint32_t len)
{
    if (spi1_dma_begin(buf, len) != 0) {
        return -1;                    /* buffer DMA 讀不到，什麼都沒送 */
    }
    return spi1_wait_done();          /* 逾時時回 -1；呼叫端可忽略 */
}

int spi1_send_buffer_async(const uint8_t *buf, uint32_t len)
{
    return spi1_dma_begin(buf, len);
    /* 立刻 return，使用者自己 wait_done */
}

int spi1_is_busy(void)
{
    return !s_dma_done;
}

/*
 * 等 DMA + SPI 傳完。
 *
 * 兩個等待都必須有上限：這支是在「握著 spi1_lock」的情況下被呼叫的
 * （OLED 更新在鎖內），只要其中一個永遠不返回，抱著鎖的那支 task 就會
 * 永久空轉，所有需要 SPI1 的人（logger 寫 SD）跟著一起死，而且外觀上
 * 系統還「活著」—— CLI 照樣回應，只是資料再也不會被寫出去。
 *
 * 逾時上限抓 100ms：正常一次 1024-byte 傳輸在 5.25MHz 下約 1.6ms，
 * SD 降速到 2.6MHz 時約 3.1ms，100ms 有 30 倍餘裕。
 *
 * 回傳 0 = 正常完成，-1 = 逾時（DMA 已被關掉，SPI1 可以交給下一個使用者）。
 */
int spi1_wait_done(void)
{
    uint32_t start;
    int      failed = 0;

    /* ---- 等 DMA：阻塞，不是忙等 ----
     * 1024 bytes 在 5.25MHz 下約 1.6ms，而一次睡眠/喚醒來回約 3~8µs，
     * 比例 300:1，睡覺明顯划算。DMA2_Stream3 的 ISR 在 TC 或 TE 時
     * spi1_xfer_signal() 把我們叫醒。
     *
     * 為什麼這件事重要：這支是在握著 spi1_lock 的情況下被呼叫的，
     * 原本那 1.6ms 的空轉不只燒 CPU，還把想寫 SD 的 logger 擋在鎖外面。 */
    /* s_dma_done 已經是 1 = 根本沒有傳輸在跑（或 ISR 比我們先跑完），
     * 這時不該去等號誌，否則會空等到逾時。原本忙等版天生有這個語意
     * （初值就是 1），換成阻塞後要明確寫出來。
     * 沒有競態：真的在傳的話 spi1_dma_start() 已經先把它設成 0；若 ISR
     * 剛好在這個檢查之後才 give，xSemaphoreTake 會 latch 住，一樣拿得到。 */
    if (!s_dma_done) {
        if (spi1_xfer_wait() != 0) {
            /* 號誌沒來（DMA 卡住、中斷沒觸發）。關掉 stream 免得它之後才
             * 醒過來動到 buffer，並把 busy 旗標放掉，讓下一次傳輸還能發起。 */
            DMA2_Stream3->CR &= ~(1U << 0);          // EN = 0
            spi1_dma_clear_flags();
            s_dma_done = 1;
            failed     = 1;
        }
    }
    if (s_dma_err) {
        failed = 1;                                  // ISR 收到 TEIF
    }

    /* ---- 等 SPI BSY：維持忙等 ----
     * DMA 的 TC 是在「最後一個 byte 交給 SPI」時觸發，此時 shift register
     * 還在把它推出去，還要一個 byte 時間（5.25MHz 下 1.52µs）。
     * 睡一次要 3~8µs，比等待本身還久 —— 粒度不對，忙等才是對的。
     * 但仍然要有上限，理由同上：這裡卡住就是抱著 spi1_lock 卡住。 */
    start = systick_get_ms();
    while (SPI1->SR & (1 << 7)) {                    // 確保 SPI 真的空了
        if (systick_get_ms() - start >= SPI1_DMA_TIMEOUT_MS) {
            failed = 1;
            break;
        }
    }

    /* TX-only DMA 全程沒讀 RX → 結束時 RXNE/OVR 是髒的。
     * 不清掉的話，下一個共用 SPI1 的裝置（SD 卡的 spi1_transfer_byte）
     * 會讀到這裡殘留的舊 byte，命令/資料流被汙染。
     * 清 OVR 的標準順序：先讀 DR、再讀 SR。 */
    (void)SPI1->DR;
    (void)SPI1->SR;

    return failed ? -1 : 0;
}

/* ============================================================================
 *                              ISR
 * ============================================================================ */

/*
 * DMA2 Stream3（SPI1_TX）完成 / 錯誤中斷。
 *
 * 兩種結果都要喚醒等待的 task —— 錯誤時如果不喚醒，TC 永遠不會來，
 * 等待端只能白白耗完 50ms 逾時才發現。
 *
 * s_dma_done 除了給等待端看，也是 spi1_is_busy() 的依據，所以兩條路徑
 * 都要設。喚醒放在最後，確保 task 醒來時狀態已經一致。
 */
void DMA2_Stream3_IRQHandler(void)
{
    ISR_PORT_ENTER();
    {
        int finished = 0;

        if (DMA2->LISR & DMA_FLAG_TCIF3) {
            DMA2->LIFCR = DMA_FLAG_TCIF3;           // 清 flag
            s_dma_done  = 1;
            finished    = 1;
        }

        if (DMA2->LISR & DMA_FLAG_TEIF3) {
            DMA2->LIFCR = DMA_FLAG_TEIF3;
            /* 傳輸錯誤時硬體已經自動把 EN 清掉了，這裡只記錄並放行。 */
            s_dma_err   = 1;
            s_dma_done  = 1;
            finished    = 1;
        }

        if (finished) {
            spi1_xfer_signal();                     // 喚醒 spi1_wait_done()
        }
    }
    ISR_PORT_EXIT();
}