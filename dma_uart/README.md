# 實驗八：DMA + UART TX

### 跟前面實驗的差異

```
實驗六（DMA）：memory → memory，軟體觸發，EN=1 就開始搬完
實驗七（UART）：CPU polling，一個 byte 一個 byte 等
實驗八（合體）：memory → USART1_DR，硬體觸發，CPU 完全不佔用
```

### 為什麼是 DMA2 Stream7 Channel4

查 RM0090 Table 43：
```
USART1_TX → DMA2, Stream 7, Channel 4
```
這是晶片設計時就固定的硬體連線，沒得改。

### 硬體連線關係

```
SRAM buffer[] ──DMA2──→ USART1_DR ──PA9──→ CP2102 ──→ PC Tera Term
      ↑                      ↑
   來源（memory）        目的（peripheral）
      位址遞增              位址固定
```

### DMAT 的作用

DMAT = DMA Transmit enable，在 USART1_CR3 的 bit7。

```
DMAT = 0 → TXE=1 時什麼都不做，等 CPU 來寫 DR
DMAT = 1 → TXE=1 時自動向 DMA 發出搬移請求
```

只有支援 DMA 的外設才有這類控制位：
```
USART → DMAT / DMAR（CR3）
SPI   → TXDMAEN / RXDMAEN（CR2）
I2C   → DMAEN（CR2）
ADC   → DDS + DMA（CR2）
TIM   → 各種 DDE（DIER）
```

### MSIZE/PSIZE 不用設定的原因

```
MSIZE bits[12:11] = 00 → 8-bit（reset 預設）
PSIZE bits[14:13] = 00 → 8-bit（reset 預設）
```
UART 一次傳 1 byte，預設剛好。好習慣是明確寫出來，即使是 0。

### 執行流程

```
CPU:  uart_dma_send() → 設定 DMA → 啟動 → return → CPU 做其他事
                                       ↓
DMA:                           USART1 TXE → 搬 1 byte
                               USART1 TXE → 搬 1 byte
                               ...
                               NDTR = 0 → Transfer Complete → IRQ
                                       ↓
ISR:                           dma_done = 1 → 下次可再發送
```

### 完整程式碼

```c
#include <stdint.h>
#include <string.h>

// RCC
#define RCC_BASE      0x40023800UL
#define RCC_AHB1ENR   (*((volatile uint32_t *)(RCC_BASE + 0x30)))
#define RCC_APB2ENR   (*((volatile uint32_t *)(RCC_BASE + 0x44)))

// GPIOA
#define GPIOA_BASE    0x40020000UL
#define GPIOA_MODER   (*((volatile uint32_t *)(GPIOA_BASE + 0x00)))
#define GPIOA_AFRH    (*((volatile uint32_t *)(GPIOA_BASE + 0x24)))

// USART1
#define USART1_BASE   0x40011000UL
#define USART1_SR     (*((volatile uint32_t *)(USART1_BASE + 0x00)))
#define USART1_DR     (*((volatile uint32_t *)(USART1_BASE + 0x04)))
#define USART1_BRR    (*((volatile uint32_t *)(USART1_BASE + 0x08)))
#define USART1_CR1    (*((volatile uint32_t *)(USART1_BASE + 0x0C)))
#define USART1_CR3    (*((volatile uint32_t *)(USART1_BASE + 0x14)))

// DMA2
#define DMA2_BASE     0x40026400UL
#define DMA2_HISR     (*((volatile uint32_t *)(DMA2_BASE + 0x04)))
#define DMA2_HIFCR    (*((volatile uint32_t *)(DMA2_BASE + 0x0C)))
#define DMA2_S7CR     (*((volatile uint32_t *)(DMA2_BASE + 0x10 + 0x18 * 7)))
#define DMA2_S7NDTR   (*((volatile uint32_t *)(DMA2_BASE + 0x14 + 0x18 * 7)))
#define DMA2_S7PAR    (*((volatile uint32_t *)(DMA2_BASE + 0x18 + 0x18 * 7)))
#define DMA2_S7M0AR   (*((volatile uint32_t *)(DMA2_BASE + 0x1C + 0x18 * 7)))

// NVIC
#define NVIC_ISER2    (*((volatile uint32_t *)(0xE000E108UL)))

// SysTick
#define SYSTICK_CTRL  (*((volatile uint32_t *)(0xE000E010UL)))
#define SYSTICK_LOAD  (*((volatile uint32_t *)(0xE000E014UL)))
#define SYSTICK_VAL   (*((volatile uint32_t *)(0xE000E018UL)))

volatile uint32_t tick = 0;
volatile uint8_t dma_done = 1;  // 1 = 閒置可發送

void SysTick_Handler(void) { tick++; }

void delay_ms(uint32_t ms)
{
    uint32_t s = tick;
    while ((tick - s) < ms);
}

// DMA2 Stream7 Transfer Complete ISR
void DMA2_Stream7_IRQHandler(void)
{
    if (DMA2_HISR & (1 << 27))    // TCIF7 = bit27
    {
        DMA2_HIFCR |= (1 << 27); // 清除 TCIF7
        dma_done = 1;             // 標記傳輸完成
    }
}

void uart_dma_send(const char *str)
{
    uint16_t len = strlen(str);

    while (!dma_done);            // 等上一次傳完
    dma_done = 0;

    // 確保 stream 關閉後再設定
    DMA2_S7CR &= ~(1 << 0);              // disable EN
    while (DMA2_S7CR & (1 << 0));         // 等 EN 確實歸零

    DMA2_HIFCR |= (0xF << 22);           // 清除 Stream7 所有 flag

    DMA2_S7NDTR = len;
    DMA2_S7PAR  = (uint32_t)&USART1_DR;
    DMA2_S7M0AR = (uint32_t)str;

    DMA2_S7CR = (4 << 25)    // Channel 4（USART1_TX）
              | (1 << 6)     // DIR = memory → peripheral
              | (1 << 10)    // MINC = memory 位址遞增
              | (1 << 4)     // TCIE = Transfer Complete 中斷
              | (1 << 0);    // EN = 啟動
}

int main(void)
{
    // SysTick
    SYSTICK_LOAD = 16000 - 1;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = 0x7;

    // Clock enable
    RCC_AHB1ENR |= (1 << 0);     // GPIOA
    RCC_AHB1ENR |= (1 << 22);    // DMA2
    RCC_APB2ENR |= (1 << 4);     // USART1

    // PA9 = AF7 (TX), PA10 = AF7 (RX)
    GPIOA_MODER &= ~(0x3 << (9 * 2));
    GPIOA_MODER |=  (0x2 << (9 * 2));
    GPIOA_MODER &= ~(0x3 << (10 * 2));
    GPIOA_MODER |=  (0x2 << (10 * 2));
    GPIOA_AFRH  &= ~(0xF << 4);
    GPIOA_AFRH  &= ~(0xF << 8);
    GPIOA_AFRH  |=  (0x7 << 4);    // PA9 = AF7
    GPIOA_AFRH  |=  (0x7 << 8);    // PA10 = AF7

    // USART1: 9600 baud, TE, UE, DMAT
    USART1_CR1 = 0;
    USART1_BRR = 0x0683;                     // 16MHz / 9600
    USART1_CR3 = (1 << 7);                   // DMAT，開啟 TX DMA 請求
    USART1_CR1 = (1 << 13) | (1 << 3);       // UE | TE

    // NVIC: DMA2_Stream7 IRQ = 70 → ISER2 bit(70-64) = bit6
    NVIC_ISER2 |= (1 << 6);

    // 開機訊息
    uart_dma_send("DMA+UART ready!\r\n");

    uint32_t count = 0;
    char buf[32];

    while (1)
    {
        count++;
        buf[0] = 'c'; buf[1] = 'n'; buf[2] = 't';
        buf[3] = '=';
        buf[4] = '0' + (count / 100) % 10;
        buf[5] = '0' + (count / 10) % 10;
        buf[6] = '0' + count % 10;
        buf[7] = '\r'; buf[8] = '\n'; buf[9] = '\0';

        uart_dma_send(buf);
        delay_ms(500);
    }
}
```

### uart_dma_send 的重點

1. **先等上一次傳完**（`while (!dma_done)`），避免覆蓋正在傳的 buffer
2. **關閉 EN 後再設定**，DMA 要在 EN=1 時才不能改 register
3. **清除 flag**，上一次的 TCIF 不清掉，下一次中斷不會觸發
4. **SxCR 用直接賦值**（`=`），不用 `|=`，因為先歸零過了

### DMA + UART 的速度瓶頸

DMA 本身搬運速度很快，但受限於 USART 的 baud rate：
```
9600 bps → 每秒約 960 bytes（10 bit/byte）
115200 bps → 每秒約 11,520 bytes
```
DMA 再快也要等 USART 把每個 byte 送完才能傳下一個。
但 CPU 在這段時間是**完全自由**的，這才是 DMA 的效益。

---

## struct 風格 Header（stm32f407xx.h）

### 為什麼從 #define 升級到 struct

```c
// 舊寫法（每個 register 單獨 define，容易打錯 offset）
#define USART1_SR  (*((volatile uint32_t *)(0x40011000)))
#define USART1_DR  (*((volatile uint32_t *)(0x40011004)))

// 新寫法（struct 讓 compiler 算 offset）
USART1->SR    // compiler 自動算 0x40011000 + 0x00
USART1->DR    // compiler 自動算 0x40011000 + 0x04
```

兩者結果完全相同，struct 只是讓 compiler 幫你算 offset。

### NVIC 便利公式

```c
// 手動算
NVIC_ISER2 |= (1 << 6);  // IRQ70 → ISER[70/32] bit(70%32)

// 用 header 的 NVIC struct
NVIC->ISER[IRQn / 32] |= (1 << (IRQn % 32));
```
