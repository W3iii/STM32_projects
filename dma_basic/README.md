# 實驗六：DMA（memory to memory）

### 什麼是 DMA
DMA = **Direct Memory Access**，讓硬體控制器代替 CPU 搬運資料。

```
沒有 DMA：CPU 讀 src → 寫 dst → 讀 src → 寫 dst...（CPU 被綁死）
有 DMA：  CPU 告訴 DMA（從哪搬到哪、搬幾筆）→ DMA 自己搬 → CPU 做其他事
```

DMA 需要知道四件事：
```
1. 來源位址（src）
2. 目標位址（dst）
3. 資料量（幾筆）
4. 觸發源（誰來啟動）
```

---

### STM32F407 DMA 架構

```
DMA1 → 連接 APB1 外設（UART2、SPI2、TIM2~7...）
DMA2 → 連接 APB2 外設（UART1、SPI1、ADC...）
```

每個 DMA 有 8 條 Stream（Stream0~7），每條 Stream 有 8 個 Channel 可選。
每條 Stream 同時只能跑一個 Channel。

**memory to memory 只有 DMA2 支援**，DMA1 不行。

---

### 三種傳輸方向

```
memory → peripheral   （送資料給 UART、SPI）
peripheral → memory   （從 ADC、SPI 收資料）
memory → memory       （記憶體複製）
```

---

### 關鍵 Register

| Register | 功能 |
|----------|------|
| DMA_SxCR   | Stream 控制（方向、資料寬度、enable）|
| DMA_SxNDTR | 資料筆數（Number of Data to Transfer）|
| DMA_SxPAR  | Peripheral 位址（mem→mem 時當 src）|
| DMA_SxM0AR | Memory 位址（mem→mem 時當 dst）|
| DMA_LISR/HISR  | 中斷狀態旗標（Stream0~3 用 LISR，Stream4~7 用 HISR）|
| DMA_LIFCR/HIFCR| 清除中斷旗標 |

### SxCR bit field 重點

```
bit[27:25] = CHSEL   → Channel 選擇（0~7）
bit[7:6]   = DIR     → 傳輸方向（00=P→M, 01=M→P, 10=M→M）
bit[10]    = MINC    → Memory 位址遞增
bit[9]     = PINC    → Peripheral 位址遞增
bit[12:11] = MSIZE   → Memory 資料寬度（00=8bit, 01=16bit, 10=32bit）
bit[14:13] = PSIZE   → Peripheral 資料寬度
bit[4]     = TCIE    → Transfer Complete 中斷 enable
bit[0]     = EN      → 啟動 Stream
```

### TCIF flag 位置

Stream 不同，TCIF 在 ISR/IFCR 裡的 bit 位置不同：
```
Stream 0 → TCIF0 = LISR bit5
Stream 7 → TCIF7 = HISR bit27
```
查 RM0090 DMA 章節的 bit map 確認。

### IRQ 編號

```
DMA2_Stream0 → IRQ 56 → NVIC_ISER1 bit24（56-32=24）
DMA2_Stream7 → IRQ 70 → NVIC_ISER2 bit6 （70-64=6）
```

### 完整程式碼

```c
#include <stdint.h>

// RCC
#define RCC_BASE      0x40023800UL
#define RCC_AHB1ENR   (*((volatile uint32_t *)(RCC_BASE + 0x30)))

// GPIOD（完成提示 LED）
#define GPIOD_BASE    0x40020C00UL
#define GPIOD_MODER   (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_ODR     (*((volatile uint32_t *)(GPIOD_BASE + 0x14)))

// DMA2 Stream0
#define DMA2_BASE         0x40026400UL
#define DMA2_LISR         (*((volatile uint32_t *)(DMA2_BASE + 0x00)))
#define DMA2_LIFCR        (*((volatile uint32_t *)(DMA2_BASE + 0x08)))
#define DMA2_S0CR         (*((volatile uint32_t *)(DMA2_BASE + 0x10)))
#define DMA2_S0NDTR       (*((volatile uint32_t *)(DMA2_BASE + 0x14)))
#define DMA2_S0PAR        (*((volatile uint32_t *)(DMA2_BASE + 0x18)))
#define DMA2_S0M0AR       (*((volatile uint32_t *)(DMA2_BASE + 0x1C)))

// 來源和目標陣列
uint32_t src[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint32_t dst[8] = {0};

volatile int dma_done = 0;

// DMA2_Stream0 中斷 handler
void DMA2_Stream0_IRQHandler(void)
{
    DMA2_LIFCR |= (1 << 5);    // 清除 TCIF0（bit5）
    dma_done = 1;
}

int main(void)
{
    // 1. 開啟 DMA2 和 GPIOD clock
    RCC_AHB1ENR |= (1 << 22);  // DMA2
    RCC_AHB1ENR |= (1 << 3);   // GPIOD

    // 2. PD12 設為 output
    GPIOD_MODER &= ~(0x3 << (12 * 2));
    GPIOD_MODER |=  (0x1 << (12 * 2));

    // 3. 設定 DMA2 Stream0
    DMA2_S0CR = 0;                  // 先清零確保乾淨

    DMA2_S0PAR   = (uint32_t)src;   // 來源位址
    DMA2_S0M0AR  = (uint32_t)dst;   // 目標位址
    DMA2_S0NDTR  = 8;               // 搬 8 個 word

    DMA2_S0CR |= (0x2 << 6);    // DIR=10，memory to memory
    DMA2_S0CR |= (1 << 10);     // MINC，目標位址自動遞增
    DMA2_S0CR |= (1 << 9);      // PINC，來源位址自動遞增
    DMA2_S0CR |= (0x2 << 11);   // MSIZE，32-bit
    DMA2_S0CR |= (0x2 << 13);   // PSIZE，32-bit
    DMA2_S0CR |= (1 << 4);      // TCIE，transfer complete 中斷

    // 4. 開啟 NVIC DMA2_Stream0（IRQ56）
    NVIC->ISER[1] |= (1 << (56%32));

    // 5. 啟動 DMA
    DMA2_S0CR |= (1 << 0);      // EN

    // 6. 等待完成
    while (!dma_done);

    // 7. 驗證，搬移成功就亮 LED
    int ok = 1;
    for (int i = 0; i < 8; i++)
        if (dst[i] != src[i]) ok = 0;

    if (ok)
        GPIOD_ODR |= (1 << 12);     // 綠 LED 亮

    while (1) { }
}
```

### mem→mem vs mem→peripheral 觸發差異

```
mem → mem：
  EN=1 → DMA 自己立刻搬完 → TCIF=1
  不需要任何 request 信號

mem → peripheral：
  EN=1 → DMA 等待
       → 外設 TXE=1 → DMA 搬 1 byte
       → 外設 TXE=1 → DMA 搬 1 byte
       → ...重複 NDTR 次 → TCIF=1
  每搬一筆都要等外設說「我準備好了」。
```
