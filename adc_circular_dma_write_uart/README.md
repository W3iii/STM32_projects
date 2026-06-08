# 實驗九 / 十：ADC + Circular DMA + UART

本專案實作兩個遞進版本：
- **實驗九**：ADC + Circular DMA + UART CPU Polling 輸出
- **實驗十**：ADC + Circular DMA + UART DMA TX（CPU 不 Polling）

---

## 實驗九：ADC + Circular DMA + UART（CPU Polling）

### 實驗目標
讀 STM32 內部溫度感測器的讀值，每秒透過 UART 送到 PC：
```
Temp: 28.3 C
Temp: 28.4 C
Temp: 28.3 C
```

### 完整資料流
```
[內部溫度感測器]
       ↓  類比信號
       ↓
[ADC1 CH16]                            硬體自動採樣（continuous mode）
       ↓  12-bit value
       ↓
[DMA2 Stream0 Channel 0, Circular]    硬體自動搬移
       ↓  16-bit per sample
       ↓
[adc_buf[16] in SRAM]                  循環覆寫，永遠最新
       ↓
       ↓  CPU 隨時來讀
       ↓
[CPU 軟體]
       ↓  算平均、換算溫度
       ↓
[uart_send_char × N]                   → CPU polling，卡住等 TXE
       ↓
[USART1 TX → PA9 → CP2102 → PC]
```

### 關鍵設計：三段 CPU/硬體分工
```
階段一（純硬體，CPU 完全不佔用）：
  ADC 不斷轉換 → 每等觸發 DMA → DMA 寫入 adc_buf → wrap-around 繼續

階段二（CPU 軟體運算）：
  讀 adc_buf → 算 16 筆平均 → 12-bit value → mV → 0.1°C

階段三（CPU polling）：
  for each char in "Temp: 28.3 C\r\n":
    while (!(USART1->SR & TXE));
    USART1->DR = c;
```

### 內部溫度感測器

STM32F407 內建一個溫度感測器，接到 ADC1 的 channel 16。

**特性：**
- 25°C 時輸出約 0.76V（V25）
- 靈敏度約 2.5 mV/°C（Avg_Slope）
- 採樣時間至少 10µs（CH16 內阻大）

**換算公式（Datasheet Section 5.3.21）：**
```
Temperature (°C) = (V_sense - V25) / Avg_Slope + 25
                 = (V_sense - 0.76) / 0.0025 + 25
```

整數運算版本（避免 float，單位 0.1°C）：
```
vsense_mV = ADC * 3300 / 4095
temp_x10  = (vsense_mV - 760) * 10 / 25 + 250
```

### ADC 設定要點

**1. 啟用內部溫度感測器**
```c
ADC_COMMON->CCR |= (1 << 23);   // TSVREFE = 1
```
不啟用的話，CH16 讀到的是亂數。

**2. 採樣時間要夠長（至少 10µs）**
```c
ADC clock = APB2 / 2 = 16MHz / 2 = 8MHz
480 cycles × 0.125µs = 60µs（足夠）
ADC1->SMPR1 |= (0x7 << ((16-10)*3));   // CH16 = 480 cycles
```

**3. 序列只能 1 個 channel**
```c
ADC1->SQR1 &= ~(0xF << 20);   // L = 0（只轉 1 次）
ADC1->SQR3 = 16;               // 第一個轉 CH16
```

**4. Continuous mode + DMA**
```c
ADC1->CR2 |= (1 << 0);   // ADON
ADC1->CR2 |= (1 << 1);   // CONT（連續轉）
ADC1->CR2 |= (1 << 8);   // DMA = 1
ADC1->CR2 |= (1 << 9);   // DDS = 1（每次轉完都發 DMA 請求）
```

### Circular DMA 設定要點

**為什麼用 circular：**
ADC 連續採樣 → 永遠不停 → DMA 也永遠不停 → 用 circular 自動 wrap-around。
不需要 CPU 介入重設 NDTR，也不需要 TC 中斷。

**為什麼 buffer 要 16 筆：**
平均後降噪。讀 16 筆取平均可以濾掉隨機雜訊。buffer 越大平均越穩定，但延遲也越長。

**資料寬度：**
- ADC_DR 是 12-bit 資料放在 16-bit register → PSIZE = 16-bit
- adc_buf 是 `uint16_t[]` → MSIZE = 16-bit

```c
DMA2_Stream0->CR |= (1 << 8);        // CIRC = 1 ← 關鍵
DMA2_Stream0->CR |= (0x1 << 11);     // PSIZE = 16-bit
DMA2_Stream0->CR |= (0x1 << 13);     // MSIZE = 16-bit
DMA2_Stream0->CR |= (1 << 10);       // MINC = 1
DMA2_Stream0->CR &= ~(1 << 9);       // PINC = 0（ADC_DR 位址固定）
```

### Polling 的成本

```
送出 "Temp: 28.3 C\r\n"（15 bytes）@ 1200 baud
每 byte = 10 bits（1 start + 8 data + 1 stop）→ 8.3ms
總共 → 125ms

這 125ms 內 CPU 都卡在：
  while (!(USART1->SR & (1 << 7)));
```

CPU 16MHz × 125ms = 200 萬個指令週期被浪費。這就是實驗十要解決的問題。

---

## 實驗十：ADC + Circular DMA + UART DMA TX（CPU 不 Polling）

### 改良範圍
保持實驗九的 ADC + Circular DMA 部分**完全不動**，只改 UART 輸出：
```
實驗九：CPU 一個 byte 一個 byte 寫 USART_DR（卡 125ms）
實驗十：CPU memcpy 進 buffer → 啟動 DMA → 立刻 return（卡 < 1µs）
```

### 兩個 DMA Stream 同時運作
```
DMA2 Stream0：ADC → adc_buf       （Circular，永遠在跑）
DMA2 Stream7：tx_buf → USART_DR   （Normal，每次手動觸發）
```

兩個 stream 各自獨立，互不干擾，CPU 兩端都不佔用。

### 對比表
| 項目 | Stream0 (ADC RX) | Stream7 (UART TX) |
|------|------------------|---------------------|
| 方向 | Peripheral → Memory | Memory → Peripheral |
| Mode | Circular | Normal（送完就停）|
| 資料寬度 | 16-bit | 8-bit |
| TC 中斷 | 不需要（永遠不結束）| 必須（CPU 要回收 buffer）|
| Channel | 0（ADC1）| 4（USART1_TX）|

### 為什麼 UART TX 需要 NVIC，ADC RX 不需要

| 情境 | CPU 需要被通知嗎 | NVIC |
|------|------------------|------|
| ADC circular | 不需要（buffer 永遠最新，CPU 隨時讀）| 不開 |
| UART TX normal | 需要（buffer 要回收，不能在傳輸中被覆寫）| 開 |

### 關鍵設計：Buffer 的生命週期

UART TX DMA 啟動後函式立刻 return，但 DMA 還在背景搬。
**如果搬的是別人的 buffer，函式 return 後那塊記憶體被覆寫，DMA 就讀到錯誤資料。**

**錯誤寫法：**
```c
void uart_dma_send(const char *str, uint16_t len) {
    DMA2_Stream7->M0AR = (uint32_t)str;   // ← 危險
    DMA2_Stream7->CR  |= (1 << 0);
}

// 呼叫者
char local[20] = "hello";
uart_dma_send(local, 5);   // local 在 stack
// ←
// 函式 return 後 local 可能被回收，DMA 還在搬
```

**正確寫法（memcpy pattern）：**
```c
void uart_dma_send(const void *src, uint16_t len) {
    static uint8_t tx_buf[128];           // ← static = 永遠存在

    while (!dma_done);
    dma_done = 0;

    if (len > sizeof(tx_buf)) len = sizeof(tx_buf);
    memcpy(tx_buf, src, len);              // ← 同步複製進來

    DMA2_Stream7->M0AR = (uint32_t)tx_buf; // ← DMA 搬可靠的
    ...
}
```

工業界這個 pattern 叫 **copy on enqueue**，FreeRTOS queue、Linux printk buffer 都是這樣設計。

### API 設計：const void *

```c
void uart_dma_send(const void *src, uint16_t len);
                   ^^^^^^^^^^^^^^^
                   const = 函式承諾不修改 src
                   void* = 接受任何型別的指標（最寬鬆）
```

### CPU 與 DMA 的同步：dma_done flag

```c
volatile uint32_t dma_done = 1;   // 初始 1，第一次呼叫才能進去

void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & (1 << 27)) {       // TCIF7
        DMA2->HIFCR |= (1 << 27);       // 寫 1 清 flag
        dma_done = 1;
    }
}

void uart_dma_send(...) {
    while (!dma_done);                   // 等上次傳完
    dma_done = 0;                        // 標記為「正在傳」
    ...
}
```

**volatile 的必要性：**
ISR 改的變數、main loop 讀的變數都要 volatile。否則編譯器會把 `while (!dma_done)` 優化成「dma_done 不變，直接死迴圈」。

### TCIF flag 操作：寫 1 清

```c
DMA2->HIFCR |= (1 << 27);   // ✓ 寫 1 清 TCIF7
DMA2->HIFCR &= ~(1 << 27);  // ✗ 錯誤觀念
```

**為什麼是「寫 1 清」：**
這是硬體的常見設計。CPU 不能直接「清掉」flag，因為 CPU 寫入瞬間硬體可能正在設置那個 flag，有 race。寫 1 清是硬體層的 atomic 操作。EXTI_PR、USART_SR 的某些 flag 也是這個機制。

### 完整程式碼

```c
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

#define BUF_SIZE 16
volatile uint16_t adc_buf[BUF_SIZE];
volatile uint32_t tick = 0;
volatile uint32_t dma_done = 1;

void SysTick_Handler(void) { tick++; }

void delay_ms(uint32_t ms) {
    uint32_t start = tick;
    while ((tick - start) < ms);
}

void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & (1 << 27)) {       // TCIF7
        DMA2->HIFCR |= (1 << 27);
        dma_done = 1;
    }
}

void uart_dma_send(const void *src, uint16_t len) {
    static uint8_t tx_buf[128];

    while (!dma_done);
    dma_done = 0;

    if (len > sizeof(tx_buf)) len = sizeof(tx_buf);
    memcpy(tx_buf, src, len);

    DMA2_Stream7->CR &= ~(1 << 0);
    while (DMA2_Stream7->CR & (1 << 0));

    DMA2->HIFCR |= (0x3F << 22);

    DMA2_Stream7->NDTR = len;
    DMA2_Stream7->PAR  = (uint32_t)&USART1->DR;
    DMA2_Stream7->M0AR = (uint32_t)tx_buf;

    DMA2_Stream7->CR = (4 << 25)              // CHSEL = 4
                     | (1 << 6)                // DIR = M2P
                     | (1 << 10)               // MINC
                     | (1 << 4)                // TCIE
                     | (1 << 0);               // EN
}

void uart_dma_send_str(const char *str) {
    uart_dma_send(str, strlen(str));
}

void uart_print_temp(int32_t temp_x10) {
    uart_dma_send_str("Temp: ");
    // ...（整數轉字串 + 小數點輸出）
}

int main(void) {
    // systick / rcc / gpio / dma_adc / adc / uart init...

    USART1->CR3 |= (1 << 7);               // DMAT
    NVIC->ISER[2] |= (1 << 6);             // DMA2_Stream7 IRQ 70

    ADC1->CR2 |= (1 << 30);               // SWSTART

    while (1) {
        uint32_t sum = 0;
        for (int i = 0; i < BUF_SIZE; i++) sum += adc_buf[i];
        uint32_t avg = sum / BUF_SIZE;

        int32_t vsense = avg * 3300 / 4095;
        int32_t temp_x10 = (vsense - 760) * 10 / 25 + 250;

        uart_print_temp(temp_x10);
        delay_ms(1000);
    }
}
```

### Polling vs DMA 對比

| 項目 | 實驗九（Polling）| 實驗十（DMA）|
|------|-----------------|--------------|
| CPU 卡住時間 | 每次 send → 125ms | 每次 send → 1µs（memcpy）|
| Buffer 管理 | 不需要 | 內部 static + memcpy |
| 同步機制 | 不需要 | volatile flag + ISR |
| 程式複雜度 | 簡單 | 複雜 |
| CPU 1 秒可做的事 | 87.5%（卡 125ms）| 99.9%（卡 1µs）|

### 兩種 DMA 使用模式總結

**Pattern A：Peripheral → Memory + Circular（採樣型）**
```
ADC、I2S 麥克風、SPI slave 接資料
特徵：外設不斷產生資料，CPU 隨時讀
配置：CIRC = 1，無 TC 中斷
```

**Pattern B：Memory → Peripheral + Normal（發送型）**
```
UART TX、SPI master 送資料、I2C 送資料
特徵：CPU 準備好資料，DMA 自動送出
配置：CIRC = 0，TC 中斷通知 CPU 完成
```

之後做 SPI（SD 卡）、I2C（OLED）的 DMA，都是這兩個 pattern 的變形。
