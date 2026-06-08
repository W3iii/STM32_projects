# 實驗七：UART（CPU Polling）

### 概念
UART = **Universal Asynchronous Receiver/Transmitter**
不需要 clock 線，雙方約定好 baud rate 同步。

```
STM32 TX ─── USB-UART bridge ─── PC（Tera Term）
STM32 RX ─── USB-UART bridge ─── PC
GND ─────── GND（共地必須接）
```

### Pin 對應

| Pin  | 功能       | AF 編號 |
|------|------------|---------|
| PA9  | USART1_TX  | AF7 |
| PA10 | USART1_RX  | AF7 |

### 需要開的 Clock

| Register    | Bit  | 外設    |
|-------------|------|---------|
| RCC_AHB1ENR | bit0 | GPIOA   |
| RCC_APB2ENR | bit4 | USART1  |

USART1 掛在 APB2，預設頻率 = 系統 clock = 16MHz（HSI）。

### BRR 計算

```
BRR = f_clk / baud rate

115200 @ 16MHz：
  16,000,000 / 115,200 = 138.88...
  整數部分（mantissa）= 138 = 0x8A
  小數部分 = 0.88 × 16 = 14.08 → 14 = 0xE
  BRR = (138 << 4) | 14 = 0x8AE

9600 @ 16MHz：
  16,000,000 / 9,600 = 1666.67...
  BRR = 0x0683
```

### 關鍵 Register

| Register    | offset | 功能 |
|-------------|--------|------|
| USART1_SR   | 0x00 | 狀態（bit7=TXE（TX 空），bit6=TC（傳輸完成））|
| USART1_DR   | 0x04 | 資料暫存器，寫入即送出 |
| USART1_BRR  | 0x08 | Baud Rate Register |
| USART1_CR1  | 0x0C | 控制（bit13=UE，bit3=TE，bit2=RE）|
| USART1_CR3  | 0x14 | 控制 3（bit7=DMAT，DMA TX enable）|

### 發送流程

```
1. 等 TXE=1（TX data register 空了）
2. 寫資料到 DR
3. 硬體自動把 DR 遷移出去
4. 重複
```

### 完整程式碼

```c
#include <stdint.h>

// RCC
#define RCC_BASE     0x40023800UL
#define RCC_AHB1ENR  (*((volatile uint32_t *)(RCC_BASE + 0x30)))
#define RCC_APB2ENR  (*((volatile uint32_t *)(RCC_BASE + 0x44)))

// GPIOA
#define GPIOA_BASE   0x40020000UL
#define GPIOA_MODER  (*((volatile uint32_t *)(GPIOA_BASE + 0x00)))
#define GPIOA_AFRH   (*((volatile uint32_t *)(GPIOA_BASE + 0x24)))

// USART1
#define USART1_BASE  0x40011000UL
#define USART1_SR    (*((volatile uint32_t *)(USART1_BASE + 0x00)))
#define USART1_DR    (*((volatile uint32_t *)(USART1_BASE + 0x04)))
#define USART1_BRR   (*((volatile uint32_t *)(USART1_BASE + 0x08)))
#define USART1_CR1   (*((volatile uint32_t *)(USART1_BASE + 0x0C)))

void uart_send_char(char c)
{
    while (!(USART1_SR & (1 << 7)));  // 等 TXE=1
    USART1_DR = c;
}

void uart_print(const char *str)
{
    while (*str)
        uart_send_char(*str++);
}

int main(void)
{
    // 開啟 clock
    RCC_AHB1ENR |= (1 << 0);   // GPIOA
    RCC_APB2ENR |= (1 << 4);   // USART1

    // PA9, PA10 設為 AF mode
    GPIOA_MODER &= ~(0x3 << (9 * 2));
    GPIOA_MODER |=  (0x2 << (9 * 2));
    GPIOA_MODER &= ~(0x3 << (10 * 2));
    GPIOA_MODER |=  (0x2 << (10 * 2));

    // PA9=AF7, PA10=AF7（AFRH，pin8~15）
    GPIOA_AFRH &= ~(0xF << ((9 - 8) * 4));
    GPIOA_AFRH |=  (0x7 << ((9 - 8) * 4));
    GPIOA_AFRH &= ~(0xF << ((10 - 8) * 4));
    GPIOA_AFRH |=  (0x7 << ((10 - 8) * 4));

    // USART1 設定
    USART1_CR1 = 0;
    USART1_BRR = 0x0683;               // 9600 @ 16MHz
    USART1_CR1 = (1 << 13) | (1 << 3); // UE + TE

    while (1)
    {
        uart_print("Hello from STM32!\r\n");
        for (volatile int i = 0; i < 1000000; i++);
    }
}
```

### UART 除錯經驗

**症狀：** 送 `'U'` (0x55) 在 Tera Term 顯示出亂碼。

**排除過程：**
```
1. 用 CubeIDE SFR 視窗確認 RCC_CFGR = 0
   → SWS=00 → 系統 clock 是 HSI 16MHz
   → BRR 算式正確，排除 clock 問題

2. 確認 GND 共地已接

3. Baud rate 降到 1200 → Tera Term 正常顯示
   → 結論：3.3V/5V 電平不相容，是硬體問題
```

**根本原因：** CH340 模組是 5V 邏輯，STM32 輸出 3.3V。高速時 CH340 RX 多讀 3.3V 信號在臨界值附近不穩定。1200 bps 每個 bit = 833µs 容錯大；9600 bps 每個 bit = 104µs 容錯小。

**解決方案：** 改用 CP2102（3.3V 邏輯），完美匹配 STM32。

**教訓：** 程式沒錯不代表結果正確，硬體相容性（電平、接線）是另一個維度。Debug 先確認 clock（RCC_CFGR），再確認電平。
