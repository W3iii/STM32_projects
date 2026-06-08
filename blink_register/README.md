# 實驗一：Blink LED（純 Register）

### 板子 LED 腳位
| 標示 | 顏色 | Pin  |
|-----|------|------|
| LD4 | 綠   | PD12 |
| LD3 | 橙   | PD13 |
| LD5 | 紅   | PD14 |
| LD6 | 藍   | PD15 |

### 關鍵 Register
| Register    | 位址       | 功能 |
|-------------|------------|------|
| RCC_AHB1ENR | 0x40023830 | 開 GPIOD clock（bit3）|
| GPIOD_MODER | 0x40020C00 | 設定 pin 方向（每 pin 佔 2 bits）|
| GPIOD_ODR   | 0x40020C14 | 輸出資料暫存器 |

### MODER 編碼
```
00 → input
01 → output
10 → alternate function
11 → analog
```
PD12 在 bit[25:24]，公式：`pin N → bit[2N+1 : 2N]`

### 完整程式碼
```c
#include <stdint.h>

#define RCC_BASE     0x40023800UL
#define RCC_AHB1ENR  (*((volatile uint32_t *)(RCC_BASE + 0x30)))

#define GPIOD_BASE   0x40020C00UL
#define GPIOD_MODER  (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_ODR    (*((volatile uint32_t *)(GPIOD_BASE + 0x14)))

int main(void)
{
    RCC_AHB1ENR |= (1 << 3);                   // 開 GPIOD clock

    GPIOD_MODER &= ~(0x3 << (12 * 2));         // 清除 PD12 mode
    GPIOD_MODER |=  (0x1 << (12 * 2));         // PD12 = output

    while (1)
    {
        GPIOD_ODR ^= (1 << 12);                // toggle PD12
        for (volatile int i = 0; i < 1000000; i++);
    }
}
```
