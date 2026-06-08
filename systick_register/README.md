# 實驗二：SysTick 精確延遲

### 概念
SysTick 是 Cortex-M4 內部的倒數計時器，不需要任何 GPIO。

```
LOAD = 15999  → 設定倒數目標（16000 次 = 1ms @16MHz）
VAL  = 目前倒數值（硬體自動遞減）
CTRL = 0x7    → bit0=enable, bit1=tickint, bit2=clksource
```

倒數流程：
```
VAL: 15999 → 15998 → ... → 0 → 觸發中斷 → 重新從 15999 開始
每次到 0 = 經過 1ms → SysTick_Handler 執行 → tick++
```

### 硬體 vs 軟體層次
```
硬體：VAL 倒數，每 1ms 到 0，→ 觸發中斷
軟體：tick++，記錄距開機後經過幾 ms，
      delay_ms 靠 tick 計算時間
```

### 關鍵 Register
| Register     | 位址       | 功能 |
|--------------|------------|------|
| SYSTICK_LOAD | 0xE000E014 | 倒數目標值 |
| SYSTICK_VAL  | 0xE000E018 | 目前倒數值 |
| SYSTICK_CTRL | 0xE000E010 | 控制暫存器 |

### 完整程式碼
```c
#include <stdint.h>

#define RCC_BASE     0x40023800UL
#define RCC_AHB1ENR  (*((volatile uint32_t *)(RCC_BASE + 0x30)))

#define GPIOD_BASE   0x40020C00UL
#define GPIOD_MODER  (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_ODR    (*((volatile uint32_t *)(GPIOD_BASE + 0x14)))

#define SYSTICK_BASE  0xE000E010UL
#define SYSTICK_CTRL  (*((volatile uint32_t *)(SYSTICK_BASE + 0x00)))
#define SYSTICK_LOAD  (*((volatile uint32_t *)(SYSTICK_BASE + 0x04)))
#define SYSTICK_VAL   (*((volatile uint32_t *)(SYSTICK_BASE + 0x08)))

volatile uint32_t tick = 0;

void SysTick_Handler(void)
{
    tick++;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = tick;
    while ((tick - start) < ms);
}

int main(void)
{
    SYSTICK_LOAD = 16000 - 1;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = 0x7;

    RCC_AHB1ENR |= (1 << 3);
    GPIOD_MODER &= ~(0x3 << (12 * 2));
    GPIOD_MODER |=  (0x1 << (12 * 2));

    while (1)
    {
        GPIOD_ODR ^= (1 << 12);
        delay_ms(500);
    }
}
```
