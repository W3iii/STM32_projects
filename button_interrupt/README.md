# 實驗三：Button 中斷（EXTI + NVIC）+ 實驗四：Button Debounce

---

## 實驗三：Button 中斷（EXTI + NVIC）

### 板子 Button
| 標示 | Pin | 行為 |
|------|-----|------|
| B1 User Button | PA0 | 按下 = 高電位（上升沿）|

### 中斷流程
```
PA0 電位變化
  ↓
EXTI 硬體偵測到邊沿
  ↓
IMR 允許 → 產生中斷請求
  ↓
NVIC 允許 → 通知 CPU
  ↓
CPU 在指令邊界跳到 EXTI0_IRQHandler
```

### 需要設定的模組
```
SYSCFG  → 告知 EXTI0 監聽哪個 GPIO port（PA/PB/PC...）
EXTI    → 設定觸發邊沿、啟用 IMR
NVIC    → 允許 CPU 接受這個中斷
GPIO    → PA0 設為 input，PD12 設為 output
```

### 關鍵 Register
| Register       | 位址       | 功能 |
|----------------|------------|------|
| SYSCFG_EXTICR1 | 0x40013808 | EXTI0~3 對應哪個 GPIO port |
| EXTI_IMR       | 0x40013C00 | 中斷遮罩（bit=1 才允許觸發）|
| EXTI_RTSR      | 0x40013C08 | 上升沿觸發選擇 |
| EXTI_FTSR      | 0x40013C0C | 下降沿觸發選擇 |
| EXTI_PR        | 0x40013C14 | Pending register（ISR 裡寫 1 清除）|
| NVIC_ISER0     | 0xE000E100 | IRQ0~31 的 enable（EXTI0 = bit6）|
| GPIOA_IDR      | 0x40020010 | 讀 PA0 目前電位 |

### EXTI line 編號規則
```
pin 編號 = EXTI line 編號
PA0 → EXTI line 0 → IMR bit0、RTSR bit0
PA1 → EXTI line 1 → IMR bit1、RTSR bit1
```

### IMR 的作用
預設所有 EXTI line 被遮罩，需要才啟用。可隨時關閉某條 line：
```c
EXTI_IMR &= ~(1 << 0);  // 暫時關閉 EXTI0（初始化保護）
EXTI_IMR |=  (1 << 0);  // 重新啟用
```

### Polling vs 中斷
| | Polling | 中斷 |
|--|---------|------|
| 誰主動 | CPU 主動查 | 硬體主動通知（有 IRQ line）|
| CPU 效率 | 低（一直在查）| 高（有事才處理）|
| 即時性 | 取決於 polling 頻率 | 指令邊界內響應 |

中斷不能直接打斷正在執行的指令，CPU 只在**指令和指令之間**檢查中斷。

### BSRR vs ODR
```c
// ODR：read-modify-write，非 atomic，中斷可能夾入
GPIOD_ODR ^= (1 << 12);

// BSRR：atomic，直接 set/reset，不需要 read
GPIOD_BSRR = (1 << 12);        // set PD12
GPIOD_BSRR = (1 << (12 + 16)); // reset PD12
```

### 完整程式碼
```c
#include <stdint.h>

#define RCC_BASE       0x40023800UL
#define RCC_AHB1ENR    (*((volatile uint32_t *)(RCC_BASE + 0x30)))
#define RCC_APB2ENR    (*((volatile uint32_t *)(RCC_BASE + 0x44)))

#define GPIOA_BASE     0x40020000UL
#define GPIOA_MODER    (*((volatile uint32_t *)(GPIOA_BASE + 0x00)))
#define GPIOA_IDR      (*((volatile uint32_t *)(GPIOA_BASE + 0x10)))

#define GPIOD_BASE     0x40020C00UL
#define GPIOD_MODER    (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_ODR      (*((volatile uint32_t *)(GPIOD_BASE + 0x14)))

#define SYSCFG_BASE    0x40013800UL
#define SYSCFG_EXTICR1 (*((volatile uint32_t *)(SYSCFG_BASE + 0x08)))

#define EXTI_BASE      0x40013C00UL
#define EXTI_IMR       (*((volatile uint32_t *)(EXTI_BASE + 0x00)))
#define EXTI_RTSR      (*((volatile uint32_t *)(EXTI_BASE + 0x08)))
#define EXTI_FTSR      (*((volatile uint32_t *)(EXTI_BASE + 0x0C)))
#define EXTI_PR        (*((volatile uint32_t *)(EXTI_BASE + 0x14)))

#define NVIC_ISER0     (*((volatile uint32_t *)(0xE000E100UL)))

void EXTI0_IRQHandler(void)
{
    EXTI_PR |= (1 << 0);           // 清除 pending flag（必須）

    if (GPIOA_IDR & (1 << 0))      // 上升沿（按下），不動作
    {
    }
    else                            // 下降沿（放開），toggle LED
    {
        GPIOD_ODR ^= (1 << 12);
    }
}

int main(void)
{
    RCC_AHB1ENR |= (1 << 0);       // GPIOA clock
    RCC_AHB1ENR |= (1 << 3);       // GPIOD clock
    RCC_APB2ENR |= (1 << 14);      // SYSCFG clock

    GPIOA_MODER &= ~(0x3 << 0);    // PA0 = input
    GPIOD_MODER &= ~(0x3 << (12 * 2));
    GPIOD_MODER |=  (0x1 << (12 * 2));  // PD12 = output

    SYSCFG_EXTICR1 &= ~(0xF << 0); // EXTI0 = GPIOA

    EXTI_IMR  |= (1 << 0);         // 啟用 EXTI0
    EXTI_RTSR |= (1 << 0);         // 上升沿觸發
    EXTI_FTSR |= (1 << 0);         // 下降沿觸發

    NVIC_ISER0 |= (1 << 6);        // 啟用 EXTI0 中斷（IRQ6）

    while (1) { }
}
```

---

## 實驗四：Button Debounce

### 問題
按鍵機械彈跳現象，短時間內觸發多次中斷：
```
理想：  ──────────────────────────
實際：  ──────────────────────────
              ↑↑↑ 這些彈跳觸發多次中斷
```

### 解法
記錄上次觸發時間，兩次中斷間隔小於 50ms 視為彈跳忽略：
```
第一次中斷 → 記錄 last_tick，執行動作
後續彈跳   → (tick - last_tick) < 50ms → 忽略
真正的按下 → (tick - last_tick) > 50ms → 執行動作
```

### 注意事項
- 不能讓 `tick` 歸零，它是全域時間基準
- 用獨立的 `last_tick` 變數記錄上次觸發時間
- `last_tick` 要宣告為 `volatile`

### 完整程式碼
```c
#include <stdint.h>

// SysTick
#define SYSTICK_BASE  0xE000E010UL
#define SYSTICK_CTRL  (*((volatile uint32_t *)(SYSTICK_BASE + 0x00)))
#define SYSTICK_LOAD  (*((volatile uint32_t *)(SYSTICK_BASE + 0x04)))
#define SYSTICK_VAL   (*((volatile uint32_t *)(SYSTICK_BASE + 0x08)))

volatile uint32_t tick = 0;
volatile uint32_t last_tick = 0;

void SysTick_Handler(void) { tick++; }

void EXTI0_IRQHandler(void)
{
    EXTI_PR |= (1 << 0);

    if ((tick - last_tick) < 50)    // 50ms 內的觸發視為彈跳
        return;

    last_tick = tick;               // 更新上次觸發時間

    if (GPIOA_IDR & (1 << 0))      // 上升沿（按下），不動作
    {
    }
    else                            // 下降沿（放開），toggle
    {
        GPIOD_ODR ^= (1 << 12);
    }
}

int main(void)
{
    SYSTICK_LOAD = 16000 - 1;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = 0x7;

    // （其他初始化同實驗三）
}
```
