# 實驗五：TIM PWM 呼吸燈

### 什麼是 PWM
PWM = **Pulse Width Modulation（脈衝寬度調變）**

藉由改變高電位脈衝的寬度來控制平均輸出電壓，進而控制 LED 亮度。

```
100% duty：──────────────  → 全亮
50%  duty：──────────────  → 半亮
10%  duty：──────────────  → 微亮
0%   duty：──────────────  → 全滅
```

人眼視覺暫留約 16ms，PWM 頻率 1kHz 的閃爍速度遠超人眼感知，看起來是連續亮度。

---

### 時脈鏈
```
16MHz（系統 clock）
  ÷ (PSC+1) = ÷16
1MHz（計數頻率，每個 tick = 1µs）
  ÷ (ARR+1) = ÷1000
1kHz（PWM 頻率，每個週期 = 1ms）
```

---

### 三個關鍵參數

| 參數 | 全名 | 功能 |
|------|------|------|
| PSC | Prescaler | 對 clock 分頻，決定計數速度 |
| ARR | Auto Reload Register | 計數週期，決定 PWM 頻率與解析度 |
| CCR | Capture Compare Register | 比較基準，決定 duty cycle |

**比較邏輯：**
```
計數器 < CCR  → 輸出高電位（LED 亮）
計數器 >= CCR → 輸出低電位（LED 滅）

duty cycle = CCR / (ARR + 1)
```

---

### 1 秒呼吸週期時間軸
```
時間:   0ms        500ms       999ms
CCR:    0  →  1  →  2  → ... → 999
duty:   0% → 0.1% → 0.2% → ... → 100%
亮度:   暗 ──────────────────────── 亮

軟體每 1ms 改一次 CCR（delay_ms(1)）
999 步 × 1ms = 999ms → 1 秒漸到亮
加 999ms 亮到暗 → 總共 2 秒一個呼吸
```

---

### ARR 與解析度的關係
```
ARR=9    → 10 個等級（跳躍感明顯）
ARR=99   → 100 個等級（稍微平滑）
ARR=999  → 1000 個等級（非常平滑）
ARR=9999 → 10000 個等級（極度平滑，但 PWM 頻率降低）
```

**設計原則：**
```
PWM 頻率 > 1kHz → 人眼不閃（必要條件）
ARR 越大        → 解析度越高，過渡越平滑
PSC 越小        → 計數越快，可在高頻率下使用大 ARR
```

---

### 硬體 vs 軟體分工
```
TIM4 硬體  → 每 1µs 比較計數器與 CCR，輸出精準 PWM 波形（CPU 不佔用）
SysTick    → 每 1ms 提供時間基準
while(1)   → 每 1ms 改一次 CCR，控制亮度變化速度
人眼       → 看不到 1kHz 閃爍，感受到連續亮度
```

---

### SysTick vs TIM 的比較
| | SysTick | TIM2~TIM14 |
|--|---------|------------|
| 位置 | Cortex-M4 核心內 | STM32 外設 |
| 功能 | 只能計時 | PWM、capture、compare... |
| 輸出到腳位 | 無 | 有，可直接驅動 GPIO |
| 數量 | 1個 | STM32F407 有 14 個 |

---

### 需要設定的模組
```
RCC     → 開 GPIOD clock（AHB1 bit3）+ TIM4 clock（APB1 bit2）
GPIOD   → PD12 設為 AF mode（MODER=10），選 AF2（TIM4_CH1）
TIM4    → PSC、ARR、CCR1、CCMR1（PWM mode 1）、CCER（CH1 enable）、EGR（update）、CR1（start）
```

### 關鍵 Register
| Register | offset | 功能 |
|----------|--------|------|
| TIM4_PSC  | 0x28 | 分頻值 |
| TIM4_ARR  | 0x2C | 計數週期 |
| TIM4_CCR1 | 0x34 | CH1 比較值（duty cycle）|
| TIM4_CCMR1| 0x18 | CH1 輸出模式（PWM mode 1 = 0x6 << 4）|
| TIM4_CCER | 0x20 | CH1 輸出 enable（bit0=1）|
| TIM4_EGR  | 0x14 | Update event（讓 PSC/ARR 生效，bit0=1）|
| TIM4_CR1  | 0x00 | 啟動 Timer（bit0=CEN）|

### PWM mode 1 vs mode 2
```
mode 1：計數 < CCR → 高電位（CCR大=亮，直覺）
mode 2：計數 < CCR → 低電位（CCR大=滅，反向）
```
選 mode 1 因為 LED 高電位亮，CCR 大 = 亮，邏輯自然。

### 完整程式碼
```c
#include <stdint.h>

#define RCC_BASE      0x40023800UL
#define RCC_AHB1ENR   (*((volatile uint32_t *)(RCC_BASE + 0x30)))
#define RCC_APB1ENR   (*((volatile uint32_t *)(RCC_BASE + 0x40)))

#define GPIOD_BASE    0x40020C00UL
#define GPIOD_MODER   (*((volatile uint32_t *)(GPIOD_BASE + 0x00)))
#define GPIOD_AFRH    (*((volatile uint32_t *)(GPIOD_BASE + 0x24)))

#define TIM4_BASE     0x40000800UL
#define TIM4_CR1      (*((volatile uint32_t *)(TIM4_BASE + 0x00)))
#define TIM4_ARR      (*((volatile uint32_t *)(TIM4_BASE + 0x2C)))
#define TIM4_PSC      (*((volatile uint32_t *)(TIM4_BASE + 0x28)))
#define TIM4_CCR1     (*((volatile uint32_t *)(TIM4_BASE + 0x34)))
#define TIM4_CCMR1    (*((volatile uint32_t *)(TIM4_BASE + 0x18)))
#define TIM4_CCER     (*((volatile uint32_t *)(TIM4_BASE + 0x20)))
#define TIM4_EGR      (*((volatile uint32_t *)(TIM4_BASE + 0x14)))

#define SYSTICK_BASE  0xE000E010UL
#define SYSTICK_CTRL  (*((volatile uint32_t *)(SYSTICK_BASE + 0x00)))
#define SYSTICK_LOAD  (*((volatile uint32_t *)(SYSTICK_BASE + 0x04)))
#define SYSTICK_VAL   (*((volatile uint32_t *)(SYSTICK_BASE + 0x08)))

#define PWM_MAX 999

volatile uint32_t tick = 0;
void SysTick_Handler(void) { tick++; }
void delay_ms(uint32_t ms) { uint32_t s = tick; while ((tick - s) < ms); }

int main(void)
{
    SYSTICK_LOAD = 16000 - 1;
    SYSTICK_VAL  = 0;
    SYSTICK_CTRL = 0x7;

    RCC_AHB1ENR |= (1 << 3);   // GPIOD
    RCC_APB1ENR |= (1 << 2);   // TIM4

    GPIOD_MODER &= ~(0x3 << (12 * 2));
    GPIOD_MODER |=  (0x2 << (12 * 2));   // AF mode

    GPIOD_AFRH &= ~(0xF << ((12 - 8) * 4));
    GPIOD_AFRH |=  (0x2 << ((12 - 8) * 4));  // AF2 = TIM4_CH1

    TIM4_PSC  = 15;             // 16MHz / 16 = 1MHz
    TIM4_ARR  = PWM_MAX;        // 1MHz / 1000 = 1kHz
    TIM4_CCR1 = 0;

    TIM4_CCMR1 &= ~(0x7 << 4);
    TIM4_CCMR1 |=  (0x6 << 4); // PWM mode 1
    TIM4_CCMR1 |=  (1 << 3);   // OC1PE preload enable

    TIM4_CCER |= (1 << 0);     // CH1 output enable
    TIM4_EGR  |= (1 << 0);     // update event
    TIM4_CR1  |= (1 << 0);     // start timer

    int ccr = 0;
    int dir = 1;

    while (1)
    {
        ccr += dir;
        if (ccr >= PWM_MAX) dir = -1;
        if (ccr <= 0)       dir = 1;
        TIM4_CCR1 = ccr;
        delay_ms(1);
    }
}
```
