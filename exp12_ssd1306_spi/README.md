# 實驗十二：SPI DMA + SSD1306 OLED + I2C MPU6050

### 實驗目標
整合 SPI OLED 顯示器與 I2C 加速度感測器，即時顯示 MPU6050 的 X/Y/Z 加速度原始值：
```
Accel raw:
X: 120
Y: -50
Z: 16384
```

SSD1306 畫面透過 **SPI DMA 非同步更新**，I2C 讀取 MPU6050 的 polling 與 DMA 畫面傳輸**平行執行**。

---

### 硬體接線

#### SSD1306 OLED（SPI）
| STM32 | OLED    | 說明 |
|-------|---------|------|
| PA5   | SCK     | SPI1 clock，AF5 |
| PA6   | MISO    | SPI1（OLED 不回傳，可不接）|
| PA7   | MOSI    | SPI1 data，AF5 |
| PA4   | CS      | Chip select，低電位有效 |
| PB0   | RES     | Reset，低電位 reset |
| PB1   | DC      | Data/Command，低=命令，高=資料 |
| 3.3V  | VCC     | 電源 |
| GND   | GND     | 共地 |

#### MPU6050（I2C）
| STM32 | MPU6050 | 說明 |
|-------|---------|------|
| PB6   | SCL     | I2C1 clock，AF4，open-drain |
| PB7   | SDA     | I2C1 data，AF4，open-drain |
| 3.3V  | VCC     | 電源（不要接 5V）|
| GND   | GND     | 共地 |

#### UART（debug 輸出）
| STM32 | CP2102  | 說明 |
|-------|---------|------|
| PA9   | RX      | USART1_TX，AF7 |
| GND   | GND     | 共地 |

---

### 軟體架構

```
main.c
├── systick_init()          → 1ms SysTick 基準
├── spi1_init()             → SPI1 + DMA2 Stream3
├── i2c1_init()             → I2C1 100kHz
├── usart1_init(1200)       → USART1 1200 baud + DMA TX
├── ssd1306_init()          → OLED 初始化序列
├── i2c1_scan()             → 掃描 I2C bus，找連接的裝置
├── mpu6050_init()          → 確認 WHO_AM_I + 喚醒
└── while(1)
    ├── ssd1306_clear()
    ├── ssd1306_draw_string/int()
    ├── ssd1306_update_screen_async()   ← 啟動 DMA，立刻 return
    ├── mpu6050_read_accel()            ← 與 DMA 平行執行
    └── ssd1306_wait_update_done()      ← 等 DMA 完成
```

### 平行執行的關鍵

```
ssd1306_update_screen_async()
  → 設定 DMA2 Stream3 → 拉低 CS/DC → 啟動 DMA → return
                                                    ↓（背景跑）
mpu6050_read_accel()
  → I2C polling 讀 6 bytes                         DMA 同時在傳 OLED 資料

ssd1306_wait_update_done()
  → while (spi1_is_busy());  ← 等 DMA 的 TCIF
```

這樣 I2C 讀取（約 600µs）跟 SPI DMA（約 1ms）可以重疊，提高 frame rate。

---

### 使用的 Driver 模組

| 模組 | 功能 |
|------|------|
| `systick.c/h` | 1ms tick、`delay_ms()` |
| `driver_spi1.c/h` | SPI1 初始化、同步/非同步傳輸、DMA2 Stream3 |
| `driver_i2c1.c/h` | I2C1 初始化、register 讀寫、bus scan、timeout |
| `driver_usart1.c/h` | USART1 初始化、DMA TX、字串輸出 |
| `device_ssd1306.c/h` | OLED framebuffer 操作、畫字/畫整數、同步/非同步推送 |
| `device_mpu6050.c/h` | MPU6050 初始化、讀加速度 |

---

### SPI DMA 配置

DMA2 Stream3 Channel3 → SPI1_TX（查 RM0090 Table 43）

```
PA5=SCK, PA7=MOSI → AF5（SPI1）
PA4=CS, PB0=RES, PB1=DC → GPIO output（手動控制時序）

DMA2_Stream3->CR:
  CHSEL = 3     → Channel 3（SPI1_TX）
  DIR   = M2P   → memory → peripheral
  MINC  = 1     → memory 位址遞增
  PSIZE = 8-bit → SPI 一次傳 1 byte
  MSIZE = 8-bit
  TCIE  = 1     → Transfer Complete 中斷

IRQ: DMA2_Stream3 = IRQ 59 → NVIC->ISER[1] bit(59-32) = bit27
```

---

### SSD1306 Framebuffer

```
128 col × 64 row = 8192 pixel
每個 byte 表示垂直 8 pixel（bit 0 在上，bit 7 在下）
128 × 8 page = 1024 bytes

Index 公式：
  pixel (x, y) → framebuffer[(y / 8) * 128 + x] 的第 (y % 8) 個 bit
```

先在 framebuffer 上做所有繪圖操作，最後一次 `ssd1306_update_screen()` 才推到 OLED。

---

### I2C Bus Scan

```c
uint8_t addrs[8];
int n = i2c1_scan(addrs, 8);
```

掃描 0x01 ~ 0x7E，對每個位址發 START + 位址 + 等 ADDR flag。
ACK 回應 → 裝置存在 → 記錄位址。
NACK（AF flag）→ 無裝置 → 繼續下一個。

---

### 錯誤處理

```c
int mpu_st = mpu6050_init();
if (mpu_st != 0) {
    if (mpu_st == -2) {
        ssd1306_draw_string(0, 0, "I2C BUS STUCK");
        ssd1306_draw_string(0, 16, "Recovery tried");
    } else {
        ssd1306_draw_string(0, 0, "MPU NOT FOUND");
        ssd1306_draw_string(0, 16, "Check wiring");
    }
    ssd1306_update_screen();
    while (1);
}
```

mpu6050_init() 回傳值：
- `0` → 成功
- `-1` → WHO_AM_I 不是 0x68（接線問題或 sensor 壞）
- `-2` → I2C bus stuck（SDA/SCL 被拉死，driver 已嘗試 recovery）

---

### 關鍵 Register 總覽

| 外設 | Clock | GPIO |
|------|-------|------|
| SPI1  | RCC_APB2ENR bit12 | PA5/PA6/PA7 AF5 |
| I2C1  | RCC_APB1ENR bit21 | PB6/PB7 AF4 open-drain |
| USART1| RCC_APB2ENR bit4  | PA9 AF7 |
| DMA2  | RCC_AHB1ENR bit22 | — |

---

### 常見問題

1. **OLED 白屏或無反應** — 確認 CS 初始為 HIGH，RES 有確實低電位 reset 後再高
2. **MPU6050 init 失敗** — 先用 I2C scan 確認 0x68 有回應；PB6/PB7 必須 open-drain + pull-up
3. **畫面撕裂** — `ssd1306_update_screen_async()` 後要呼叫 `ssd1306_wait_update_done()` 才能再修改 framebuffer
4. **UART 亂碼** — 確認 CP2102（3.3V 邏輯），不要用 CH340（5V 邏輯）+ 確認共地
5. **i2c1_scan 卡住** — 需要 timeout 機制，若 bus stuck 才能脫逃；driver 內有 AF flag 逃出路徑

---

## UART RX 設計演進：從 RXNE 中斷到 Circular DMA

這個章節記錄 `driver_usart1.c` 的 RX 接收機制的設計決策過程，包括為什麼選擇某個方案、後來又為什麼換掉，以及兩個方案在技術上的根本差異。

---

### 背景：為什麼需要 UART RX

exp12 的前期只有 UART TX：把感測器資料用 DMA 送出去印在 Tera Term 上，板子是純粹的「廣播者」。

要做 CLI（Command Line Interface），也就是讓 PC 端可以傳指令給板子（例如 `mpu read`、`oled clear`、`rate 10`），就需要 RX。RX 的挑戰在於：資料什麼時候來、來幾個 byte，都是 PC 端決定的，firmware 必須能「隨時準備好接」，又不能讓 CPU 卡在那裡等。

---

### 方案一：RXNE Interrupt Per Byte（第一版，已被取代）

#### 概念

`RXNE`（RX Not Empty）是 USART 的一個 SR flag：每當 USART 的 shift register 收到一個完整的 frame（1 byte），硬體就把資料搬到 DR，同時拉起 RXNE。

如果開啟 `RXNEIE`（RXNE Interrupt Enable），這個 flag 一被拉起就觸發 ISR。做法就是：

- 每收到 1 byte → 觸發一次 ISR
- ISR 裡從 DR 讀資料，存進 ring buffer
- main loop 從 ring buffer 讀出資料解析

#### 實作內容

```c
// 私有狀態
static volatile uint8_t rx_buf[128];
static volatile uint8_t rx_head = 0;  // ISR 推進
static          uint8_t rx_tail = 0;  // main 推進

// ISR
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
        uint8_t data = (uint8_t)(USART1->DR & 0xFF);
        if (sr & USART_SR_RXNE) {
            uint8_t next = (rx_head + 1) & 0x7F;
            if (next != rx_tail) {
                rx_buf[rx_head] = data;
                rx_head = next;
            }
        }
    }
}
```

Init 時開啟 `RXNEIE`：

```c
USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
NVIC_EnableIRQ(IRQ_USART1);
```

Ring buffer 是 SPSC（Single Producer Single Consumer）設計：
- ISR 是唯一的 producer，只推進 `rx_head`
- main 是唯一的 consumer，只推進 `rx_tail`
- 不需要 critical section，Cortex-M4 的 16-bit 對齊寫入是原子操作

這個方案也同時處理了 ORE（Overrun Error）、FE（Framing Error）、NE（Noise Error）：讀一次 DR 就清除這些 flag，不讓硬體錯誤狀態堆積。

#### 這個方案在 CLI 場景下是可以用的

CLI 的使用情境：使用者在 Tera Term 手動打字，每秒最多幾十個 byte。以 9600 baud 計算，每個 byte 的間距是 1.04ms；115200 baud 也有 87µs。對於 CLI，每個 byte 觸發一次 ISR 完全沒有問題，中斷頻率很低。

---

### 為什麼換掉：Circular DMA 更合適的根本原因

雖然 RXNE per-byte ISR 能用，但從**設計概念**上它有幾個不足：

#### 1. 每個 byte 都有中斷開銷

每次 ISR 觸發，CPU 都要：
- 儲存 caller registers（push context）
- 執行 ISR 本體（讀 DR、計算 index、寫 ring buffer）
- 恢復 context、return

對於 9600 baud，這個開銷完全可以忽略。但如果之後升到 115200 baud 或 1Mbaud（USART1 支援到 1.3125Mbaud@168MHz），每 87µs 就一次 ISR，會在 CPU 上佔掉可觀的比例。

更根本的問題：這個工作（把硬體搬好的 byte 放進一塊記憶體）本來就是 DMA 最擅長的事。

#### 2. RXNE ISR 有 overrun 的隱患

如果有段時間 ISR 不能及時執行（例如被更高優先序的中斷搶佔），DR 還沒被讀，新的 byte 進來就會拉起 ORE（Overrun Error）。DR 裡的舊 byte 此時還在，但 shift register 的新 byte 就永遠丟了。

DMA 就不會有這個問題：DMA 直接把 DR 搬到記憶體，不通過 CPU，只要 AHB bus 沒有長時間被霸佔，就不會 overrun。

#### 3. Circular DMA 是這個場景的標準做法

對「接收不定長資料、需要隨時 ready」這個問題，STM32F4 的標準解法就是：

- DMA circular mode：DMA 永遠在跑，把每個到來的 byte 填進 buffer，buffer 填滿後自動從頭開始
- CPU 不用管每個 byte，只要定期（或在 IDLE 觸發時）去看「DMA 目前寫到哪了」

---

### 方案二：Circular DMA + IDLE Interrupt（現在的實作）

#### 概念

```
USART1_RX（PA10）
   ↓ 每收一個 byte，RXNE 產生 DMA request
DMA2 Stream2 Channel4（USART1_RX）
   ↓ 把 USART1->DR 搬到 rx_dma_buf[head]，head 遞增
   ↓ circular：到 buf[127] 後自動繞回 buf[0]
rx_dma_buf[128]                               ← CPU 可以讀這裡
                                              ← IDLE interrupt 通知「一包資料到了」
```

CPU 完全不參與每個 byte 的搬移，只在 USART IDLE（線路靜默一個 frame 以上）時觸發一次 ISR，設一個旗標。

#### ring buffer head 的計算方式

舊方案的 `rx_head` 由 ISR 維護（軟體計數）。新方案直接從 DMA 的 NDTR 暫存器計算：

```c
static inline uint8_t dma_head(void)
{
    return (uint8_t)((RX_BUF_SIZE - (uint16_t)DMA2_Stream2->NDTR) & RX_BUF_MASK);
}
```

`NDTR` 是 DMA 的 "Number of Data Transfer" 暫存器，circular 模式下持續倒數：`128 → 127 → ... → 1 → 128 → ...`。

- NDTR = 128（剛啟動，沒有 byte 收到）→ `dma_head() = 0`
- NDTR = 127（收了 1 byte，寫進 buf[0]）→ `dma_head() = 1`（下一個要寫 buf[1]）
- NDTR = 1（收了 127 byte）→ `dma_head() = 127`

也就是說，`dma_head()` 永遠指向「DMA 下一個要寫的位置」，同時也等於「目前有幾個 byte 等待被讀（從 tail 算）」。這樣 ring buffer 的 available / read / flush 計算邏輯跟 RXNE 版本完全一樣，只是把 `rx_head` 換成 `dma_head()` 的呼叫。

#### USART1 IDLE interrupt

```c
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & USART_SR_IDLE) {
        (void)USART1->DR;   /* 讀 DR 清除 IDLE flag（必須先讀 SR 再讀 DR）*/
        rx_idle_flag = 1;
    }
}
```

`IDLE` flag 的觸發條件：USART 接收到至少一個 byte 之後，偵測到線路靜默超過一個 frame 的時間。對 CLI 來說，這幾乎等同於「使用者按下 Enter，一行指令送完了」。

`rx_idle_flag` 讓 main loop 可以做：

```c
if (rx_idle_flag) {
    rx_idle_flag = 0;
    char line[64];
    uint16_t len;
    if (usart1_read_line(line, sizeof(line), &len) == USART1_OK) {
        handle_command(line);
    }
}
```

當然也可以不用這個 flag，直接在 main loop 每輪呼叫 `usart1_read_line()`，有 '\n' 就處理，沒有就繼續跑，效果一樣。

#### init 的變化

```c
/* DMA2 Stream2 Channel4：USART1_RX，circular，永遠跑 */
DMA2_Stream2->CR  = 0;
while (DMA2_Stream2->CR & DMA_CR_EN) {}
DMA2->LIFCR = DMA_FLAGS_STREAM2;
DMA2_Stream2->PAR  = (uint32_t)&USART1->DR;
DMA2_Stream2->M0AR = (uint32_t)rx_dma_buf;
DMA2_Stream2->NDTR = RX_BUF_SIZE;
DMA2_Stream2->CR   = DMA_CR_CHSEL(4) | DMA_CR_DIR_P2M
                   | DMA_CR_MINC | DMA_CR_CIRC | DMA_CR_EN;

USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;  /* 加了 DMAR */
USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE
            | USART_CR1_IDLEIE;                  /* RXNEIE → IDLEIE */
```

注意這裡啟動 DMA2_Stream2 時沒有設任何 interrupt enable（沒有 TCIE、HTIE）。DMA 只是靜靜地搬資料，不會觸發 DMA ISR；CPU 的唯一通知來源是 USART1 的 IDLE interrupt。

---

### 兩個方案比較

| 項目 | 方案一：RXNE ISR | 方案二：Circular DMA + IDLE |
|---|---|---|
| 每個 byte 的 CPU 參與 | ISR + register save/restore | 零（DMA 硬體搬） |
| head 維護 | 軟體計數（volatile `rx_head`） | 讀 `NDTR` 暫存器計算 |
| Overrun 風險 | ISR 被搶佔時有 ORE 風險 | 幾乎無（除非 AHB bus 極端堵塞）|
| 「一行資料到了」的通知 | 無（main 需持續 poll） | IDLE interrupt（精確觸發一次）|
| 適合場景 | 低 baud rate、極簡實作 | 通用；特別適合「burst + idle」的協定（CLI、AT 命令、GPS NMEA）|
| 公開 API 是否相容 | — | 完全相同（`rx_available` / `read_byte` / `read_line` / `rx_flush`）|

#### 一個關鍵的理解

RXNE ISR 版本的 ring buffer 和 Circular DMA 版本的 ring buffer，**結構上完全相同**，差別只在「誰負責把 byte 放進 buffer 並推進 head」：

- RXNE ISR：CPU 在 ISR 裡一個個搬
- Circular DMA：DMA 硬體直接寫入，CPU 只是讀 NDTR 反推 head 的位置

這個理解是從 RXNE 版本過渡到 DMA 版本最重要的一步：**ring buffer 的邏輯不變，只是 head 的計算來源從軟體計數器換成了硬體暫存器**。
