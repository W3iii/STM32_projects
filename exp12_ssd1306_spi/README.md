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
