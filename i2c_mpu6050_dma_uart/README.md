# 實驗十一：I2C + MPU6050（讀加速度感測器）

### 實驗目標
透過 I2C 與 MPU6050 通訊：
1. 確認感測器身份（讀 WHO_AM_I = 0x68）
2. 喚醒感測器（清 SLEEP bit）
3. 連續讀 6 byte 加速度資料
4. 透過 UART 送到 PC

```
AX=120 AY=-50 AZ=16384      → 平放
AX=80 AY=30 AZ=-16380        → 翻轉過來
```

---

### 硬體接線

| STM32 | MPU6050 (GY-521) | 說明 |
|-------|------------------|------|
| 3.3V  | VCC | 電源（不要接 5V）|
| GND   | GND | 共地（必須）|
| PB6   | SCL | I2C clock |
| PB7   | SDA | I2C data |

I2C1 SDA 替代腳位有 PB7 或 PB9（二選一）。

**AD0 不接**（內部 pull-down，浮空 = 位址 0x68）。

---

### I2C 協定快速複習

兩條線（SCL clock、SDA data），SDA 是 **open-drain**，必須 pull-up。

```
START   SCL=H 時，SDA H→L
位址    7-bit slave address + 1-bit R/W
ACK     接收方拉低 SDA 表示「收到」
Data    8-bit data + ACK
STOP    SCL=H 時，SDA L→H
```

---

### Register-based slave 設計

MPU6050 內部有一百多個 register，每個 1-byte address：

| Address | Register | 功能 |
|---------|----------|------|
| 0x3B~0x40 | ACCEL_XOUT_H ~ ACCEL_ZOUT_L | 加速度 X/Y/Z（每軸 16-bit 分 H/L）|
| 0x43~0x48 | GYRO_XOUT_H ~ GYRO_ZOUT_L | 陀螺儀 X/Y/Z |
| 0x6B | PWR_MGMT_1 | 電源管理（bit6 SLEEP）|
| 0x75 | WHO_AM_I | 固定 = 0x68（裝置識別）|

**讀取流程必須先指定 register address，然後換方向讀資料：**
```
START → addr+W → reg_addr → RESTART → addr+R → data → NACK → STOP
                  ↑                              ↑
             設定 slave 內部 register pointer   slave 從 pointer 所指的內容
```

**RESTART 的意義：** 不發 STOP，bus 繼續被 master 佔用，slave 的 register pointer 不被重置。

---

### STM32 I2C 的操作機制

ST 的 I2C 是「等 flag → 動作 → 等下一個 flag」。看看機制，不需要自己 toggle SDA/SCL。

| Flag | SR | bit | 何時被設 | 怎麼清 |
|------|----|----|---------|--------|
| SB   | SR1 | 0 | START 已送出 | 讀 SR1 + 寫 DR |
| ADDR | SR1 | 1 | 位址確認且 ACK | 讀 SR1 + 讀 SR2 |
| TXE  | SR1 | 7 | DR 空（可放下個 byte）| 寫 DR |
| RXNE | SR1 | 6 | DR 有資料可讀 | 讀 DR |
| BTF  | SR1 | 2 | shift + DR 都滿，bus 自動暫停 | 讀 DR / 寫 DR / STOP |
| AF   | SR1 | 10 | NACK 收到 | 寫 0 |

**ADDR 含有「ACK 確認」：** ADDR = 1 隱含「位址 ACK 已收到」。NACK 的話 ADDR 永遠 0，AF 會 set。

---

### TXE vs BTF

```
DR (data register) ──→ Shift Register ──→ SDA pin
        ↑                    ↑
        TXE = 1              BTF = 1
        DR 空                DR 與 shift 都空
        （byte 在 shift 中）  （bus 完全空閒）
```

**要訣：**
- 中間 byte（要繼續下一個 byte）→ 用 TXE（流水線並行）
- **最後一個 byte（要 STOP / RESTART / 換方向）→ 用 BTF**（bus 必須空閒才能改變狀態）

如果最後用 TXE 立刻 STOP，shift register 還在送 byte 時 STOP 就把它截斷。

---

### i2c_read_regs 的 N=1、N=2、N≥3 三種 case

讀資料的核心問題：**最後一個 byte 必須 NACK，但 ACK 在收到瞬間決定**。

CR1.ACK bit 影響「下一個進 shift register 的 byte」：
- byte 在 shift 時滿的瞬間 → 硬體看當下 CR1.ACK 決定 ACK/NACK

修改 CR1.ACK 的時機必須早於「最後一個 byte 進 shift」。

**N = 1（在清 ADDR 前設 ACK = 0）**
```
ACK = 0 → 清 ADDR → slave 開始送 byte 0 → 收到的 ACK = 0 → NACK →
```

**N = 2（用 POS bit，特殊）**
- 流水線太短，「清 ADDR 後到 byte 0 收到」沒有機會改 ACK
- 用 POS = 1 讓 ACK 設定「延後一個 byte 的效果」
- 設 ACK = 1, POS = 1 → 清 ADDR → 改 ACK = 0 → byte 1 收到 NACK

**N ≥ 3（用 BTF 等流水線滿，race-free）**

```c
// ← 危險寫法（race condition）：
for (int i = 0; i < len - 1; i++) {
    while (!(SR1 & RXNE));
    buf[i] = DR;
}
ACK = 0;                  // ← race！
while (!(SR1 & RXNE));
buf[len-1] = DR;
```

CPU 從讀完 byte N-2 到改 ACK 的時間 T_cpu **不固定**（中斷、cache）。
I2C 硬體收到 byte N-1 的時間 T_byte 也**不固定**（slave clock stretching）。

兩者賽跑：
- T_cpu < T_byte → ACK = 0 提前 → byte N-1 NACK → 正確
- T_cpu > T_byte → byte N-1 已被 ACK → byte N+1 開始送進來 → 錯誤

**這是間歇性 bug，有時會有時不會，難以 debug。**

**BTF 策略（race-free）：**
```
等 BTF = 1 → bus 自動暫停 → CPU 從容改 ACK 不會輸給硬體
讀 DR → bus 繼續
```

### 完整 i2c_read_regs

```c
void i2c_read_regs(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (len == 0) return;

    // ===== 前段：指定 register =====
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & (1 << 0)));

    I2C1->DR = (slave << 1) | 0;
    while (!(I2C1->SR1 & (1 << 1)));
    (void)I2C1->SR1; (void)I2C1->SR2;

    while (!(I2C1->SR1 & (1 << 7)));
    I2C1->DR = reg;
    while (!(I2C1->SR1 & (1 << 2)));          // BTF（要 RESTART）

    // ===== RESTART → 換讀 =====
    I2C1->CR1 |= (1 << 8);                    // RESTART
    while (!(I2C1->SR1 & (1 << 0)));

    I2C1->DR = (slave << 1) | 1;

    if (len == 1) {
        I2C1->CR1 &= ~(1 << 10);              // ACK = 0
        while (!(I2C1->SR1 & (1 << 1)));
        (void)I2C1->SR1; (void)I2C1->SR2;
        I2C1->CR1 |= (1 << 9);                // STOP
        while (!(I2C1->SR1 & (1 << 6)));
        buf[0] = I2C1->DR;

    } else if (len == 2) {
        I2C1->CR1 |= (1 << 10);               // ACK = 1
        I2C1->CR1 |= (1 << 11);               // POS = 1
        while (!(I2C1->SR1 & (1 << 1)));
        (void)I2C1->SR1; (void)I2C1->SR2;
        I2C1->CR1 &= ~(1 << 10);              // ACK = 0（延後效果）
        while (!(I2C1->SR1 & (1 << 2)));      // BTF
        I2C1->CR1 |= (1 << 9);                // STOP
        buf[0] = I2C1->DR;
        buf[1] = I2C1->DR;
        I2C1->CR1 &= ~(1 << 11);              // 清 POS

    } else {
        I2C1->CR1 |= (1 << 10);               // ACK = 1
        while (!(I2C1->SR1 & (1 << 1)));
        (void)I2C1->SR1; (void)I2C1->SR2;

        for (int i = 0; i < len - 3; i++) {
            while (!(I2C1->SR1 & (1 << 6)));
            buf[i] = I2C1->DR;
        }

        while (!(I2C1->SR1 & (1 << 2)));      // BTF（流水線卡滿）
        I2C1->CR1 &= ~(1 << 10);              // ACK = 0
        buf[len - 3] = I2C1->DR;

        while (!(I2C1->SR1 & (1 << 2)));      // BTF
        I2C1->CR1 |= (1 << 9);                // STOP
        buf[len - 2] = I2C1->DR;
        buf[len - 1] = I2C1->DR;
    }
}
```

---

### I2C 設定流程

```c
void i2c_init(void) {
    I2C1->CR1 &= ~(1 << 0);          // PE = 0
    I2C1->CR1 |=  (1 << 15);         // SWRST
    I2C1->CR1 &= ~(1 << 15);

    I2C1->CR2   = 16;                // FREQ = 16MHz
    I2C1->CCR   = 80;                // 100kHz：16M / (2 × 100k) = 80
    I2C1->TRISE = 17;                // T_rise：16 × 1µs + 1 = 17

    I2C1->CR1 |= (1 << 0);           // PE = 1
}
```

GPIO PB6/PB7 必須設 AF4，OTYPER = open-drain，PUPDR = pull-up。

---

### MPU6050 使用流程

```c
// 1. 確認裝置身份
uint8_t id;
i2c_read_regs(0x68, 0x75, &id, 1);   // WHO_AM_I
if (id != 0x68) { /* 錯誤處理 */ }

// 2. 喚醒感測器（清 SLEEP bit）
i2c_write_reg(0x68, 0x6B, 0x00);     // PWR_MGMT_1 = 0

// 3. 讀加速度（連讀 6 bytes）
uint8_t buf[6];
i2c_read_regs(0x68, 0x3B, buf, 6);   // ACCEL_XOUT_H 開始

int16_t ax = (buf[0] << 8) | buf[1];
int16_t ay = (buf[2] << 8) | buf[3];
int16_t az = (buf[4] << 8) | buf[5];
```

MPU6050 register 是 big-endian（H byte 在前），需要手動拼成 int16_t。

±2g range 下：
- +1g → +16384
- 0g → 0
- -1g → -16384

---

### 常踩的坑

1. **沒啟用 i2c_init()** — 最常見 bug，I2C 外設根本沒 enable，所有操作都卡死
2. **沒設 OTYPER = open-drain** — I2C bus 必須 open-drain，push-pull 會兩端打架
3. **沒 pull-up** — GY-521 模組通常板上有 4.7kΩ，沒有的話用 STM32 內建 pull-up（~40kΩ）
4. **多 byte 讀函式直接讀 1 byte** — case 1 跟 case 3 機制完全不同，不能混用
5. **卡在 while 等 ADDR** — slave 沒 ACK 時 ADDR 永遠 0，要加 AF flag 檢查才能逃出
6. **RESTART 前用 TXE 而不用 BTF** — TXE 時 byte 還在 shift register，不 BTF 直接 RESTART 會截斷
7. **最後一個 byte 的 ACK = 0 設太晚** — race condition，間歇性 bug，用 BTF 策略解決
8. **POS 用完沒清** — 2-byte case 結束後 POS 還是 1，下次讀會錯，記得清為 0

---

### 三種 case 對比表

| len | 策略 | 關鍵時機 |
|-----|------|----------|
| 1   | 清 ADDR 前設 ACK=0 | 趕在硬體還沒開始收 byte |
| 2   | POS=1, ACK=1 → 清 ADDR → ACK=0 | POS 讓 ACK 設定延後一格 |
| ≥3  | BTF 等流水線卡滿，bus 暫停後改 ACK | 等流水線滿，bus 暫停才改 ACK |

每個 case 的「能在哪個時間點安全改 ACK」不同，所以策略不同。
