# STM32 軟韌體工程師進度 Roadmap

更新日期：2026-06-08  
平台：STM32F407VGTX、STM32CubeIDE、bare-metal C、register-level driver  
目標：成為可獨立 bring-up、寫 driver、除錯、整合系統的軟韌體工程師

---

## 一句話定位

已進入 **bare-metal driver integration 中階**：能查 register、設定所有主要外設、用 DMA 做 non-blocking 傳輸、寫帶 timeout 與 status code 的 driver API，並已完成跨外設整合（MPU6050 + SSD1306 + UART）。下一步的重心是**塊裝置 driver（SD card via SPI）**與**系統架構能力（UART CLI、cooperative scheduler）**。

---

## 目前專案地圖

| 專案 | 主題 | 狀態 |
|---|---|---|
| `blink_register` | RCC/GPIO register-level LED blink | 完成 |
| `systick_register` | SysTick 1ms tick 與 delay | 完成 |
| `button_interrupt` | EXTI0 + SysTick debounce | 完成 |
| `tim_pwm` | TIM4 PWM 呼吸燈 | 完成 |
| `dma_basic` | DMA memory-to-memory + TC interrupt | 完成 |
| `UART_register` | USART1 polling TX | 完成 |
| `dma_uart` | USART1 TX DMA（non-blocking 概念）| 完成 |
| `adc_circular_dma_write_uart` | ADC internal temp + circular DMA + UART DMA TX | 完成 |
| `i2c_mpu6050_dma_uart` | I2C polling 讀 MPU6050（帶 timeout 前版本）| 完成 |
| `exp12_ssd1306_spi` | **整合最成熟**：SPI DMA OLED + I2C MPU6050 + UART DMA；完整 driver 架構含 timeout / status code / bus recovery | **主線專案** |

---

## 已具備的能力

**外設操作：**
- 能直接操作 STM32F407 memory map、RCC、GPIO、USART、DMA、TIM、ADC、I2C、SPI、SysTick、NVIC
- 能從裸 register 寫出可運作的 peripheral 範例，不依賴 HAL

**Driver 設計（exp12 已達到的品質）：**
- `i2c1_write_reg()` / `i2c1_read_regs()`：有 timeout（`systick_get_ms()`）、NACK 偵測、回傳 `i2c_status_t`
- `i2c1_bus_recovery()`：送 9 clock pulse 強制 slave 釋放 SDA，保護系統不卡死
- `i2c1_scan()`：掃描 I2C bus，診斷工具
- `usart1_send_dma()` / `usart1_send_busy()`：非阻塞 DMA TX，有 busy flag
- `usart1_send_byte()` / `usart1_recv_byte()`：polling API 有 timeout
- `spi1_send_buffer_async()` / `spi1_wait_done()`：SPI DMA 非同步傳輸
- `ssd1306_update_screen_async()` + `ssd1306_wait_update_done()`：framebuffer async push
- `mpu6050_init()`：檢查 WHO_AM_I、喚醒、回傳錯誤碼（-1 = NACK，-2 = bus stuck）

**系統設計：**
- 理解 interrupt handler、`volatile`、DMA transfer complete、polling vs async 的差異
- 已建立 `stm32f407xx.h`，struct mapping 取代散落的裸 address macro
- `systick_get_ms()` 跨 driver 共用做 timeout 基準
- I2C 錯誤在 OLED 上顯示，不是 `while(1)` 卡死

---

## 目前缺口（誠實清單）

| 項目 | 說明 |
|---|---|
| UART RX | 目前只有 polling recv_byte，沒有 ring buffer 或中斷式 RX |
| UART CLI | 無法從 PC 端下指令控制板子 |
| SD card | 尚未實作，是下一個主要目標 |
| MPU6050 scaling | raw value 沒有轉成 g 與 deg/s；無 DLPF / 量程設定 |
| DMA error handling | TEIF/DMEIF/FEIF 只記錄，沒有 recovery 流程 |
| Cooperative scheduler | 還沒有多 task 架構，目前是單一 while loop |
| HardFault handler | 發生 fault 時無 stack dump，難以 debug |
| RTOS | 尚未進入 FreeRTOS |

---

## 下一個目標：SD Card SPI Driver

### 為什麼選 SD card

- 面試高頻外設：SD card 是塊裝置，涵蓋 SPI 時序、命令協定、塊讀寫、錯誤處理
- 技術深度：比單純的 OLED / sensor 更接近真實產品中的儲存子系統
- 已有 SPI DMA driver 底座（`spi1_send_buffer_async()`），可以直接複用

### SD card SPI 模式基礎

```
初始化序列：
  1. CS HIGH + 送 ≥74 個 clock pulse（讓 SD 卡進入 SPI 模式）
  2. CS LOW + CMD0（GO_IDLE_STATE）→ R1 = 0x01（Idle 狀態）
  3. CMD8（SEND_IF_COND）→ 確認電壓範圍（SDHC/SDXC 支援）
  4. ACMD41（SD_SEND_OP_COND）→ 輪詢到 R1 = 0x00（Ready）
  5. CMD58（READ_OCR）→ 讀 OCR，確認 SDHC bit

讀取單一塊（512 bytes）：
  CMD17（READ_SINGLE_BLOCK）→ 等 Data Token 0xFE → 讀 512 bytes + 2 bytes CRC

寫入單一塊：
  CMD24（WRITE_BLOCK）→ 送 Data Token 0xFE → 送 512 bytes + 2 bytes CRC
  → 等 busy（MISO = 0）結束
```

### 計劃 API

```c
typedef enum {
    SD_OK = 0,
    SD_ERR_TIMEOUT,
    SD_ERR_INIT,
    SD_ERR_CMD,
    SD_ERR_CRC,
} sd_status_t;

sd_status_t sd_init(void);
sd_status_t sd_read_block (uint32_t block_addr, uint8_t *buf);
sd_status_t sd_write_block(uint32_t block_addr, const uint8_t *buf);
```

### 建議順序

1. 先用 polling SPI 完成 init + CMD0 + CMD8 + ACMD41
2. 用 polling 讀寫單塊（CMD17 / CMD24），UART 驗證資料
3. 再把讀寫換成 DMA（`spi1_send_buffer_async()`）
4. 加入錯誤處理和 retry 邏輯

---

## Phase 2：通訊能力補齊（進行中）

### SD Card（首要）

- [ ] polling SPI 完成 SD 初始化（CMD0/CMD8/ACMD41）
- [ ] 讀寫單塊 512 bytes（CMD17/CMD24）
- [ ] 換成 DMA 非同步讀寫
- [ ] 整合進 `exp12_ssd1306_spi`：SD log + OLED 顯示

### UART RX / CLI（次要）

- [ ] UART RX interrupt + ring buffer
- [ ] USART IDLE line + DMA circular RX（封包接收）
- [ ] line-based command parser：
  - `mpu read` → 印出 acc/gyro raw
  - `oled clear` → 清 OLED
  - `sd read <block>` → 印出 512 bytes hex dump
  - `rate <n>` → 設定顯示更新頻率
- [ ] UART TX DMA queue：多筆字串連續送不互相覆蓋

### I2C / MPU6050 深化

- [ ] MPU6050 raw → g（÷16384）、deg/s（÷131）
- [ ] 設定 DLPF（Digital Low Pass Filter）
- [ ] 設定量程（±2g / ±4g / ±8g / ±16g）

---

## Phase 3：Timer、ADC、DMA 深化

目標：從「會設定外設」進到「會設計資料流」。

- TIM input capture：量測外部 PWM 頻率與 duty
- TIM encoder mode：接旋轉編碼器
- ADC external channel + circular DMA
- ADC + timer trigger：固定 sample rate，不靠 while loop
- DMA double buffer：一邊收資料，一邊處理上一包
- DMA error flag（TEIF/DMEIF/FEIF）recovery 流程

完成標準：
- 能畫出資料流：Timer trigger → ADC → DMA buffer → processing → UART/OLED
- 能解釋 DMA buffer 的 ownership rule，何時可以讀、何時不能改

---

## Phase 4：系統設計與 RTOS

先做 non-RTOS：

- cooperative scheduler（1ms tick 下跑多個 task）
- event queue（button/sensor/uart command 都丟事件）
- state machine（BOOT、INIT_SENSOR、RUN、ERROR、RECOVERY）
- HardFault handler：印出 stacked PC/LR/xPSR
- watchdog

再做 FreeRTOS：

- task、queue、semaphore、mutex
- UART RX task + sensor task + display task + SD log task
- DMA ISR 裡只做通知，不做重工作
- priority inversion 與 stack size 觀念

完成標準：
- sensor、display、UART CLI、SD logging 同時跑，彼此不互卡
- 出錯時能進 error state 或重啟 peripheral，不是整機死等

---

## Phase 5：作品集專題

以 `exp12_ssd1306_spi` 為核心，發展成兩個代表作。

### 作品集 1：MPU6050 OLED Motion Dashboard + SD Logger

功能：
- MPU6050 讀取 acc/gyro，轉換成 g / deg/s
- OLED 即時顯示 X/Y/Z、姿態估測（傾斜角）
- UART CLI 控制顯示模式、量程、更新率
- SD card 記錄 IMU data（CSV 格式，可用 Python/Excel 畫圖）
- 系統 state machine：BOOT → INIT_SENSOR → RUN → ERROR

展示重點：
- 完整的 register-level driver 堆疊
- 跨外設 DMA 非同步整合
- 塊裝置 driver（SD card）
- 可操作的 CLI + 可分析的 log 檔

### 作品集 2：ADC DMA Data Logger

功能：
- Timer 觸發 ADC，固定 sample rate
- ADC circular DMA double buffer
- UART DMA 輸出 CSV 或 binary packet
- SD card 儲存長時間 log

展示重點：
- real-time sampling pipeline
- buffer ownership
- throughput 與 latency 分析

### 作品集 3：小型 Bootloader（進階）

- UART 接收 firmware image
- 寫入 flash，checksum/CRC
- 跳轉 application
- 展示 linker script、vector table、flash erase/program

---

## 必備知識矩陣

| 領域 | 目前狀態 | 下一步 |
|---|---|---|
| C 語言 | 已能寫裸機 C、指標、volatile、struct register map、status code pattern | 補 ring buffer、state machine、const correctness |
| MCU 架構 | 已做 SysTick/NVIC/interrupt/DMA | 補 exception model、HardFault stack dump、startup flow |
| 外設 driver | GPIO/UART/DMA/TIM/ADC/I2C/SPI 皆有，I2C/UART 已有 timeout + status | 補 SD card（塊裝置）、UART RX ring buffer |
| Toolchain | 使用 CubeIDE 與 linker script | 補 map file、objdump、size、startup assembly |
| Debug | 目前偏 LED/UART/OLED debug | 補 breakpoint/watchpoint、logic analyzer、HardFault dump |
| RTOS | 尚未進入 | 先寫 cooperative scheduler，再進 FreeRTOS |
| 工程化 | Driver API 有 status code；每個專案有 README | 補 UART CLI、state machine、demo 影片 |

---

## 面試準備清單

- `volatile` 解決什麼問題？不能解決什麼問題？
- ISR 裡為什麼不適合做太多事？
- DMA transfer complete 代表資料真的送完了嗎？SPI 的 BSY 為什麼還要等？
- UART baud rate 怎麼從 clock 算出 BRR？
- I2C single-byte、two-byte、multi-byte read 為什麼 ACK/NACK 流程不同？
- circular DMA buffer 什麼時候可以讀？什麼時候不能改？
- SD card SPI 初始化為什麼要送 74 個 clock before CS LOW？
- linker script 裡 FLASH/RAM 區段在做什麼？
- startup assembly 如何跳到 `main()`？
- HardFault 發生時你會怎麼查？
- sensor 掉線，firmware 應該卡住、重試、進 error state，還是 reset peripheral？

---

## 建議下一個工作項目

1. **SD card driver（polling 版）**：接好 SPI，完成 CMD0 → CMD8 → ACMD41 初始化，UART 印出成功訊息
2. **SD 單塊讀寫**：CMD17 讀 + CMD24 寫，寫完再讀回來驗證資料
3. **SD DMA 版**：把讀寫換成 `spi1_send_buffer_async()`
4. **UART RX + ring buffer**：讓 `exp12` 可以收 PC 的指令
5. **MPU6050 scaling**：raw → g / deg/s，加簡單的傾斜角計算
