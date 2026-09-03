# STM32F407 — 從暫存器到 RTOS

> STM32F407VGTX（Discovery）· STM32CubeIDE · **全部 register-level，不使用 HAL/LL**
> 一條連續的學習與工程軌跡：**裸暫存器 → bare-metal driver 整合 → FreeRTOS 多工系統**。

這個 repo 不是十三個互不相干的範例，而是**同一套 driver 一路長出來的過程**。
每個專案都是下一個的地基：`blink_register` 學會的 RCC/GPIO 位元操作，最後仍活在
FreeRTOS 版的 `driver_spi1.c` 裡；exp12 踩到的 SPI OVR 坑，決定了 exp13 的匯流排收尾規則；
exp13 的 co-operative 主迴圈之所以要換成 RTOS，是因為它已經被自己的輪詢節奏卡住。

---

## 三個階段

```
 階段一 · Register Level          階段二 · Bare-metal 整合         階段三 · RTOS
 「讓這顆外設動起來」              「讓多顆外設一起活著」            「讓多件事同時發生」

 blink / systick / button          exp12  SPI OLED + I2C IMU        FreeRTOS_sd_imu_oled
 tim_pwm / dma_basic         ──►   exp13  + SD driver + FatFs   ──► 五個事件驅動 task
 UART / dma_uart / adc / i2c              co-operative 主迴圈         Queue + 雙 bus mutex
                                                                     中斷驅動 I2C
 查 RM0090 直接寫暫存器            分層架構、status code、          自建 CPU 統計
 一個專案一顆外設                  timeout、bus recovery            供電邊界降級模式
```

| 階段 | 核心問題 | 學到的東西 |
|---|---|---|
| **一** | 這個位元設下去，硬體到底做了什麼？ | memory map、clock gating、AF 對應、中斷向量、DMA 通道是硬體綁死的 |
| **二** | 兩顆裝置共用一條匯流排會怎樣？ | 分層、錯誤碼、逾時、bus recovery，以及**共用狀態的髒污會咬人** |
| **三** | 誰在等？等多久？這段時間 CPU 在幹嘛？ | 同步原語、優先權、事件驅動、**先量測再優化**、失敗時的降級 |

---

## 專案地圖

### 階段一 — Register Level（一顆外設一個實驗）

| 專案 | 實驗 | 主題 | 關鍵收穫 |
|---|---|---|---|
| [`blink_register`](blink_register) | 一 | RCC + GPIO | 開 clock、MODER 每 pin 佔 2 bit、ODR/BSRR |
| [`systick_register`](systick_register) | 二 | SysTick | Cortex-M 內建計時器、1ms tick、`volatile` 的必要性 |
| [`button_interrupt`](button_interrupt) | 三 / 四 | EXTI + NVIC | 邊沿偵測 → EXTI → NVIC → ISR 這條鏈；軟體去抖 |
| [`tim_pwm`](tim_pwm) | 五 | TIM4 PWM | 時脈鏈 → PSC/ARR/CCR，呼吸燈 |
| [`dma_basic`](dma_basic) | 六 | DMA mem→mem | DMA 要知道的四件事、TC 中斷 |
| [`UART_register`](UART_register) | 七 | USART1 polling | BRR 怎麼從 clock 算、TXE/TC 的差別 |
| [`dma_uart`](dma_uart) | 八 | USART1 TX DMA | **stream/channel 是晶片畫死的**（RM0090 Table 43）|
| [`adc_circular_dma_write_uart`](adc_circular_dma_write_uart) | 九 / 十 | ADC + circular DMA + UART DMA | 完整資料流，CPU 全程不搬資料 |
| [`i2c_mpu6050_dma_uart`](i2c_mpu6050_dma_uart) | 十一 | I2C + MPU6050 | 多 byte 讀的 ACK/NACK 收尾規則各不相同 |

### 階段二 — Bare-metal 整合

| 專案 | 實驗 | 主題 |
|---|---|---|
| [`exp12_ssd1306_spi`](exp12_ssd1306_spi) | 十二 | **第一個整合系統**：SPI DMA OLED + I2C MPU6050 + UART DMA。driver 開始有 `*_status_t`、timeout、`i2c1_bus_recovery()`、async 傳輸 |
| [`exp13_sd_imu_oled`](exp13_sd_imu_oled) | 十三 | **IMU 資料記錄器**：自己刻 SD 卡 SPI driver（CMD0/8/17/24/55/58、ACMD41）+ 掛 FatFs（FAT32）+ `app_datalogger` 狀態機 + UART CLI。co-operative 主迴圈 |
| [`test_uart`](test_uart) | — | 除錯用的最小專案：把 USART1 單獨拉出來驗 Discovery 的替代腳位（見下方坑 §3）|

### 階段三 — RTOS

| 專案 | 主題 |
|---|---|
| [`FreeRTOS_sd_imu_oled`](FreeRTOS_sd_imu_oled) | **exp13 的 FreeRTOS 移植**（Kernel V11.1.0 LTS）。五個事件驅動 task、Queue + 雙 bus mutex、中斷驅動 I2C、自建 CPU 佔用統計、INA219 功耗量測、供電邊界下的降級模式。SYSCLK 拉到 168MHz（HSE→PLL，HSERDY 逾時自動 fallback HSI）。**[→ 完整說明](FreeRTOS_sd_imu_oled/README.md)** |

---

## 貫穿全程的幾件事

**一、`stm32f407xx.h` 是自己寫的。**
不是 ST 的那份。從散落的 `#define GPIOD_MODER (*(volatile uint32_t*)0x40020C00)` 慢慢收斂成
struct mapping，一路沿用到 RTOS 版。看得懂官方 header 為什麼長那樣，是因為自己走過一次。

**二、driver 有錯誤碼、有逾時，沒有 `while(1)`。**
從 exp12 開始，每支會等硬體的函式都帶 `systick_get_ms()` 逾時與狀態回傳。
`i2c1_bus_recovery()` 送 9 個 clock pulse 把卡住 SDA 的 slave 逼放手 —— sensor 掉線時
系統該重試或進 error state，不是整機死等。

**三、分層是為了能換掉其中一層。**

```
 Application   app_datalogger · app_cli · app_tasks
 Middleware    FatFs (FAT32) ── diskio.c
 Device        device_mpu6050 · device_ssd1306 · device_sdcard · device_ina219
 Bus / Driver  driver_i2c1 · driver_spi1 · driver_usart1 · driver_rcc · systick
 Porting       bus_port.h · isr_port.h        ← RTOS 中立，link-time 綁定
 MCU           stm32f407xx.h（自寫 register map）
```

`bus_port.h` / `isr_port.h` 這層是 RTOS 化時長出來的：**只宣告具名函式、不 include 任何
RTOS 標頭**，FreeRTOS 版實作成 mutex/semaphore，bare-metal 版寫成 no-op。所以同一份
`device_sdcard.c` 在 exp13 和 RTOS 版都能編，driver 層完全不知道底下有沒有 kernel。

**四、先量測，再優化。**
RTOS 版自己寫了一套比 kernel 內建更準的 CPU 佔用統計（kernel 那份把 ISR 時間算到被打斷的
task 頭上，而且漏掉當下未結算的切片）。用它量到「16.2% 的 CPU 花在空轉等 I2C」，才決定把
I2C 改成中斷驅動 —— 降到 0.8%，而且省下來的每一點都在 IDLE 那欄對得起來。
沒有那個數字，改哪裡都只是猜。

**五、失敗要看得見。**
最難查的 bug 不是當機，是**安靜地騙人**：`status` 說 LOGGING、OLED 有畫面、CPU 分布正常，
但一筆都沒寫進卡片（FatFs 第一次失敗就把錯誤鎖進 `fp->err`，之後每次 `f_write` 直接返回）。
現在每個「可能默默失敗」的地方都有計數器，並且印在 `status` 上。

---

## 五個值得講的坑

| # | 現象 | 根因 |
|---|---|---|
| **1** | SD 卡 CMD0/CMD8 都過，ACMD41 卻永遠停在 idle；UART 同時糊字、MCU 間歇重開機 | **5V 灌進 3.3V 腳**。低電流命令跑得動，ACMD41 要驅動內部 flash 就撐不住。三個現象同時出現，只有「電壓被拉垮」能一起解釋。教訓：CMD8 只是 echo 你宣告的電壓範圍，**不代表實際 VDD 正確**。 |
| **2** | 只跑 SD 一切正常，**一把 OLED 加進主迴圈**，CSV 就截斷、下一個檔整個空白 | OLED 用 **TX-only DMA** 推 framebuffer，全程不讀 RX → 傳完後 SPI 的 **OVR** 被設起、DR 卡著舊 byte。接著 SD 的全雙工 `spi1_transfer_byte()` 看到 RXNE 早已 set，**直接讀走舊 byte、根本沒把命令時脈出去**。修法：所有 DMA 收尾都排空 RX 並清 OVR（先讀 DR 再讀 SR）。→ **共用匯流排最隱晦的坑：一個用 DMA-TX-only、一個用全雙工 polling，前者的髒狀態會咬到後者。** |
| **3** | USART1 接 PA9/PA10 就是亂碼 | Discovery 板上 **PA9/PA10 被板載 USB-OTG（CN5 VBUS/ID）佔用**。改用 PB6/PB7，並把 I2C1 SCL 從 PB6 讓到 PB8。`test_uart` 就是為了隔離驗證這件事而建的最小專案。**注意：exp12/exp13 的 README 仍寫 PA9/PA10 —— 那是發現這件事之前寫的，只有 FreeRTOS 版改過腳位。** |
| **4** | I2C 非同步交易「還沒開始就結束」，而且會自我延續 | SR1 的錯誤位元是**準位觸發**的。原本「先開中斷、再發 START」，只要殘留 `AF`/`BERR`，`ITERREN` 一開 ER 中斷就當場觸發；沒人收拾的 START 留在匯流排上，讓下一筆更容易出錯。改成發起前先清旗標、START 移到開中斷之前。 |
| **5** | SD 寫入逾時、INA219 偶爾讀不到、`f_mount` 偶爾 `FR_NOT_READY` —— 三個間歇故障同時出現 | 三顆周邊、兩條匯流排、全部間歇 → **共同上游是電**（ST-LINK USB 供電，SD 寫入湧浪 100~200mA）。這是硬體問題，但軟體的責任是**讓它可診斷、可存活**：I2C 失敗依 nack/bus/timeout 分類計數、掛載重試並顯示重試過、初始化失敗改成降級而非 suspend（不然 SD 掛不起來連 CLI 都沒有，正好在最需要診斷工具的時候）。 |

---

## 硬體與環境

| 項目 | 內容 |
|---|---|
| MCU | STM32F407VGTX（STM32F4 Discovery），Cortex-M4F |
| 時脈 | 階段一/二：HSI 16MHz。階段三：HSE 8MHz → PLL **168MHz**，HSERDY 逾時 fallback HSI |
| 感測 | MPU6050（I2C，GY-521）· INA219（I2C，功耗）· 晶片內建溫度感測（ADC）|
| 顯示 | SSD1306 128×64 OLED（SPI，DMA 推 framebuffer）|
| 儲存 | microSD（SPI 模式，自寫 driver）+ FatFs R0.15a |
| 通訊 | USART1 → CP2102/CH340 → PC，115200 8N1 |
| 工具 | STM32CubeIDE（managed build）· ST-LINK · RM0090 / PM0214 / 各晶片 datasheet |

> ⚠️ SD 卡務必吃 **3.3V**，VDD–GND 建議補 `10µF + 100nF` 去耦（見坑 §1、§5）。

---

## 怎麼讀這個 repo

- **想看完成度最高的東西** → [`FreeRTOS_sd_imu_oled`](FreeRTOS_sd_imu_oled/README.md)（RTOS 設計、量測數據、取捨與已知限制都寫在它自己的 README）
- **想看 bare-metal 怎麼整合** → [`exp13_sd_imu_oled`](exp13_sd_imu_oled/README.md)（SD driver 從 CMD0 刻起、FatFs 整合、開發歷程與踩坑）
- **想看某顆外設怎麼從暫存器設起** → 階段一的任一專案，每個都有自己的 README，附暫存器表與時序說明

每個資料夾都是獨立的 STM32CubeIDE 專案：Import → Existing Projects into Workspace → Build → 燒錄。
FreeRTOS 版的 kernel 原始碼直接 vendored 在 `Src/` / `Inc/`，managed build 自動收編，**零 `.cproject` 改動**。

---

## 目前狀態與下一步

**已完成**：階段一全部、階段二全部、階段三的 Phase 1~6（排程器 bring-up → 拆 task →
事件驅動 → 功耗與匯流排擴充 → 量測與優化 → 收乾忙等與降級模式）。

**進行中 / 下一步**：

- **上板驗證 Phase 6**：SPI 睡眠等待、SD DMA 寫入、`spi1_poll_yield()` 的預期數字要用 `tasks` 對帳
- **供電根因**：SD 獨立供電 + 就近去耦 —— 降級模式只是讓它可診斷，不是修好
- **SD 讀取路徑**：寫入已走 DMA，讀取仍是逐 byte polling（要收 MISO，得開 RX DMA 或全雙工雙 stream）；多 block CMD18/25 未做
- **HardFault handler**：印出 stacked PC/LR/xPSR，目前 fault 發生時沒有任何線索
- **MPU6050 深化**：DLPF、量程設定、raw → g / °/s 的定點換算與傾斜角
- **Timer / ADC 資料流**：input capture、encoder mode、timer 觸發 ADC 固定 sample rate、DMA double buffer
- **小型 bootloader**：UART 收 firmware image、CRC、寫 flash、跳轉 application（linker script / vector table 的實戰）

---

## 面試準備清單（自問自答用）

- `volatile` 解決什麼問題？不能解決什麼問題？
- ISR 裡為什麼不適合做太多事？「ISR 只做通知」在這個 repo 的哪些地方看得到？
- DMA transfer complete 代表資料真的送完了嗎？SPI 的 `BSY` 為什麼還要等？
- UART baud rate 怎麼從 clock 算出 BRR？
- I2C single-byte / two-byte / multi-byte read 為什麼收尾流程不同？`POS` 是做什麼的？
- circular DMA buffer 什麼時候可以讀？什麼時候不能改？
- SD card SPI 初始化為什麼要在 CS 拉低前送 74 個 clock？
- 兩個裝置共用一條 SPI，一個用 DMA、一個用 polling，會出什麼事？（坑 §2）
- 一段等待要忙等還是睡覺，你怎麼決定？（等待長度 ÷ 切換成本）
- mutex 的優先權繼承解決什麼？什麼情況下它救不了你？
- 為什麼要自己寫 CPU 佔用統計，而不是用 `configGENERATE_RUN_TIME_STATS`？
- 承載 `...FromISR()` 的中斷，優先權為什麼必須 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`？
- linker script 裡 FLASH/RAM 區段在做什麼？startup assembly 如何跳到 `main()`？
- HardFault 發生時你會怎麼查？
- sensor 掉線，firmware 應該卡住、重試、進 error state，還是 reset peripheral？
