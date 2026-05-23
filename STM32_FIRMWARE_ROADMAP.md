# STM32 軟韌體工程師進度 Roadmap

掃描日期：2026-05-19  
目前平台：STM32F407VGTX、STM32CubeIDE、bare-metal C、register-level driver  
目標：成為可獨立 bring-up、寫 driver、除錯、整合系統的軟韌體工程師

## 一句話定位

你目前已經不是單純「會點 LED」的階段，而是進入 **bare-metal driver integration 初中階**：你會查 register、設定外設、用中斷與 DMA，並且已經開始把 SPI/I2C/OLED/MPU6050 做成可重用 driver。下一步重點不是多做幾個範例，而是把現有能力打磨成「穩定、可除錯、可維護、可展示」。

## 目前專案地圖

| 專案 | 主題 | 目前狀態 | 下一步 |
|---|---|---|---|
| `blink_register` | RCC/GPIO register-level LED | 已完成基礎 GPIO 輸出 | 修正註解與 pin 一致性，練 BSRR atomic set/reset |
| `systick_register` | SysTick 1ms tick 與 delay | 已完成中斷式 delay | 加入 tick overflow 說明、封裝成共用 module |
| `button_interrupt` | EXTI0 + SysTick debounce | 已完成按鍵中斷與去彈跳 | 加 NVIC priority、pull-up/down 設定、事件 flag 設計 |
| `UART_register` | USART1 polling TX | 已完成 UART TX | 統一 baud rate 設定，加入 RX interrupt |
| `dma_basic` | DMA memory-to-memory | 已完成 DMA transfer complete interrupt | 補 DMA error flag、half transfer、邊界測試 |
| `dma_uart` | USART1 TX DMA | 已完成非阻塞 TX 概念 | 加 UART RX DMA circular + IDLE line |
| `tim_pwm` | TIM4 PWM 呼吸燈 | 已完成 4 channel PWM | 練 input capture、encoder mode、servo PWM |
| `adc_circular_dma_write_uart` | ADC internal temperature + circular DMA + UART DMA | 已能整合 ADC/DMA/UART | 加 half/full transfer callback、校準公式、外部 ADC channel |
| `i2c_mpu6050_dma_uart` | I2C polling 讀 MPU6050 + UART DMA 輸出 | 已完成 sensor bring-up | 加 timeout、bus recovery、gyro/acc scaling |
| `exp12_ssd1306_spi` | SPI DMA OLED + I2C MPU6050 + framebuffer | 目前最成熟的整合專案 | 修正資料初始化、加簡單 scheduler、整理成作品集 |

## 已具備的能力

- 能直接操作 STM32F407 memory map、RCC、GPIO、USART、DMA、TIM、ADC、I2C、SPI、SysTick、NVIC。
- 能從裸 register 寫出可運作的 peripheral 範例，而不是只依賴 HAL。
- 已經理解 interrupt handler、`volatile`、DMA transfer complete、polling vs async 的差異。
- 已建立自製 `stm32f407xx.h`，用 struct mapping 取代散落的裸 address macro。
- 已開始寫 driver API，例如 `spi1_send_buffer_async()`、`ssd1306_update_screen_async()`、`mpu6050_read_accel()`。
- 已能做跨外設整合：MPU6050 via I2C、SSD1306 via SPI DMA、UART DMA debug output。

## 立即修正清單

這些不是否定你的進度，反而是現在最適合修的工程品質點。

| 優先級 | 項目 | 建議 |
|---|---|---|
| P0 | `exp12_ssd1306_spi/Src/main.c` 裡 `acc` 在第一次畫面前尚未讀取 | 進入 while 前先 `mpu6050_read_accel(&acc);` 或初始化為 0 |
| P0 | I2C driver 幾乎所有 `while (!(flag))` 都沒有 timeout | 讓 `i2c1_write_reg()` / `i2c1_read_regs()` 回傳 status，避免 sensor 掉線時卡死 |
| P1 | `UART_register` 的 `BRR = 0x3415` 和 9600 baud 註解不一致 | 明確決定要 1200 或 9600；16 MHz / 9600 常見 BRR 約 `0x0683` |
| P1 | 多個專案重複放 `stm32f407xx.h` | 之後建立共用 driver/include 專案，避免 header 分裂 |
| P1 | DMA ISR 目前多半只處理 transfer complete | 加入 TEIF/DMEIF/FEIF error handling，至少記錄錯誤 flag |
| P2 | GPIO alternate function 註解偶爾和實際 AF 不一致 | 對照 datasheet 表格修註解，訓練 datasheet discipline |

## Roadmap Phase 1：把基礎變穩，2 週

目標：讓現有裸機能力變成可靠底座。

- 整理一份共用 `stm32f407xx.h`，不要在每個 project 各放一份。
- 建立共用 modules：`systick.c/h`、`gpio.c/h`、`uart.c/h`、`dma.c/h`。
- 寫一份 clock note：目前大多假設 16 MHz HSI，之後要能說清楚 HCLK、PCLK1、PCLK2、USART BRR、I2C CCR、timer clock。
- 所有 driver 的 blocking wait 都加 timeout。
- 所有 peripheral init 都用明確命名 macro，少用裸 `(1 << n)`。
- 每個實驗專案補一個短 README：目標、接線、用到的 register、驗證方法。

完成標準：

- 拔掉 MPU6050 或 OLED 時，程式不會永遠卡死，會顯示錯誤或回傳 fail。
- 你能口頭解釋 GPIO AF、USART baud、I2C ACK/NACK、DMA stream/channel 的設定流程。

## Roadmap Phase 2：通訊能力補齊，3 到 5 週

目標：把 firmware 工程師常被要求的 UART/I2C/SPI 能力補成完整形狀。

### UART

- 實作 RX interrupt。
- 實作 ring buffer。
- 實作 line-based command parser，例如：
  - `help`
  - `mpu read`
  - `oled clear`
  - `rate 50`
- 實作 UART TX DMA queue，避免多個字串連續送時互相覆蓋。
- 練 USART IDLE line + DMA circular RX。

### I2C

- `i2c1_read_regs()` 改成回傳 error code。
- 加 timeout、NACK handling、bus busy recovery。
- 寫 I2C scanner。
- 將 MPU6050 raw data 轉成 g 與 deg/s。
- 加 DLPF、量程設定、取樣率設定 API。

### SPI

- 補 SPI DMA error state。
- OLED framebuffer 加 double buffering 或 dirty rectangle。
- 練一個 SPI flash 或 SD card module，理解 full-duplex、chip select timing、block device 概念。

完成標準：

- PC 端可以透過 UART CLI 控制板子。
- OLED 顯示資料時，UART 仍可輸入指令。
- Sensor 掉線時系統能報錯，不會無聲卡住。

## Roadmap Phase 3：Timer、ADC、DMA 深化，1 到 2 個月

目標：從「會設定外設」進到「會設計資料流」。

- TIM input capture：量測外部 PWM 頻率與 duty。
- TIM encoder mode：接旋轉編碼器。
- TIM one-pulse mode：產生精準觸發。
- ADC external channel + circular DMA。
- ADC half-transfer / transfer-complete callback。
- ADC + timer trigger：固定 sample rate，不靠 while loop。
- DMA double buffer：一邊收資料，一邊處理上一包。
- UART DMA log system：避免 debug print 卡住主流程。

完成標準：

- 你能畫出資料流：Timer trigger -> ADC -> DMA buffer -> processing -> UART/OLED。
- 你能解釋為什麼 DMA buffer 需要 ownership rule，何時可以改、何時不能改。

## Roadmap Phase 4：系統設計與 RTOS，2 到 3 個月

目標：從 driver 作者變成能做完整 embedded application 的人。

先做 non-RTOS：

- cooperative scheduler，例如 1 ms tick 下跑多個 task。
- event queue，例如 button/sensor/uart command 都丟事件。
- state machine，例如 boot、sensor init、run、error、recovery。
- watchdog。
- fault handler：HardFault 時印出 stacked PC/LR/xPSR。

再做 FreeRTOS：

- task、queue、semaphore、mutex。
- UART RX task + sensor task + display task。
- DMA ISR 裡只做通知，不做重工作。
- priority inversion 與 stack size 觀念。

完成標準：

- 同一個專案裡可以同時跑 sensor、display、UART CLI、logging，且彼此不互卡。
- 出錯時能進 error state 或重啟 peripheral，而不是整機死等。

## Roadmap Phase 5：作品集專題，1 到 2 個月

建議把 `exp12_ssd1306_spi` 發展成你的第一個代表作。

### 作品集 1：MPU6050 OLED Motion Dashboard

功能：

- MPU6050 讀取 acc/gyro。
- OLED 即時顯示 X/Y/Z、角速度、簡單姿態估測。
- UART CLI 可切換顯示模式、更新率、量程。
- SPI OLED 使用 DMA async update。
- I2C 有 timeout 與錯誤狀態。
- 系統有 state machine：BOOT、INIT_SENSOR、RUN、ERROR。

展示重點：

- 自己寫 register-level drivers。
- DMA 非同步更新畫面。
- I2C sensor driver error handling。
- 清楚的 README、接線圖、架構圖、demo 影片。

### 作品集 2：ADC DMA Data Logger

功能：

- Timer 固定頻率觸發 ADC。
- ADC circular DMA double buffer。
- UART DMA 輸出 CSV 或 binary packet。
- PC 端可畫圖或儲存 log。

展示重點：

- real-time sampling。
- buffer ownership。
- throughput 與 latency 分析。

### 作品集 3：小型 Bootloader

進階題，先不急，但很有面試價值。

功能：

- UART 接收 firmware image。
- 寫入 flash。
- checksum/CRC。
- 跳轉 application。

展示重點：

- linker script。
- vector table。
- flash erase/program。
- reset 與 boot flow。

## 必備知識矩陣

| 領域 | 你目前狀態 | 下一步 |
|---|---|---|
| C 語言 | 已能寫裸機 C、指標、volatile、struct register map | 補 const correctness、static linkage、bit macro、ring buffer、state machine |
| MCU 架構 | 已碰 SysTick/NVIC/interrupt | 補 exception model、priority、stack、HardFault debug |
| 外設 driver | 已做 GPIO/UART/DMA/TIM/ADC/I2C/SPI | 補 timeout、error handling、callback、non-blocking API |
| Toolchain | 使用 CubeIDE 與 linker script | 補 map file、objdump、size、startup flow |
| Debug | 目前偏 LED/UART debug | 補 breakpoint/watchpoint、logic analyzer、fault dump |
| RTOS | 尚未看到 | 先寫 cooperative scheduler，再進 FreeRTOS |
| 工程化 | 已開始 driver 化 | 補 README、coding style、module boundary、版本控制 |

## 每週練習節奏

- 2 天：實作一個小 driver 或修一個現有 driver。
- 1 天：只讀 Reference Manual 和 datasheet，補 register 筆記。
- 1 天：故意製造錯誤並練 debug，例如拔線、錯 baud、DMA flag 不清。
- 1 天：整理 README、畫架構圖、寫「我踩到什麼坑」。
- 週末：把一個功能整合進代表作，而不是另開一個孤立 demo。

## 面試準備清單

你要能清楚回答：

- `volatile` 解決什麼問題？不能解決什麼問題？
- ISR 裡為什麼不適合做太多事？
- DMA transfer complete 代表資料真的送完了嗎？SPI 的 BSY 為什麼還要等？
- UART baud rate 怎麼從 clock 算出 BRR？
- I2C single-byte、two-byte、multi-byte read 為什麼 ACK/NACK 流程不同？
- circular DMA buffer 什麼時候可以讀？什麼時候不能改？
- linker script 裡 FLASH/RAM 區段在做什麼？
- startup assembly 如何跳到 `main()`？
- HardFault 發生時你會怎麼查？
- 如果 sensor 掉線，firmware 應該卡住、重試、進 error state，還是 reset peripheral？

## 建議下一個 commit/工作項目

如果要從現在的專案繼續，建議照這個順序做：

1. 修 `exp12_ssd1306_spi/Src/main.c` 的 `acc` 初始化問題。
2. 把 `i2c_driver.c` 的三個 API 改成有 timeout 與回傳 status。
3. 在 OLED 上顯示 I2C error，而不是 `while (1)`。
4. 加 UART CLI 到 `exp12_ssd1306_spi`，能查 `WHO_AM_I`、切換 acc/gyro 顯示。
5. 整理 `exp12_ssd1306_spi/README.md`，把它變成作品集專案。

## 目前等級判斷

以成為 junior firmware engineer 的路線來看，你已完成：

- register-level GPIO / clock / interrupt 基礎
- SysTick timing
- UART TX
- DMA 基礎與 peripheral DMA
- PWM
- ADC circular DMA
- I2C sensor bring-up
- SPI OLED + framebuffer + DMA async
- 初步 driver/module 拆分

還需要補強：

- robust error handling
- UART RX 與 ring buffer
- clock tree 與 baud/timing 精準性
- non-blocking system architecture
- debugging discipline
- README/diagram/demo 作品包裝
- RTOS 基礎

結論：你現在最有價值的練習，不是再多開十個 demo，而是把 `exp12_ssd1306_spi` 變成一個穩定、有架構、有文件、能展示的完整 firmware project。這會比單純「我做過 UART/I2C/SPI」更像真正工程師的履歷訊號。
