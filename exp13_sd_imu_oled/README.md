# 實驗十三：IMU 資料記錄器（SD + FatFs + MPU6050 + OLED）

> bare-metal、register-level driver，無 HAL。STM32F407VGTX。
> 這是 exp12（SPI OLED + I2C IMU）的延伸：加上 **SPI SD 卡**、自己寫的 **SD driver**、
> 整合 **FatFs（FAT32）**，做成一台能 `start`/`stop`/`dump` 的 IMU 資料記錄器。

---

## 專案一句話

MPU6050 以 100Hz 取樣 → 經 FatFs 寫成 CSV 存進 SD 卡（拔卡即可用 Excel/Python 分析）→ OLED 顯示即時狀態 → UART 提供 `start/stop/status/dump` 命令列。

---

## 硬體接線

| 訊號 | 腳位 | 模式 | 備註 |
|---|---|---|---|
| SPI1 SCK | PA5 | AF5 | **OLED + SD 共用** |
| SPI1 MISO | PA6 | AF5 | SD 用（軟體上拉）|
| SPI1 MOSI | PA7 | AF5 | **OLED + SD 共用** |
| OLED CS | PA4 | GPIO out | |
| OLED RES | PB0 | GPIO out | |
| OLED DC | PB1 | GPIO out | |
| SD CS | PB10 | GPIO out | |
| I2C1 SCL | PB6 | AF4 | MPU6050 |
| I2C1 SDA | PB9 | AF4 | MPU6050 |
| USART1 TX | PA9 | AF7 | → CP2102/CH340 → PC |
| USART1 RX | PA10 | AF7 | ← PC（CLI 命令）|

> ⚠️ **SD 卡務必吃 3.3V**，不要接 5V（見下面「踩坑紀錄 §1」）。
> ⚠️ SD VDD–GND 建議補 `10µF + 100nF` 去耦，ACMD41 初始化是高電流動作。

---

## 系統架構（分層）

```
 Application   app_datalogger（狀態機/取樣/緩衝）, app_cli（UART 命令）
 Middleware    FatFs (FAT32)  ──  diskio.c（媒體接口層）
 Device        device_mpu6050 · device_ssd1306 · device_sdcard
 Bus / HAL     driver_i2c1 · driver_spi1 · driver_usart1 · systick
 MCU           stm32f407xx.h（register map）
```

資料流：

```
MPU6050 ─I2C1→ 100Hz 取樣 ─→ FatFs/CSV ─SPI1→ SD 卡  (log_NNN.csv)
                              └────────→ OLED (SPI1) 顯示狀態 ~10Hz
UART(USART1) ←→ CLI: start / stop / status / dump
```

---

## SD 卡 driver（自己刻，SPI 模式）

`device_sdcard.c` 完整初始化序列：

| 步驟 | 命令 | 作用 |
|---|---|---|
| 1 | 降速 ≤400kHz + 80 dummy clock | 喚醒卡 |
| 2 | **CMD0** (GO_IDLE_STATE) | 進 idle，R1=0x01 |
| 3 | **CMD8** (SEND_IF_COND) | 判 v1.x / v2.0+、確認電壓 |
| 4 | **ACMD41** (SD_SEND_OP_COND) | 反覆送到卡離開 idle（R1=0x00）|
| 5 | **CMD58** (READ_OCR) | 讀 CCS bit → SDHC（block 定址）/ SDSC（byte 定址）|
| 6 | 鮑率拉到 4MHz | init 完成加速 |

讀寫：`sd_read_block` (CMD17) / `sd_write_block` (CMD24)，單一 512B block。
SDHC 用 block 號定址、SDSC 用 byte 位址，由 `sd_block_to_addr()` 處理。

---

## CSV 格式

```
t_ms,ax,ay,az,gx,gy,gz
22323,-272,-100,18404,-455,-110,-68
22334,-196,44,18372,-432,-117,-70
...
```

- `t_ms` = `systick_get_ms()`
- accel/gyro 目前記 **raw int16**（±2g → 16384 LSB/g、±250°/s → 131 LSB/(°/s)）
- scaling 成 g / °/s 是下一步（為省 code，會用定點而非開 float printf）

---

## UART CLI

| 命令 | 動作 |
|---|---|
| `start` | 開新檔 `log_NNN.csv`、寫表頭、進 LOGGING |
| `stop` | `f_sync` + `f_close`、回 IDLE |
| `status` | 顯示狀態、檔名、已記錄筆數 |
| `dump` | 把上一個 CSV 從卡讀回來印到 UART（**沒讀卡機也能驗證真實資料**）|
| `ls` | 列出卡上檔案 + 大小 + 剩餘空間（像真的設備 console）|
| `clear` | 刪掉所有 `log_NNN.csv`（重複 demo 前清場）|

> 檔名**自動遞增**（`log_000` → `log_001` → …），永不覆蓋舊資料 —— 這是刻意的設計（資料安全）。`clear` 才會清檔。

---

## 開發歷程（心路歷程）

整個專案是一步一腳印疊上去的，每一步都先獨立驗證再往上加：

### 1. 從 CMD0 開始，刻 SD 卡 SPI driver
原本 `driver_spi1` 是 OLED 用的 **TX-only**（送完就把 MISO 丟掉）。SD 卡是「送命令／收回應」成對動作，所以先加了**全雙工** `spi1_transfer_byte()` 和可調鮑率的 `spi1_set_baud()`，再寫 CMD0 喚醒 + 進 idle。

### 2. 補完開卡序列 CMD8 → ACMD41 → CMD58
判斷卡片版本、把卡帶離 idle、讀 OCR 判 SDHC/SDSC。一路用 UART 印 R1 驗證每一步。

### 3. block 讀寫 CMD17 / CMD24
把命令框架抽成 `sd_cmd_frame()`（不碰 CS），讓資料傳輸階段能維持 CS 拉低。寫 pattern → 讀回 → 逐 byte 驗證通過。

### 4. 掛上 FatFs（FAT32）
寫 `diskio.c` 把 FatFs 的 `disk_read/write/initialize` 接到 `sd_*_block`/`sd_init`，設定 `ffconf.h`。`f_open`→`f_printf`→`f_close`→重開讀回，成功在卡上產生 PC 讀得到的 `LOG.CSV`。

### 5. 整合成 datalogger
加 `app_datalogger`（狀態機 + 100Hz 取樣 + 每秒 `f_sync`）和 `app_cli`（UART 命令），主迴圈走 co-operative scheduler。把 OLED 狀態顯示也接上 —— **然後就踩到下面那個雷。**

---

## 踩坑紀錄（值得講的 bug）

### §1. 「卡能溝通，卻死不初始化」其實是 5V 灌進 3.3V 腳
ACMD41 一直回 `R1=0x01`（停在 idle）、UART 字元糊掉、MCU 還間歇重開機。
一開始往「卡狀態」「時序」查，後來發現三個現象（UART 糊 + 重開機 + init 不完成）**只有電壓被拉垮能同時解釋**。
根因：**SD 卡的 5V 被接到了 3.3V 腳**。CMD0/CMD8/CMD55 這種低電流動作能跑，但 ACMD41 要驅動內部 flash，電不夠就完成不了。
> 教訓：CMD8 只是 echo 你宣告的電壓範圍，**不代表實際 VDD 正確**。

### §2.（主角）OLED 一接上，SD 寫進去的 CSV 就截斷／空白
**現象**：只有 SD 的測試一切正常；一把 OLED 顯示加進主迴圈，記錄出來的檔案就出問題 —— `log_000.csv` 尾巴被截斷、`log_001.csv` 整個空的（連表頭都沒有）。

**追查**：資料本身是對的（重力、gyro offset 都合理），代表 SD 讀寫「大致」能動，只是**間歇被汙染**。關鍵線索是「換了 OLED 才壞」。

**根因**：OLED 推 framebuffer 用的是 **TX-only DMA**（`spi1_send_buffer`，1024 byte），**全程不讀 SPI 的 RX**。傳完後 SPI 的 **OVR（overrun）旗標被設起來、DR 裡卡著一個沒被讀走的舊 byte**。
接著 SD 卡呼叫 `spi1_transfer_byte()`（寫 DR → 等 RXNE → 讀 DR）時，RXNE 因 OVR 早就是 set 的 → 它**直接讀到那個舊 byte、根本沒把要送的 byte 時脈出去** → SD 的命令/資料流被汙染。每 100ms 一次 OLED 更新就汙染一次 SD，於是 FatFs 寫入間歇壞掉。

> 兩個裝置共用一條 SPI，**一個用 DMA-TX-only、一個用全雙工 polling**，前者留下的 OVR 髒狀態會咬到後者。這是「共用匯流排」最隱晦的坑。

**修法**：在所有 DMA 傳輸的收尾（`spi1_wait_done()`）把 RX 排空、清 OVR —— 清 OVR 的標準順序是**先讀 DR、再讀 SR**：
```c
void spi1_wait_done(void) {
    while (!s_dma_done);
    while (SPI1->SR & (1 << 7));   // 等 BSY=0
    (void)SPI1->DR;                // 清 RXNE + 讀走殘留 RX
    (void)SPI1->SR;                // 清 OVR
}
```
OLED 所有 DMA 路徑都會經過 `spi1_wait_done`，改這一處即可根治。

---

## 下一步（規劃）

- **MPU6050 scaling**：raw → g / °/s（定點，不開 float printf）
- **FreeRTOS 化**：拆 Sensor / Logger / Display / Console 四個 task
  - **SPI1 共用 → 上 mutex**（§2 那個坑在 RTOS 下會更明顯：兩個 task 搶匯流排）
  - Sensor→Logger 用 Queue 解耦（SD 寫入偶有 stall，不能拖累取樣頻率）
- SD 多 block 讀寫（CMD18/25）加速大量寫入

---

## 檔案結構

```
Src/
  main.c              co-operative 主迴圈
  app_datalogger.c    狀態機 + 取樣 + 緩衝寫入
  app_cli.c           UART 命令列
  diskio.c            FatFs 媒體接口層（接 sd_*_block）
  ff.c                FatFs R0.15a（第三方）
  device_sdcard.c     SD 卡 SPI driver（CMD0/8/17/24/55/58, ACMD41）
  device_mpu6050.c    IMU（I2C）
  device_ssd1306.c    OLED（SPI + DMA）
  driver_spi1.c       SPI1 + DMA2 Stream3（含 OVR 清除修正）
  driver_i2c1.c       I2C1（timeout + bus recovery）
  driver_usart1.c     USART1（DMA TX + 中斷 RX）
  systick.c           1ms tick
Inc/
  app_datalogger.h app_cli.h device_*.h driver_*.h ff.h ffconf.h diskio.h ...
```
