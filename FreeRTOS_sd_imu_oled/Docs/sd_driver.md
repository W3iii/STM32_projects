# SD Card SPI Driver — 技術總覽

> STM32F407VGTX / SPI1 / FatFs
> 對應檔案：`Src/device_sdcard.c`、`Inc/device_sdcard.h`、`Src/diskio.c`
> （§10 的恢復流程另外牽涉 `Src/app_datalogger.c`、`Src/app_tasks.c`、`Src/app_cli.c`）

---

## 0. 三十秒版本

> 「SD 卡在 SPI 模式下的 bare-metal driver。負責開卡協商（CMD0/CMD8/ACMD41/CMD58）、
> 單 block 讀寫（CMD17/CMD24），以 `diskio.c` 接上 FatFs。
> 設計重點有三個：**init 階段的速率與時序約束**、**三種不同性質的等待各用不同策略**、
> 以及 **在 RTOS 下不讓忙等吃掉 CPU**。」

這三點就是後面章節的骨架，也是最可能被追問的地方。§10 另外記錄一次實際故障：
供電邊界下寫入卡死，從症狀判讀到三層恢復的完整設計。

---

## 1. 硬體與分層

### 接線

| 訊號 | 腳位 | 說明 |
|---|---|---|
| SCK | PA5 | SPI1, AF5 |
| MISO | PA6 | SPI1, AF5 |
| MOSI | PA7 | SPI1, AF5 |
| CS | **PB10** | 普通 GPIO 推挽輸出，**軟體控制** |

CS 刻意不用 SPI 的硬體 NSS。理由：SD 協定要求 CS 在**整個資料階段**（命令 + 512B + CRC + busy 輪詢）持續拉低，而硬體 NSS 會按 byte 自動抬起放下。這類「交易跨多個 byte」的裝置一律用軟體 CS。

### 分層

```
app_datalogger.c          <- 業務邏輯，持有 spi1_lock
      |  f_write / f_read
ff.c  (FatFs)
      |  disk_read / disk_write / disk_ioctl
diskio.c                  <- glue：LBA -> block 號、DSTATUS/DRESULT 轉換
      |  sd_read_block / sd_write_block
device_sdcard.c           <- 本文主角：SD 協定狀態機
      |  spi1_transfer_byte / spi1_send_buffer / spi1_poll_yield
driver_spi1.c             <- 暫存器層，不含任何 RTOS 相依
```

**關鍵設計：`device_sdcard.c` 不 include 任何 FreeRTOS 標頭。** 它要讓出 CPU 時呼叫
`bus_port.h` 宣告的 `spi1_poll_yield()`，由 `bus_port_freertos.c` 實作成
`vTaskDelay(1)`；bare-metal 版本則實作成空函式。這讓同一份 driver 能在
RTOS 與非 RTOS 專案裡共用。

---

## 2. 初始化序列

`sd_init()` 共七步。**每一步都是「不做就壞」，沒有一步是裝飾性的**：

| # | 動作 | 不做會怎樣 |
|---|---|---|
| 1 | SPI 降速到 **<=400kHz**（`SD_BAUD_INIT`=7，/256 -> 328kHz） | 卡在未初始化狀態只保證支援 400kHz 以下，高速下不回應或回 garbage |
| 2 | **CS 拉高**，送 >=74 個 clock（10 × `0xFF` = 80 clock） | 卡的內部電源電路沒跑完 ramp-up，後續命令全部無效 |
| 3 | **CMD0** (GO_IDLE_STATE)，CRC `0x95` | 卡不會切進 SPI 模式（見 §3） |
| 4 | **CMD8** (SEND_IF_COND)，arg `0x1AA`，CRC `0x87` | 無法分辨 v1.x / v2.0+ -> 定址模式會錯（見 §7） |
| 5 | **ACMD41** (SD_SEND_OP_COND) 反覆送到 `R1 == 0x00` | 卡停在 idle，所有讀寫都被拒絕 |
| 6 | **CMD58** (READ_OCR)，取 CCS bit | 不知道該用 block 還是 byte 定址 |
| 7 | 加速到 `SD_BAUD_FAST`=4（/32 -> 2.6MHz） | 不做只是慢，不會壞 |

### 第 2 步的細節：為什麼 CS 要**拉高**

這是最容易記錯的一點。那 74 個 clock 必須在 **CS = high** 時送出，因為此刻的目的只是給卡內部電路時鐘去完成 power-up，**還不是在跟它對話**。CS 拉低才代表「我要選中你」。

### 第 3 步的細節：SPI 模式是怎麼進去的

卡在 power-up 後預設是 **SD bus mode**（原生 4-bit）。它判斷要切到 SPI 的依據是：**CMD0 期間 CS 是否為低**。所以 CMD0 身兼兩職 —— 重置狀態機，同時宣告介面模式。

### 第 5 步的細節：ACMD41 的 argument

```c
uint32_t acmd41_arg = (sd_card_type == SD_TYPE_V2) ? 0x40300000 : 0x00300000;
```

- **bit30 = HCS** (Host Capacity Support)：告訴卡「我支援高容量」。**只有 v2 卡要設。**
  對 SDHC 卡送 HCS=0，卡不會完成初始化 -> 走到 `SD_ERR_ACMD41` 逾時。
- **bits21:20 = `0x00300000`**（3.2~3.4V 電壓視窗）：**規格上 SPI 模式的 ACMD41 只看 HCS，
  OCR 電壓欄位是被忽略的**（ChaN 的 FatFs 參考實作只送 `0x40000000`）。
  這裡照樣給是針對不完全合規的卡的防禦性 workaround，對合規卡無害。

> 面試可能追問：「CMD8 不是已經給過電壓了？」
> 答：CMD8 的 VHS 只是 **echo-back 的相容性測試**，卡不會因此做任何事；
> 真正的電壓能力回報在 CMD58 讀回的 OCR。整套電壓協商在 SPI + 固定 3.3V 的
> 場景裡基本是走過場，繼承自 MMC 時代。

### 抗雜訊：三處重試

CMD0 / CMD8 / CMD58 各有 **10 次重試**迴圈。原因是麵包板走線在 init 的低速階段容易吃掉 trailing bytes（CMD8 的 R7 要連讀 5 個 byte），單發很容易失敗。

CMD8 判 v1 時用**精確值** `r1 == 0x05` 而非 `r1 & 0x04`：

```
0x05 = idle(0x01) + illegal command(0x04)
```

用遮罩的話，雜訊造成的 `0xFF` 也會讓 `& 0x04` 成立，把 v2 卡誤判成 v1 -> 定址模式錯 -> 靜默資料毀損。**這是整個 driver 最不能寫錯的一行判斷。**

---

## 3. 命令 frame 與 CRC

### 48-bit frame 結構

| bit | 內容 | 本 driver |
|---|---|---|
| 47 | start bit = **0** | `0x40 \| cmd` 的高位 |
| 46 | transmission bit = **1** | 同上 |
| 45:40 | command index (6 bit) | `cmd` |
| 39:8 | argument (32 bit) | `arg`，MSB first 分 4 byte |
| 7:1 | **CRC7** | 見下表 |
| 0 | end bit = **1** | **永遠是 1** |

最後一個 byte = `CRC7 << 1 | 1`。

### CRC 一覽

| 命令 | CRC byte | 必要性 |
|---|---|---|
| **CMD0** | `0x95` | **必須正確** |
| **CMD8** | `0x87` | **必須正確** |
| CMD55 / ACMD41 / CMD58 / CMD17 / CMD24 | `0x01` | 免檢 |

驗證這個結構：`0x95` = CRC7 `0x4A` << 1 \| 1；`0x87` = CRC7 `0x43` << 1 \| 1。**兩個都是奇數，不是巧合。**

### 為什麼只有 CMD0 和 CMD8 —— 兩個理由完全不同

**CMD0：機制上的必然。** 卡解析 CMD0 時還在 SD bus mode 的規則下，而 SD bus mode 的 CRC 檢查是強制、不可關閉的。等 CMD0 被接受、切進 SPI 模式，「CRC 預設 OFF」才生效。這是雞生蛋，躲不掉。

**CMD8：規格刻意保留的例外。** 此時卡已在 SPI 模式、CRC 已 OFF，但規格明文要求檢查。合理解釋是賭注太大 —— CMD8 的結果決定定址模式（§7），誤判會造成**靜默的全卡資料毀損**。CRC 保證「illegal command」這個回答的語意乾淨：代表「我真的不支援」而非「我沒聽清楚」。

### 「CRC 關了」不等於「那個 byte 可以亂給」

`0x01` 的 bit0 是 **end bit**，不能是 0。送 `0x00` 會讓 frame 格式不合法，卡不回應（跟 CRC 無關）。

- `0x01`、`0x03`、`0xFF`、`0x95` -> 都能用（bit0 = 1）
- `0x00`、`0x42`、`0xFE` -> **不行**

免檢省掉的是「算 CRC7」，不是「湊出合法 frame」。

### CRC 關閉的範圍與風險

| 位置 | 誰算 | 本 driver 處理 |
|---|---|---|
| 命令 frame 的 CRC7 | host | CMD0/CMD8 給真值，其餘 `0x01` |
| 讀取資料後的 CRC16 | 卡送 | 讀掉丟棄 |
| 寫入資料後的 CRC16 | host 送 | 送 `0xFF 0xFF` |

三處都不被檢查，全繫於一件事：**沒有發 CMD59 (CRC_ON_OFF)**。發了它，三格同時變成必須正確。

---

## 4. 回應型別與資料 token

### 回應

| 型別 | 長度 | 用在 | 判讀 |
|---|---|---|---|
| **R1** | 1 byte | 多數命令 | MSB=0 才有效。`0x01`=idle、`0x00`=ready、`0x05`=illegal cmd |
| **R3** | R1 + 4 byte | CMD58 | 後 4 byte = OCR，`ocr[0]` bit6 = **CCS** |
| **R7** | R1 + 4 byte | CMD8 | `r7[2]`=VHS echo、`r7[3]`=pattern echo（應為 `0xAA`） |

R1 的等待上限是 `N_CR` = 1~8 bytes，所以 `sd_cmd_frame()` 連讀最多 8 次等 MSB=0。

### 資料 token

| token | 值 | 用途 |
|---|---|---|
| start block（讀 / 單 block 寫） | `0xFE` = `0b1111_1110` | 本 driver 用的 |
| start block（多 block 寫） | `0xFC` | 未使用 |
| stop tran | `0xFD` | 未使用 |
| **error token** | `0b0000_xxxx` | 高 4 bit 全 0，低 4 bit 是錯誤旗標 |

**為什麼 start token 是 `0xFE`：** 它必須自我定界。host 不知道卡會 busy 幾個 byte，不能用「第 N 個 byte」定位，只能靠內容。`0xFE` 的性質是：

1. **一串 1 之後出現單一個 0** -> 那個 0 就是明確邊界（idle 線是全 1，任何非 `0xFF` 都代表卡說話了）
2. **0 落在 LSB（最後送出的 bit）** -> 之後緊接資料第一個 bit，零 padding、不必重新對齊 byte 邊界

start token 與 error token 在最高位就分開（`1` vs `0`），不可能混淆。

寫入後的 **data response token** 格式 `xxx0_sss1`，LSB 強制為 1，`sss` = 010 接受 / 101 CRC error / 110 write error，所以 `(resp & 0x1F) == 0x05` 才算接受 —— **又是用固定 bit 當框架標記**，跟命令的 end bit 同一套思路。

---

## 5. 讀寫路徑

### 讀（CMD17）

```
CS low -> 等卡 ready（讀到 0xFF，見 §10.4）
       -> CMD17 -> R1 必須 == 0x00
       -> 等 start token 0xFE（有 timeout + yield）
       -> 逐 byte 讀 512 bytes
       -> 讀掉 2 byte CRC16（丟棄）
       -> CS high + 1 dummy byte
```

### 寫（CMD24）

```
CS low -> 等卡 ready（讀到 0xFF，見 §10.4）
       -> CMD24 -> R1 必須 == 0x00
       -> 1 dummy byte（N_WR 間隔）
       -> 送 start token 0xFE
       -> DMA 送 512 bytes           <- 與讀路徑不對稱；buf 必須在主 SRAM（見 §8）
       -> 送 2 byte 假 CRC (0xFF 0xFF)
       -> 讀 data response，(resp & 0x1F) == 0x05
       -> 輪詢等 busy 結束（回 0xFF 才算寫完）
       -> CS high + 1 dummy byte
```

### 為什麼寫用 DMA、讀不用

寫入的 512B 階段是**純送**（卡在 data-out 期間不在 MISO 上說話），正好符合
`spi1_send_buffer()` 那條 **TX-only DMA** 的前提。讀取需要 MISO 上的資料，TX-only
DMA 收不到，所以仍是 512 次 `spi1_transfer_byte()` 迴圈。

> **這是已知的優化空間**：讀取改用 full-duplex DMA（TX 餵固定 `0xFF`、RX 收進 buffer）
> 可以把 mount 階段幾十次 sector 讀取的 CPU 時間一併省掉。目前沒做。

### busy 偵測：為什麼要自己送 `0xFF`

卡寫 flash 期間確實把 DO 持續維持低電位，但**那個電位要變成一個 `uint8_t`，必須有 8 個 SCK 邊緣去取樣，而 SCK 只有 master 能產生**。

所以 `spi1_transfer_byte(0xFF)` 同時做兩件事：送出 = 產生 clock（**這才是目的**），收回 = 取樣 MISO。整個 driver 裡所有 `transfer_byte(0xFF)` 都是「**名為送、實為收**」。

dummy 固定用 `0xFF` 而非 `0x00`，是為了讓 MOSI 維持 idle 高電位 —— 避免在卡等待命令的狀態下被誤判成 frame 的 start bit（start bit = 0）。

---

## 6. 三種等待，三種策略

這是本 driver 最值得講的設計。**每個等待的規格約束都不同，所以處理方式也不同**
（標題的「三種」指 R1 / token / busy 三類延遲；命令前的 ready 是 §10 為了恢復加上的，
本質上是「上一筆寫入的 busy」換個時間點等）：

| 等什麼 | 規格允許的延遲 | 策略 |
|---|---|---|
| 讀寫命令前的 **ready** | 正常 0；上一筆寫入還在 program 時同 busy | `SD_READY_TIMEOUT_MS` = 1000ms + yield（§10.4） |
| 命令後的 **R1** | `N_CR` = 1~8 bytes | 固定讀 8 次，無 timeout（上限明確） |
| 讀取的 **start token** | `N_AC`，卡相關、可達數十 ms | `SD_READ_TIMEOUT_MS` = 200ms + yield |
| 寫入後的 **busy** | 卡相關，block erase 可達數百 ms | `SD_WRITE_TIMEOUT_MS` = 1000ms + yield |
| 寫入的 **data response** | **無**（緊接資料封包） | 單讀一個 byte |

### 混合式等待：先忙等，後讓出

```c
if (++spins > SD_SPIN_BEFORE_YIELD) {   /* 64 次 */
    spi1_poll_yield();                   /* = vTaskDelay(1) */
}
```

`64 次 × 3.1us @2.6MHz 約 200us`。卡在正常狀況下就在這範圍內回應（讀的 `N_AC` 典型 100us、寫一個 block 的 program time 通常 1~3ms），所以**常見路徑一次 yield 都不會走到，不多付任何延遲**；只有真的撞上 wear-leveling / block erase 才切到 1ms 粒度輪詢。

反過來說，**一開頭就直接睡**會讓每次讀寫無條件多付一個 tick，mount 階段幾十次 sector 讀取加起來很有感。

> **這段有實測故事可以講：** 曾經遇過一次寫入失敗，logger task 的 Peak CPU%
> 從 2.5% 跳到 **90.8%** —— 就是這個忙等迴圈把 CPU 燒滿。當次根因是供電不穩，
> 但也暴露出原本 500ms 的寫入 timeout 偏緊（block erase 停幾百毫秒是正常行為），
> 而逾時的代價是 **FatFs ABORT 會把錯誤鎖進 `fp->err`，之後每筆樣本都在
> `f_write` 開頭被打回票 -> 整個 session 從此寫不進去**。所以放寬到 1000ms。

---

## 7. 定址模式：風險最高的單一決策

```
CMD8 -> sd_card_type = V2 or V1
          |-> ACMD41 的 HCS bit 要不要設
          |-> 能不能升級成 SDHC
                    |
          if (sd_card_type == SD_TYPE_V2 && (ocr[0] & 0x40))
                  sd_card_type = SD_TYPE_SDHC;
                    |
          sd_block_to_addr():
              SDHC -> block 號
              SDSC -> block × 512
```

`SD_TYPE_V2` 是升級成 SDHC 的**必要前提**。如果 CMD8 判斷錯，對 SDHC 卡用 byte 定址，每次存取的位址被乘 512 倍。sector 0 剛好對（offset 0），所以**掛載可能成功**，但往後每個 sector 都錯位，寫入散落整張卡 —— 而 CRC 是關的，不會有任何錯誤回報。

**這是「看起來在正常運作、實際全盤錯位」的故障模式**，也是本 driver 把 CMD8 失敗設成硬性錯誤（`SD_ERR_CMD8`）、並在 R1 判斷上用精確值比對的原因。

---

## 8. RTOS 整合

| 機制 | 位置 | 說明 |
|---|---|---|
| **匯流排互斥** | `app_datalogger.c` 的 `spi1_lock()` | SPI1 由 SD 卡與 OLED 共用，所有 FatFs/SD 操作包在鎖內 |
| **鎖序** | `spi1_lock` 在外、`s_state_mtx` 在內 | 只有 writer 會巢狀，固定順序避免死鎖 |
| **讓出 CPU** | `spi1_poll_yield()` | `vTaskDelay(1)`，**不放鎖** |
| **driver 中立性** | `bus_port.h` | `device_sdcard.c` 不 include 任何 RTOS 標頭 |

### 一個已知的副作用

busy 期間 CS 必須持續拉低，而整段又包在 `spi1_lock` 裡，`spi1_poll_yield()` 只讓 CPU、**不放匯流排鎖**。所以遇到 block erase 停幾百毫秒時：CPU 讓出去了（好），但 OLED task 會卡在 `spi1_lock` 上同樣的時間 —— **畫面凍住幾百 ms**。

這是**刻意的取捨**，不是 bug：SD 卡 busy 時不能放掉 CS 去讓 OLED 用匯流排再回來接著 poll（卡會釋放 DO，就失去 busy 訊號，得改用重新 select + CMD13 查狀態的流程）。拿畫面偶爾頓一下換掉那套複雜度是划算的。

> 值得知道這條因果：「OLED 偶爾頓一下」的根因在 SD driver 的 busy 迴圈，
> 不在 OLED 或 SPI 本身。

### CCM 限制：寫入 buffer 必須在主 SRAM

FreeRTOS 的 `ucHeap`（28KB）放在 **CCM**（`.ccmbss`，`0x1000_0000`），
由 `HEAP_IN_CCM` 開關控制。CCM 只掛在 Cortex-M4 的 D-bus 上，**不在 bus
matrix 上，DMA1/DMA2 碰不到**。

而 task stack 是從 heap 配出來的 —— 所以**所有 task 的 local 陣列都在 CCM**。

對本 driver 的影響：`sd_write_block()` 的 `buf` 會被直接交給
`spi1_send_buffer()` 做 DMA，**必須落在主 SRAM**。FatFs 在單次
`f_write()` 長度 >= 512B 時會把呼叫端的 buffer 直接傳給 `disk_write`，
所以「task 裡宣告一個 `uint8_t tmp[512]` 丟進 `f_write`」會失敗。

目前安全，因為 `FIL` / `FATFS` 都是 static（在主 SRAM）。

防線在 `dma_mem.h` 的 `dma_mem_ok(buf, len)`：白名單式檢查，整段
`[buf, buf+len)` 要落在主 SRAM（`0x2000_0000`–`0x2001_FFFF`）或
Flash 才放行。檢查不過就不啟動 DMA、回 `-1`，SD 寫入因此走既有的
`SD_ERR_WRITE` -> `FR_DISK_ERR` -> OLED `!ERR` 路徑，**不會靜默失敗**。

> 面試可以講的點：這是個「把註解裡的規則變成程式碼」的例子。
> 原本「buffer 要在 DMA 碰得到的地方」只寫在註解裡，heap 搬進 CCM 之後
> 違規的代價變高（錯誤在 ISR 的 TEIF 才浮現，離原因很遠），所以加了
> 執行期檢查，並且刻意**回錯誤碼而非 `configASSERT`** —— driver 層不依賴
> FreeRTOS，而且回錯誤碼能接上既有的錯誤處理，不必把整台停掉。

---

## 9. FatFs glue（`diskio.c`）

| FatFs 介面 | 實作 |
|---|---|
| `disk_status` | 回 `s_stat`（`STA_NOINIT` 旗標） |
| `disk_initialize` | 呼叫 `sd_init()`，成功則清 `STA_NOINIT`；回傳碼存進 `sd_last_init_rc` 供診斷 |
| `disk_read` / `disk_write` | **for 迴圈逐 sector** 呼叫 `sd_read_block` / `sd_write_block`；`disk_write` 每個 sector 失敗後最多再重試 3 次（§10.4） |
| `disk_ioctl` | **只實作 `CTRL_SYNC`，且是空操作** |
| `get_fattime` | 固定值（`FF_FS_NORTC == 1` 時其實不會被呼叫） |

**`CTRL_SYNC` 為什麼可以空實作：** `sd_write_block()` 本身已經輪詢到卡 busy 結束才返回，資料已經落地，沒有需要 flush 的 driver 層快取。

---

## 9.5 診斷出口：錯誤碼往上走會被壓扁

這是分層架構的固有代價，值得單獨講。

### 壓扁鏈

```
sd_write_block 的 5 個失敗點
  ├─ CMD24 的 R1 != 0
  ├─ buffer 被 dma_mem_ok 擋下（CCM）
  ├─ DMA/SPI 傳輸逾時
  ├─ data response 被拒
  └─ busy 輪詢逾時
          ↓  全部回同一個 SD_ERR_WRITE
    diskio.c  ->  RES_ERROR
          ↓
    FatFs     ->  FR_DISK_ERR，並鎖進 fp->err
          ↓
    status 的 "write err N (FRESULT 1)"
```

讀取同理，3 個失敗點全壓成 `SD_ERR_READ`。

問題是 `FR_DISK_ERR` 背後這幾種原因的**處理方式完全不同**：

| 實際原因 | 該做什麼 |
|---|---|
| 卡被拔掉 / 接觸不良 | 插回去、查座子 |
| 供電不穩導致 busy 逾時 | 查電源、加電容 |
| buffer 在 CCM 被擋 | **改 code** |
| 線路雜訊造成假 token | 查接線、降速 |

### 解法：在最細的那層留快照

| 路徑 | 診斷出口 |
|---|---|
| **init** | `sd_last_init_rc`、`sd_last_r1`、`sd_last_cmd55_r1`、`sd_card_type`、`sd_r7`、`sd_ocr` —— 開機報告與 OLED 都印 |
| **I/O** | `sd_last_io_err`（`SD_IO_*`）、`sd_last_io_byte`（當時的 R1/token/resp）、`sd_last_io_blk` |

`status` 的 `[sd card]` 區多一行（只在 `write err` 或 `retry` 非 0 時印，否則快照無意義）：

```
  last err  wr busy timeout  byte 0x00  block 32768
```

三個設計細節：

1. **成功時刻意不清掉。** 失敗後通常還有成千上萬次成功寫入，清掉就查不到了。要判斷快照是否有效，看 `err_count` 和 `sd_write_retries`（重試成功時只有後者會動，見 §10.5）。
2. **寫入順序是 byte/blk 先、err 最後。** 這樣 CLI 看到 `err != 0` 時，另兩個欄位一定已經更新完。
3. **`spi1_send_buffer()` 的兩種失敗用 `dma_mem_ok()` 回頭分類。** 它對「位址被擋」和「DMA 逾時」都回 `-1`，但一個要改程式、一個要查硬體，壓成同一碼等於丟掉最有用的線索。

> **原則：錯誤碼的粒度在最底層最細，每往上一層就少一點。** 所以要在最細的地方
> 留快照，而不是試圖把細節塞進上層的回傳值 —— 後者會逼你改 `diskio` 甚至
> FatFs 的介面，代價高得多。這裡的改動沒有動任何既有介面。

---

## 10. 實戰案例：寫入卡死的偵測、吸收與恢復

> 一次實際踩到的故障，從症狀一路推回根因，再設計成三層防線。
> 前面各章講「driver 怎麼運作」，這章講「它壞掉時怎麼辦」。

### 10.1 症狀

錄製途中 OLED 從 `* REC` 變成 `!ERR 1`，計數停住。CLI 的 `status`：

```
  write err 1
  sd retries3
  sd io err not ready before cmd (byte 0x00, block 16384)
  chip      31.0 C  (VDDA 2.78 V)
```

（這是改版前的 status 格式，兩個顯示 bug 也在裡面，見 §10.5。）

`!ERR 1` 的 `1` 是 FatFs 的 FRESULT = `FR_DISK_ERR`，代表 `disk_write` 回了 `RES_ERROR`。
手動 `stop` 再 `start` 之後恢復正常。

### 10.2 判讀：從 status 推回根因

每一行都是一條線索，合起來只有一個解釋：

| 線索 | 代表什麼 |
|---|---|
| `retries 3`、`write err 1` | 同一個 sector 試了 4 次（1 次 + 重試 3 次）全失敗，才往上回報 |
| `not ready before cmd`、`byte 0x00` | 最後一次嘗試在**送命令前**等了 1s，卡始終把 MISO 拉低 = 仍在 busy |
| 兩者合起來 | 第 1 次 busy 逾時 1s + 後 3 次各等 ready 1s → 卡**連續 busy 至少 4 秒** |
| `VDDA 2.78 V` | 3.3V 電源只剩 2.78V，SD 卡規格下限是 2.7V |
| `stop` → `start` 就好 | 卡本身沒壞，從 CMD0 重新初始化即可恢復 |

結論：**供電邊界下，卡寫 flash 的瞬間電流把電壓拉到下限以下，卡的控制器卡死在 busy。**
SDHC/SDXC 規格的寫入 busy 上限是 500ms，4 秒不是「卡比較慢」，是「卡已經不會自己回來」。

**VDDA 這個數字可信嗎？** `tempsensor_vdda()` 不是用 VREFINT 的 typ 值（1.21V，實際分佈
1.18~1.24V，誤差 ±80mV）硬算，而是用 ST 出廠時在 VDDA = 3.3V 下量好、燒在
`0x1FFF7A2A` 的 `VREFINT_CAL`，誤差約幾十 mV，足以判斷「是不是掉到 2.7V 附近」。
但它量的是 **MCU 的 VDDA**；SD 模組若經過自己的 LDO（如 AMS1117）供電，實際電壓
只會更低，要用電表在模組 VCC 腳確認。過去也在 2.74V 下踩過 `f_close` 寫不回 FAT
（見 `app_datalogger_stop()` 的註解），同一個根因。

### 10.3 為什麼一次失敗就毀掉整個 session

根因在硬體，但軟體讓代價被放大了：

```
sd_write_block 失敗一次
  → disk_write 回 RES_ERROR
  → FatFs ABORT：把 FR_DISK_ERR 鎖進 fp->err
  → 之後每一次 f_write 都在函式開頭直接返回，連卡都不碰
  → logger 進 DL_ERROR，sensor 停止取樣
  → 只能等人來 stop / start
```

`fp->err` 是黏住的，沒有「下一次就好了」這回事。所以設計目標是：
**能吸收的錯誤不要讓它到達 FatFs；吸收不了的，自動恢復而不是等人。**

### 10.4 設計：三層，各擋一種失敗

失敗按嚴重度分成三種，每一層只處理自己那一種，處理不了才往上交：

| 層 | 擋什麼 | 機制 | 位置 |
|---|---|---|---|
| ① driver | 上一筆還沒寫完就送下一個命令 | 每筆讀寫前先等卡 ready | `device_sdcard.c` `sd_wait_ready()` |
| ② diskio | 偶發的單次失敗（busy 稍長、雜訊吃掉 response） | 同一個 sector 最多再寫 3 次 | `diskio.c` `disk_write()` |
| ③ app | 卡真的卡死，重試救不回來 | 重置卡、收拾舊檔、開新檔、接著錄 | `app_datalogger.c` `app_datalogger_recover()` |

#### ① driver 層：命令前先等 ready

**這層不做，② 的重試幾乎必定失敗。** 原因是一個很隱蔽的誤判：

```
上一筆在 busy 階段逾時放棄 → 卡其實還在寫 flash，MISO = 0x00
→ 重試直接送 CMD24
→ sd_cmd_frame() 等 R1 的條件是「MSB = 0」，0x00 符合
→ 把 busy 訊號誤認成 R1 = 0x00（命令接受）
→ 送出 512B 資料，卡根本沒在聽
→ data response 讀到 0x00，(0x00 & 0x1F) != 0x05 → 失敗
```

所以 `sd_read_block` / `sd_write_block` 在 CS 拉低後、送命令前先呼叫：

```c
static uint8_t sd_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = systick_get_ms();
    uint32_t spins = 0;
    uint8_t  b;
    while ((b = spi1_transfer_byte(0xFF)) != 0xFF) {    /* 0xFF = 卡放開 DO */
        if (systick_get_ms() - start > timeout_ms) {
            break;
        }
        if (++spins > SD_SPIN_BEFORE_YIELD) {
            spi1_poll_yield();                          /* 跟 §6 同一套混合式等待 */
        }
    }
    return b;                                           /* 0xFF = ready */
}
```

| 決策 | 理由 |
|---|---|
| 逾時 `SD_READY_TIMEOUT_MS` = `SD_WRITE_TIMEOUT_MS`（1s） | 等的就是「上一筆寫入的 busy」，同一個上限 |
| 回傳最後讀到的 byte 而不是 0/-1 | 逾時時把它存進快照（`SD_IO_NOT_READY`，byte = 那個值），`0x00` = busy 卡住、其他值 = 線路異常 |
| **init 的命令不走這裡** | CMD0 之前卡的狀態未知，那段有自己的 10 次重試（§2）|
| 正常路徑成本 | 卡早就 ready，第一個 byte 就是 `0xFF`，每筆讀寫只多約 3µs |

ChaN 的 FatFs 範例 driver 也是在每個命令前 `wait_ready`，只對 CMD12 例外
（CMD12 中止多 block 讀取時卡還在吐資料）；本 driver 沒有多 block，不涉及。

#### ② diskio 層：重試

```c
#define DISK_WRITE_RETRIES  3       /* 每個 sector 失敗後再試幾次（不含第一次）*/

uint8_t rc = sd_write_block(blk, src);
for (int r = 0; rc != SD_OK && r < DISK_WRITE_RETRIES; r++) {
    if (sd_last_io_err == SD_IO_WR_DMA_MEM) {
        break;                      /* buffer 在 CCM 是程式錯誤，重試也一樣 */
    }
    sd_write_retries++;
    rc = sd_write_block(blk, src);
}
```

| 問題 | 回答 |
|---|---|
| 重寫安全嗎？ | 安全。全是單 block CMD24，同一個 block 寫同樣的資料是冪等的。**若改成多 block（CMD25），中途失敗要先送 Stop Tran token `0xFD` 讓卡離開接收狀態，不能直接重試** |
| 哪些不重試？ | `SD_IO_WR_DMA_MEM`（buffer 在 CCM）—— 重試幾次結果都一樣。判斷直接讀 `sd_last_io_err`：它只在失敗時寫入，剛失敗時讀到的一定是這次的原因 |
| 重試之間要 delay 嗎？ | 不用。下一次 `sd_write_block` 開頭的 ①會等卡 ready，等待時間由卡決定，不是猜一個數字 |
| 代價？ | 最壞每次嘗試 = 等 ready 1s + 等 busy 1s，4 次約 8s，期間佔著 `spi1_lock`（OLED 停住）。只有卡真的卡死才會走到 |
| 重試成功了，怎麼知道出過事？ | 累計 `sd_write_retries`，status 印出來（§10.5）。**被吸收的錯誤如果看不到，卡在邊緣時就毫無預警** |

計數器放在 `device_sdcard.c` 而不是 `diskio.c`：CLI 刻意不 include `ff.h`，而
`sd_last_init_rc` 已經是「driver 定義、diskio 設定」的先例。

#### ③ app 層：自動恢復

**觸發**：Logger task 看到 `DL_ERROR` 後先等 1 秒（讓電源和卡喘口氣），之後每秒呼叫
一次 `app_datalogger_recover()`，直到成功或使用者 stop。

```
recover()                                     （整段持有 spi1_lock）
  │
  ├─ state 已不是 DL_ERROR？ → 返回（別人恢復了，或使用者 stop 了）
  │
  ├─ dl_salvage_locked()：收拾舊檔
  │     disk_initialize()  ← 只重置卡（CMD0 起），volume 不動
  │       ├─ 卡沒起來 → 舊檔留著，回 -1，下一秒再試
  │       └─ 卡起來了 → f_close(&s_fil)
  │             ├─ 成功：舊檔救到失敗前最後一筆
  │             └─ 失敗：f_mount(NULL) 卸載，丟掉可能不一致的快取
  │
  ├─ volume 被卸載過？ → f_mount(&s_fs, "", 1) 重新掛載
  ├─ 開新檔 log_NNN+1.csv、寫表頭
  └─ state → DL_LOGGING，s_recover_count++，喚醒 sensor
```

**關鍵決策：重置卡用 `disk_initialize()`，不用 `f_mount()`。**

直覺做法是 `f_close` → `f_mount(NULL)` → `f_mount(&fs)`，但卡卡死時 `f_close` 必定失敗，
接著 `f_mount` 一換掉 volume，`s_fil` 就失效 —— `fp->buf` 裡沒寫出去的那個 sector、
檔案長度、FAT 鏈全部跟著丟，舊檔只剩上一次 `f_sync`（每秒一次）的內容。

反過來，先直接呼叫 `disk_initialize()` 只重置卡、volume 原封不動，再 `f_close`：

- `f_sync` **不看 `fp->err`**（只有 `f_read` / `f_write` 看，可在 `ff.c` 的 `f_sync` 確認），
  它只透過 `validate()` 檢查 volume 有效且 `disk_status()` 沒有 `STA_NOINIT`。
- FatFs 在寫入失敗時**保留 dirty 旗標**（`FA_DIRTY`、`fs->wflag` 都只在成功後才清），
  所以卡恢復後，`f_close` 會把失敗的那個 sector、FAT、目錄項依序重寫一遍。
- 結果：舊檔救到失敗前最後一筆。代價是最後一行可能只寫了半行（`f_printf` 的輸出
  跨 sector 時，前半段已經進了 buffer）。

`diskio.c` 的 `disk_initialize()` 每次都真的重跑 `sd_init()`（沒有「初始化過就跳過」的
static 旗標），這是整個做法成立的前提。

**為什麼開新檔，不接回舊檔？** 斷線期間的樣本本來就沒了（sensor 只在 `LOGGING` 時取樣），
接回去也是一個有斷層的檔。新檔起點乾淨，而且不依賴舊檔的 FAT / 目錄項最後到底寫成功
沒有。`t_ms` 用的是開機後的 systick，跨檔仍然連續，斷層從兩檔的頭尾一眼就看得出來。

**併發：自動恢復、手動 start、手動 stop 可能同時發生。**

| 情境 | 處理 |
|---|---|
| Logger 自動恢復，同時使用者按 start | `start()` 在 `DL_ERROR` 下直接呼叫 `recover()`。狀態**在 `spi1_lock` 內才檢查**，後到的那個看到已是 `LOGGING` 就返回，不會開兩個檔 |
| 恢復開好新檔的同時使用者 stop | stop 先翻 `IDLE` 才等 `spi1_lock`，所以 recover 在鎖內看到 `IDLE` → 不翻回 `LOGGING`，**當場在鎖內**關掉新檔（放開鎖再關，可能誤關使用者接著 start 開的檔） |
| stop 讀到 `DL_ERROR` 後、進鎖前，recover 剛好成功 | stop 用 `s_fil_orphan`（在鎖內才準）而不是進鎖前讀的 state 決定走哪條：沒有孤兒檔就正常 `f_sync` + `f_close` 新檔 |

鎖序沿用 §8：`spi1_lock` 在外、`s_state_mtx` 在內。

**`DL_ERROR` 下的 stop 也改了。** 原本直接 `f_sync` + `f_close`，但卡多半還卡著，每個 sector
都要走滿 ② 的重試，stop 會停十幾秒。現在 stop 也走 `dl_salvage_locked(give_up = 1)`：
先重置卡，起得來就正常收尾，起不來就放棄舊檔、卸載 volume，立刻返回。

### 10.5 可觀測性：status 重新分組

舊的 status 十幾行平鋪，SD、I2C、logger 的計數混在一起，還有兩個顯示 bug：

- `sd retries3`：標籤剛好 10 個字元，塞滿 `cli_label()` 的欄寬，數字黏上去
- `byte 0x0x00`：`cli_print_hex8()` 自己會印 `0x`，呼叫端又多印一次

改成四區，每區只回答一個問題：

```
[logger]                       ← 現在在不在錄、錄到哪
  state     LOGGING
  file      log_004.csv  (12345 samples)
  overruns  0
[sd card]                      ← 卡健不健康（由上往下嚴重度遞減）
  mount     ok
  write err 1
  retry     3
  recovered 1
  last err  not ready before cmd  byte 0x00  block 16384
[i2c]                          ← MPU6050 / INA219 那條匯流排
  errors    nack 0  bus 0  timeout 0
  recovery  0
[chip]                         ← 晶片本身與供電
  temp      31.0 C
  VDDA      2.78 V  LOW (SD writes may hang)
```

`[sd card]` 區的讀法：

| 行 | 非 0 代表 | 該做什麼 |
|---|---|---|
| `mount` 不是 ok | 卡沒掛上，降級模式 | 看 FRESULT：1/3 查供電接線、13 不是 FAT32 |
| `write err` | 有樣本**真的沒寫進去** | 看 `last err` |
| `retry` | 出過事但被 ② 吸收了 | 偶爾跳無妨；持續增加 = 卡在邊緣，查供電 |
| `recovered` | 錄製中斷過、被 ③ 接回來 | CSV 分成多個檔，中間有斷層 |
| `last err` | 最後一次失敗的現場（§9.5） | 對照下表 |

`last err` 的判讀：

| 顯示 | 意義 |
|---|---|
| `not ready before cmd`，byte `0x00` | 卡 busy 超過 1s 不放，幾乎都是供電（本案例） |
| `wr CMD24 R1`，byte `0x01` / `0x05` | 卡退回 idle state，電源瞬斷，重試救不回來，要靠 ③ |
| `wr resp rejected` / `wr busy timeout` | 寫入本身失敗，偶發是雜訊、連續是供電 |
| `wr buf not DMA-able` | buffer 在 CCM，改程式（§8） |
| `rd ...` | 讀取失敗，`disk_read` 沒有重試（§10.7） |

`last err` 的顯示條件從「`err_count != 0`」改成「`err_count` 或 `retry` 非 0」：
重試成功時 `err_count` 不會動，舊條件會把被吸收的錯誤現場藏起來。

`VDDA` 低於 3.0V 標 `LOW`：規格下限 2.7V，加上寫入瞬間的壓降，實測 2.74V / 2.78V 都出過事，
留 0.3V 餘裕。

### 10.6 驗證方法

困難在於：被改到的路徑正常情況下不會走到。所以分「沒改壞」和「真的會動」兩部分：

**① 回歸**：正常錄 2~3 分鐘 → `retry 0`、`write err 0`、Peak CPU% 與改前相同（約 2.5%）、
`dump` 出來 `t_ms` 每行間隔約 10ms 且行數等於 samples。重開機確認掛載正常（掛載階段每次
讀取都會多走一次 ①）。

**② 故障注入（測 ① + ②）**：不寫假錯誤，而是**暫時把寫入 busy 逾時調短**，讓真的 busy 逾時發生：

```c
#define SD_WRITE_TIMEOUT_MS   2      /* 測試用，原本 1000 */
#define SD_READY_TIMEOUT_MS   1000   /* 跟寫入逾時拆開，維持 1s */
```

卡寫一個 block 的 program time 本來就 1~3ms，所以一部分寫入會在 busy 階段被放棄 ——
正是本案例「放棄時卡還在寫」的情境。

| 組別 | 預期 |
|---|---|
| 實驗組（① + ②） | `retry` 持續增加、`write err 0`、state 維持 `LOGGING`、`last err` = `wr busy timeout`、`dump` 無斷層 |
| 對照組（註解掉 ①） | 很快 `write err`，`last err` 變成 `wr resp rejected` / `wr CMD24 R1` —— 0x00 被誤判成 R1 的證據 |

實驗組過、對照組炸，就證明 ① 和 ② 缺一不可。測完務必把兩個巨集改回來。

**③ 拔卡（測 ③）**：錄製中拔卡 → `!ERR` → 插回 → 1~2 秒內自動回到 `* REC`、計數從 0 開始、
`recovered 1`、檔名跳號。`stop` 後 `ls` 舊檔大小非 0，`dump` 兩檔頭尾的 `t_ms` 差 ≈ 拔卡時間。
另測「拔卡後不插回直接 stop」：應立刻返回 `FLUSH FAILED`，不卡住。拔卡測試已在板子上通過。

### 10.7 沒解決的與取捨

| 項目 | 說明 |
|---|---|
| **根因是供電，軟體只是止血** | 每次卡死仍會掉幾秒資料。正解是獨立 3.3V、SD 座旁加 10~100µF 電容、模組有 LDO 的接 5V |
| **斷線期間不取樣** | sensor 只在 `LOGGING` 時取樣，`DL_ERROR` 到恢復之間的樣本不存在。要保住得在 RAM 做環形緩衝，代價是記憶體 |
| **OLED 會凍住** | ② 最壞約 8s；③ 每次嘗試 `sd_init()` 的 ACMD41 最多 1.5s，都在 `spi1_lock` 內。卡健康時不發生。嫌久可把 `DISK_WRITE_RETRIES` 降到 1~2 |
| **`disk_read` 沒有重試** | `f_write` 配置新 cluster 時會讀 FAT sector，讀失敗一樣鎖死 `fp->err`（由 ③ 兜底）。補法是同一套迴圈 |
| **最後一行可能半行** | 救回的舊檔結尾可能是不完整的 CSV 行，解析端要容忍 |
| **極少數情況留下只有表頭的檔** | 恢復與 stop 正好撞在同一瞬間時（§10.4 併發表第二列），無害 |

> **面試可以講的點：** 這是「分層吸收失敗」的完整例子 —— 每層只處理自己看得懂的失敗，
> 處理不了才往上交，而且每一層的吸收都留下計數，不讓問題被靜默藏起來。最值得講的是
> ① 那個 bug：busy 的 `0x00` 跟 `R1 = 0x00` 在線上長得一模一樣，不先等 ready，重試
> 反而會穩定失敗 —— 加一層防護之前，要先確認它的前提成立。

---

## 11. 已知限制與取捨（誠實清單）

面試時主動講這些，比被問出來好。

| # | 限制 | 影響 | 修法 |
|---|---|---|---|
| 1 | **只做單 block（CMD17/CMD24）**，`disk_read/write` 用 for 迴圈逐 sector | 每個 sector 都付一次完整的命令 + 回應 + busy 開銷，連續大量讀寫吞吐量明顯偏低 | 加 CMD18/CMD25 多 block 傳輸 |
| 2 | **CRC 全關** | 命令與資料都無完整性保護。假 start token（`0xFE` 與 idle `0xFF` 只差 1 bit）會讓 512B 雜訊被當資料收下，且**靜默** | 發 CMD59 開 CRC + 真的驗那兩個 CRC16 byte |
| 3 | **讀取沒用 DMA** | 512 次 `transfer_byte` 迴圈佔 CPU，mount 階段最明顯 | full-duplex DMA（TX 餵 `0xFF`、RX 收 buffer） |
| 4 | **`disk_ioctl` 只有 `CTRL_SYNC`** | `GET_SECTOR_COUNT` / `GET_SECTOR_SIZE` / `GET_BLOCK_SIZE` 都回 `RES_PARERR` -> **`f_mkfs()` 無法使用**，不能在裝置上格式化 | 補那三個 case（sector count 需讀 CSD -> CMD9） |
| 5 | **busy 期間不放 `spi1_lock`** | OLED 可能凍住幾百 ms（§8）；卡死時 disk_write 重試最壞約 8s、每次自動恢復最多約 1.5s，期間 OLED 同樣停住（§10.7） | 刻意取捨，不建議改 |
| 6 | **沒有熱插拔偵測** | 抽卡後要靠讀寫失敗才發現；插回後由 §10 的自動恢復接回，但中間的樣本會遺失 | CD 腳或定期 CMD13 探測 |
| 7 | **沒讀 CSD/CID（CMD9/CMD10）** | 不知道卡容量、廠商、序號 | 開機報告想印容量的話需要補 |
| 8 | **`SD_BAUD_FAST` 保守設在 2.6MHz** | 低於 SPI 模式上限（25MHz） | 麵包板的保守值；線路乾淨可改 BR=3（5.25MHz）起跳測 |
| 9 | **data response token 單讀一個 byte** | 規格上它緊接資料封包，但若卡慢一個 byte，`0xFF & 0x1F != 0x05` 會誤判成寫入失敗（資料其實已寫入）。現在至少看得出來了：`status` 會顯示 `wr resp rejected (byte 0xFF)` | 加 8 次上限的掃 `0xFF` 迴圈，成本約 25us |
| 10 | **寫入 buffer 不能在 CCM**（§8） | task 的 local 陣列丟進 `f_write`（>=512B）會被 `dma_mem_ok()` 擋下回 `-1`。目前 `FIL`/`FATFS` 都是 static 所以安全，但這是呼叫端要記住的約束 | 已有執行期檢查且走正規錯誤路徑；要徹底解決得在 driver 內部加一個主 SRAM 的 bounce buffer（代價是多 512B 與一次 memcpy） |
| 11 | **`disk_read` 沒有重試** | 讀 FAT sector 偶發失敗一樣會鎖住 `fp->err`（§10.7） | 用 `disk_write` 同一套迴圈 |

### 文件待修

`device_sdcard.c` 檔頭與 `sd_init()` 第 1 步的註解寫 **250kHz**，但 `SD_BAUD_INIT`=7
在 PCLK2=84MHz 下是 `84MHz / 256 = 328kHz`（巨集旁的註解才是對的）。
兩者都 <=400kHz 所以功能沒問題，但數字要統一。

---

## 12. 可能被問的問題

> 按「答得出來的收益」排序。

### 基礎機制

1. SD 卡怎麼進入 SPI 模式的？ -> CMD0 期間 CS 拉低（§2）
2. init 為什麼要降速到 400kHz 以下？ -> 卡在未初始化狀態只保證這個速率
3. 開頭那 74 個 clock 幹什麼用的？為什麼 CS 要**拉高**？ -> power-up ramp，還不是在對話（§2）
4. CS 為什麼不用硬體 NSS？ -> 交易跨多 byte，硬體 NSS 會按 byte 抬放（§1）

### 協定細節

5. CRC 都關了，為什麼還要送 `0x01` 而不是 `0x00`？ -> bit0 是 **end bit**（§3）
6. 為什麼只有 CMD0 和 CMD8 要真 CRC？ -> 兩個理由完全不同（§3）
7. start token 為什麼是 `0xFE`？ -> 自我定界 + 0 落在 LSB（§4）
8. 怎麼知道卡寫完了？為什麼要自己送 `0xFF`？ -> SPI 無 master clock 則 slave 不吐 bit（§5）
9. SDHC 和 SDSC 的定址差別？搞錯會怎樣？ -> ×512 錯位、靜默毀損（§7）
10. CMD8 已經給電壓了，ACMD41 為什麼還給？ -> 它給的是 HCS；電壓欄位 SPI 模式忽略（§2）

### 設計與取捨（最能拉開差距的部分）

11. 等卡回應為什麼不直接 `vTaskDelay`？ -> 常見路徑付不起一個 tick（§6）
12. 那為什麼不純忙等？ -> 90.8% CPU 的實測（§6）
13. 為什麼寫用 DMA 讀不用？ -> TX-only DMA 收不到 MISO（§5）
14. timeout 值怎麼定的？ -> 1000ms 的來由與 FatFs `fp->err` 鎖死的代價（§6）
15. driver 怎麼同時支援 RTOS 與 bare-metal？ -> `bus_port.h` 間接層（§1）
16. 錯誤碼往上層走會被壓扁，你怎麼處理？ -> 在最底層留快照而非改上層介面（§9.5）
17. 為什麼 DMA 碰不到 CCM？你怎麼防？ -> CCM 只在 D-bus 不在 bus matrix；
    `dma_mem_ok()` 白名單檢查 + 回錯誤碼而非 assert（§8）
18. 這個 driver 你會先改哪裡？ -> §11 的 #1（多 block，吞吐量）或 #2（CRC，可靠性），
    視場景選一個並說出理由 —— **不要說「都要改」**

### 可能的延伸

19. 為什麼不用 SDIO 介面？ -> F407 有 SDIO，4-bit + 硬體 CRC + 高時鐘，吞吐量高一個量級；
    本專案選 SPI 是為了腳位彈性與 driver 可攜性，而 logger 的資料率（取樣級）遠低於瓶頸
20. 多個裝置共用 SPI1 怎麼處理？ -> 軟體 CS + `spi1_lock` 互斥（§8）

### 故障處理（§10）

21. 寫入失敗為什麼會毀掉整個 session？ -> FatFs ABORT 把錯誤鎖進 `fp->err`（§10.3）
22. 重試前為什麼一定要先等 ready？ -> busy 的 `0x00` 會被誤判成 `R1 = 0x00`（§10.4 ①）
23. 重寫同一個 sector 安全嗎？換成多 block 呢？ -> 單 block 冪等；CMD25 要先送 `0xFD`（§10.4 ②）
24. 恢復時為什麼用 `disk_initialize()` 而不是 `f_mount()`？ -> 保住 volume，`f_close` 才能把
    dirty sector 寫回去；`f_sync` 不看 `fp->err`（§10.4 ③）
25. 你怎麼驗證一條平常走不到的錯誤路徑？ -> 調短逾時做真實的故障注入 + 拿掉 ① 當對照組（§10.6）
