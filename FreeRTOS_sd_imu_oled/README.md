# FreeRTOS IMU 資料記錄器（exp13 的 RTOS 版）

> STM32F407VGTX · bare-metal register-level driver · **FreeRTOS-Kernel V11.1.0 (LTS)**
> 把 [`exp13_sd_imu_oled`](../exp13_sd_imu_oled)（co-operative super-loop）移植到 FreeRTOS，
> 從「一個 `while(1)` 輪流做事」變成 **五個事件驅動 task 並行**，展示資源競爭與同步機制。

硬體接線、SD driver、FatFs 整合與踩坑紀錄都和 exp13 相同 —— 那部分請看
[exp13 的 README](../exp13_sd_imu_oled/README.md)。本文只談 **RTOS 移植**。

---

## 專案一句話

同一台 IMU 記錄器（MPU6050 100Hz → CSV/SD、OLED 顯示、UART CLI），但底層換成 FreeRTOS：
取樣、寫卡、顯示、命令列各自獨立成 task，靠 **Queue + Mutex** 解耦與保護共用資源；
再往上疊了**功耗量測（INA219）**、**自建 CPU 佔用統計**、**中斷驅動的 I2C**、
一套「等待夠久就去睡」的匯流排等待策略，以及供電不穩時的**降級模式**。

---

## 為什麼值得做（這個專案在展示什麼）

- **bare-metal → RTOS 的真實移植**：不是從零開新專案，而是把一個能跑的系統搬上 RTOS，
  處理 SysTick 接管、時間基準、初始化時序等實際問題。
- **共用匯流排的資源競爭**：OLED 和 SD 卡共用 SPI1、MPU6050 和 INA219 共用 I2C1。
  單執行緒時靠「輪流」天然避開，多 task 並行後必須用 mutex 保護。
- **生產者/消費者解耦**：SD 寫入偶爾會 stall（卡內部 GC），用 Queue 把高頻取樣和慢速寫卡分開，
  取樣節奏不被寫卡拖垮。
- **優先權繼承**：低優先 Display 持有 SPI 鎖時擋住中優先 Logger，FreeRTOS mutex 的優先權繼承
  可緩解優先權反轉。
- **先量測再優化**：自己做一套比 kernel 內建更準的 CPU 統計，用它找出「16% 的 CPU 花在
  空轉等 I2C」，再對症下藥把它降到 0.8%。過程與數據見下方
  [可觀測性](#可觀測性自建-cpu-統計) 與 [I2C 非同步化](#i2c-非同步化從-162-降到-08)。
- **硬體不穩時還能跟你對話**：板子靠 ST-LINK USB 供電，餘裕本來就小，SD 寫入的湧浪電流會
  讓三顆周邊間歇性出錯。與其在初始化失敗時掛起，不如**降級**：CLI、OLED、`i2cscan`、`power`
  全部照常，只有記錄功能關掉，並把失敗分類計數印在 `status`。見
  [韌性一](#韌性一六個安靜地騙人的缺陷)、[韌性二](#韌性二供電邊界下的降級模式) 兩節。

---

## Task 架構

```
 Sensor (prio4, 事件驅動+100Hz) ─imu_sample_t─> [ qSamples ] ─> Logger (prio3, 事件驅動)
   notify 喚醒 → vTaskDelayUntil 100Hz 取樣        佇列 64 筆      pop → 寫 CSV/SD (spi1_lock)
   IDLE 睡死在 ulTaskNotifyTake（0% CPU）                           每 1s f_sync
 Display(prio1, 10Hz)    量 INA219 → 取狀態快照 → spi1_lock → 畫 OLED → 取樣 CPU 統計
 Console(prio2, 事件驅動) UART RX IDLE/DMA 中斷喚醒 → app_cli_poll()
 Button (prio2, 事件驅動) PA0 EXTI0 邊緣喚醒 → 去抖（20ms 重讀）→ toggle start/stop
 bootstrap(最高, 跑一次)  周邊+掛載初始化 → 建上面五個 task + 接 ISR 通知接線 → 自刪
```

| Task | 優先 | 觸發 | 職責 | 同步 |
|---|---|---|---|---|
| **Sensor** | 4（最高）| 事件驅動：notify 喚醒後 100Hz（`vTaskDelayUntil`）| 讀 MPU6050 → 丟 Queue | Queue（生產者）+ `i2c1_lock` |
| **Logger** | 3 | 事件驅動（等 Queue）| Queue → 寫 SD、每秒 sync | Queue（消費者）+ `spi1_lock` |
| **Console** | 2 | 事件驅動（UART RX IDLE/DMA notify）| UART CLI | 經 `dl_*` 委派、`i2c1_lock`（i2cscan）|
| **Button** | 2 | 事件驅動（EXTI0 邊緣 notify）| 去抖 → toggle start/stop | 走 `app_datalogger` API |
| **Display** | 1（最低）| ~10Hz | 量 INA219 → 更新 OLED → CPU 統計取樣 | `i2c1_lock` + `spi1_lock` + state mutex |

> **全系統事件驅動**：除了 Display 固定 10Hz，其餘 task 平時都 block 在通知上、0% CPU；
> UART/EXTI 的 ISR 與 `start()` 用 `...NotifyGive` 把對應 task 喚醒。IDLE task 會執行
> **WFI** 睡覺（`wfi` 指令可即時關掉當對照組）。

---

## 同步原語

| 原語 | 角色 | 解決什麼 |
|---|---|---|
| **`qSamples`** Queue | Sensor → Logger | 把 100Hz 取樣與慢速 SD 寫入解耦；卡 stall 時樣本進佇列緩衝（64 筆 ≈ 640ms）。佇列滿才計 `overrun`（理想 0）。 |
| **`spi1_lock`** Mutex | SPI1 匯流排 | OLED 與 SD/FatFs 共用。序列化「整個 transaction」（OLED 整次更新，或一個 FatFs 操作）。 |
| **`i2c1_lock`** Mutex | I2C1 匯流排 | MPU6050（Sensor）、INA219（Display）、`i2cscan`（Console）共用。包住整組交易，不只單筆。 |
| **`s_sI2c1Xfer`** Binary Semaphore | I2C 非同步交易 | ISR 把整筆交易推完後喚醒發起的 task。詳見下方 I2C 章節。 |
| **`s_sSpi1Xfer`** Binary Semaphore | SPI1 DMA 傳輸 | DMA2_Stream3 傳完（或報錯）時喚醒等待的 task，取代原本的忙等。與 I2C 那組同一個 wait/signal 模式。 |
| **state mutex** | 狀態快照 | 保護 `state/count/filename/fres`。避免讀到半更新的欄位。 |

### 為什麼拆成兩支 bus mutex

早期只有一支 `mBus`（SPI1）。加入 INA219 之後改成 **每條匯流排一支**（`bus_port.h`）：

- **兩者互不阻擋**：Sensor 在 I2C 上讀 IMU 時，Logger 照樣能寫 SD。共用一支鎖的話這兩件
  高頻工作會互相排隊，白白拉長延遲。
- **不會 ABBA 死鎖**：沒有任何路徑同時持有兩支鎖 —— Display 是「先 I2C 量完、解鎖，再上 SPI
  畫圖」，序列而非巢狀。鎖的取得順序因此無關緊要。

`bus_port.h` 是**中性的 porting 介面**：只宣告具名函式、不 include 任何 RTOS 標頭，實作
由各專案 link-time 綁定（FreeRTOS 版是 `bus_port_freertos.c`，bare-metal 版寫成 no-op）。
底層 driver 只認 `spi1_lock()` / `i2c1_lock()`，因此 exp13 那份 bare-metal 專案仍能編。

> 取捨：粗粒度鎖 —— `dump` 大檔期間 OLED 會短暫凍結。換來的是「絕對不會死鎖、容易論證
> 正確性」。也順便解決 FatFs 非執行緒安全（`FF_FS_REENTRANT=0`）：每個 FatFs 呼叫都在
> `spi1_lock` 內，等於把所有 FatFs 操作序列化。

---

## 可觀測性：自建 CPU 統計

`tasks` 指令顯示每支 task 的 State / Pri / Stack 與三種 CPU 佔用。**百分比不是 kernel
內建的 run-time stats 算的**，是 `app_cpu_stats.c` 另外記的一份平行帳。

kernel 那份有兩個治不好的毛病：

1. **ISR 時間被算到剛好被打斷的那支 task 頭上** —— FreeRTOS 只在 context switch 時結算，
   所以 DMA、UART、EXTI 這些中斷的成本是隱形的，而且亂噴。
2. **當下執行中那支 task 的未結算切片永遠漏掉** —— `uxTaskGetSystemState` 當場讀計時器當
   分母，但各 task 的 `ulRunTimeCounter` 停在上一次切換。**分母用「現在」、分子用「過去」**，
   結果 Σ 各 task% 湊不到 100%，而且呼叫它的 CLI 系統性低報自己。

自建版掛 `traceTASK_SWITCHED_IN/OUT` 與 ISR 進出點，把時間拆成三類，相加等於牆鐘時間：

| 桶 | 何時結算 |
|---|---|
| 各 task | `switched_out`：`(now - t_in)` 再扣掉期間跑掉的 ISR |
| `[ISR]` | 每次 ISR 離開時當場結算，同時記進「本切片的 ISR 時間」供上面扣除 |
| `[SCHED]` | `switched_in`：OUT→IN 的間隔，即 `vTaskSwitchContext` 本身 |

查詢時做**虛擬結算**：把當下這支 task 尚未結算的切片加到「複製出來的副本」上，不動累加表。
這正是 kernel 漏掉的那一步。實機驗證兩欄都加總到 99.5%（差的 0.5% 是九列各自整數除法的
截斷，不是漏帳）。

取樣器 10Hz 掛在既有的 Display 迴圈上（不新增 task、不動 heap），保留 6.4 秒環形歷史與
歷史峰值 —— 秒級平均會把 SD `f_sync` 那種上百毫秒的尖峰整個抹平，Peak% 才抓得到。

```
> tasks
Task         State Pri Stack CPU%  Peak% Avg%
IDLE         Rdy     0    95  93.3  95.7  92.8
sensor       Blk     4   170   0.8   0.9   0.5
logger       Blk     3   840   0.7  90.8   1.1
display      Blk     1   145   4.2   4.4   4.2
[ISR]        -       -     -   0.3   0.4   0.1
[SCHED]      -       -     -   0.2   0.2   0.1
CPU%=last 1s  Peak%=worst 100ms sample  Avg%=since boot
```

`tasks hist [task]` 印最近 6.4 秒的時間軸（每格 100ms 壓成一個字元），`tasks reset` 清峰值。

成本：flash +1112 B、BSS +2180 B（其中 1792 B 是歷史緩衝）。

ISR 計時掛勾走 `isr_port.h`，**仿 FreeRTOS 的 `traceXXX` 巨集**：預設展開成空，專案放一支
`isr_port_config.h` 才接上實作。所以 driver 層不 include 任何 RTOS 或統計標頭，bare-metal
專案什麼都不用做（`nm -u` 驗證過目的檔裡連一個待解析的 symbol 都沒有）。

---

## I2C 非同步化：從 16.2% 降到 0.8%

`tasks` 量到 sensor 佔 **16.2%**，換算每筆取樣 1.62ms —— 對照 driver 註解「100kHz 下讀
14 byte 約 1.5ms」，那 16% 幾乎全是 CPU 卡在 `while (!(I2C1->SR1 & flag))` 空轉。而 Sensor
是 prio 4（最高），它空轉期間 Logger/Display/Console **全部被鎖在外面**。

兩刀一起下：

**1. 100kHz → 400kHz fast mode** — 線上時間縮到 1/4。

**2. 新增中斷驅動的 `i2c1_read_regs_async()`** — 匯流排時序與 polling 版逐步對應，差別只在
等待方式：發起後 task 睡覺，ISR 把整筆交易推完才喚醒**一次**。

```
task   i2c1_xfer_wait() ─────────睡（約 370µs）──────────► 醒
ISR    SB ADDR BTF SB ADDR RXNE×11 BTF BTF     18 次，每次約 1.7µs
                                          └─ 只有最後一次 give 號誌
```

`s_a.state` 是「上次做到哪」的書籤，ISR 每次只推一格。**I2C 沒有用 DMA** —— 資料是 CPU 在
ISR 裡一個 byte 一個 byte 搬的，因為 I2C 每個 byte 後都可能需要決策（要不要 ACK、是不是
最後一個、要不要轉向），硬體會拉住 SCL 等軟體回應。

為什麼 I2C 適合逐 byte 中斷而 SPI 不適合：

| 匯流排 | 一個 byte | ISR 開銷 | 比例 | 中斷驅動？ |
|---|---|---|---|---|
| I2C @400kHz | 22.5 µs | 1.7 µs | 13 : 1 | ✅ |
| SPI @5.25MHz | 1.52 µs | 1.7 µs | 0.9 : 1 | ❌ 比傳輸還久 → 用 DMA |

**實測結果**：

```
             之前      現在      差
  sensor    16.2%  →   0.8%    −15.4
  [ISR]      0.0%  →   0.3%    +0.3     ← async 真的在跑的證據
  IDLE      78.4%  →  93.3%   +14.9
```

−15.4 = +14.9 + 0.3 + 捨去誤差 —— 省下來的每一點都有去處，不是「從 task 搬到 ISR」而已。
那 370µs 原本什麼都沒產出，只是在盯一個 bit。

**polling 版完整保留**：`i2cscan`、`bus_recovery`、寫暫存器、以及 bare-metal 的 exp13 全部
沿用原路徑，一個字都沒改。只有 MPU6050 的 14-byte burst 與 INA219 的 16-bit 讀取改用 async。

三種長度（1 / 2 / >=3）都實作了，收尾規則依 RM0090 §27.3.3 各不相同（`len==2` 要用 `POS`）。
有 host 端測試：把 I2C1 換成普通記憶體、寫一個假周邊照事件順序推 SR1，真的去呼叫
`I2C1_EV_IRQHandler()` 驗資料落點與控制位元時序。

---

## 等待策略：什麼時候該睡，什麼時候該轉

RTOS 化真正改變的不是「多了幾支 task」，而是**每一處等待都變成一個要回答的問題**：
這段時間要抱著 CPU 空轉，還是進 Blocked 讓別人跑？

判準只有一個 —— **等待長度 ÷ 切換成本**。一次 context switch 約 3~8µs；比值愈大愈該睡。
專案裡每一處等待都用同一把尺量過：

| 等待什麼 | 一次多久 | 比值 | 決定 | 位置 |
|---|---|---|---|---|
| SD 卡 busy（flash program/erase）| 1 ms ~ 數百 ms | 200:1 ~ 10⁵:1 | **睡** `vTaskDelay(1)` | `spi1_poll_yield()` |
| OLED 一次 framebuffer DMA（1024 B @5.25MHz）| 1.6 ms | 300:1 | **睡** 等 DMA ISR | `spi1_wait_done()` |
| SD 一個 block 的 data-out（512 B @2.6MHz）| 1.6 ms | 300:1 | **睡** 改走 TX-only DMA | `sd_write_block()` |
| I2C 一整筆交易（14 B @400kHz）| 370 µs | 70:1 | **睡** 等 I2C ISR | `i2c1_read_regs_async()` |
| I2C 單一 byte @400kHz | 22.5 µs | 13:1 † | 中斷驅動（ISR 內逐 byte 推）| `I2C1_EV_IRQHandler` |
| DMA 結束後等 SPI `BSY` 落下 | 1.52 µs | 0.2:1 | **轉**（睡比等還久）| `spi1_wait_done()` 尾段 |
| SPI 單一 byte @2.6MHz | 3.1 µs | 0.5:1 | **轉** | `spi1_transfer_byte()` |

† 這一列的分母是 **ISR 進出開銷 1.7µs**，不是 context switch —— 逐 byte 中斷不換 task。
其餘各列都是對 3~8µs 的切換成本。

兩個從實作中長出來的細節：

**先轉一下再睡。** 「等卡」的迴圈不會一開頭就睡 —— 先忙等 `SD_SPIN_BEFORE_YIELD`（64 次
≈ 200µs），之後才改成 `spi1_poll_yield()`。SD 讀取的 NAC 典型 100µs、寫入的 program time
通常 1~3ms：常見的讀取路徑一次 yield 都走不到，不多付任何延遲；只有真的撞上 wear-leveling
那種停幾百毫秒的情況才切到 1ms 粒度輪詢。反過來說，一開頭就睡會讓每次存取無條件多付一個
tick，mount 階段幾十次 sector 讀取加起來很有感。

**讓出的是 CPU，不是匯流排。** `spi1_poll_yield()` 睡著時仍持有 `spi1_lock` —— 這是刻意的：
SD 的一次 block write 是不可中斷的交易序列，中途讓別人動 SPI1 會直接毀掉它
（exp13 §2 那個 OVR 坑就是共用匯流排咬人的前例）。另外用 `vTaskDelay(1)` 而不是
`taskYIELD()`：後者只讓給同優先權以上的 task，而 logger 是 prio 3，讓完馬上被排回來，
對 Display(1) / Console(2) 一點幫助都沒有 —— 要真的把時間讓出去就得進 Blocked。

---

## 移植階段

**Phase 1 — 排程器 bring-up**
- Kernel 原始碼丟 `Src/`、headers 丟 `Inc/`：STM32CubeIDE managed build 自動收編，**零 `.cproject` 改動**。
- port = `GCC/ARM_CM4F`、`heap_4`、PLL 168MHz、1ms tick。時脈在 `driver_rcc.c`，
  以外部 **HSE 8MHz** 為主，HSERDY timeout 自動 fallback 回 **HSI 16MHz**（兩條路都到 168MHz）。
- **SysTick 交接**：`SVC/PendSV/SysTick_Handler` 改由 port 經 `FreeRTOSConfig.h` alias 接管；
  `systick_get_ms()`→`xTaskGetTickCount()`、`delay_ms()`→`vTaskDelay()`，上層 driver 一行不用改。

**Phase 2 — 拆成四個 task**
- `app_datalogger` 拆「取樣 / 寫入」，FatFs 操作內建鎖、狀態用 state mutex。
- 新增 `app_tasks.c`：Queue / mutex / task body + bootstrap。

**Phase 3 — 從輪詢到事件驅動**
- **Button 獨立成第 5 個 task + EXTI0**：ISR 只清 pending + notify；**去抖刻意不放 ISR**，
  放在被喚醒的 task 脈絡才能安全 `vTaskDelay`、也能呼叫會走 FatFs 的 start/stop。
- **Console 改 UART RX IDLE / DMA 喚醒**：不再每 10ms 輪詢 ring buffer。
- **Sensor 也改 notify 驅動**：IDLE 時 0% CPU、0 次取樣。
- 承載 `...FromISR` 的 IRQ 優先權都設 **6 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`(5)**。
- **CLI 與 FatFs 解耦**：`app_cli.c` 不再 include `ff.h`，卡片存取走 `dl_dump/dl_list/dl_clear`。
- **故障可視化**：malloc/stack/assert hook 從靜默死迴圈改成先從 UART 印原因（heap 剩量、
  爆 stack 的 task 名、assert 的檔案:行號），輸出只用 polling UART，關中斷後仍送得出去。

**Phase 4 — 功耗與匯流排擴充**
- INA219 掛上 I2C1，量 bus/shunt/current/power，併進 Display task（慢速、只給人看的資料，
  不值得為它多開 task）；`power` 指令讀快照，不自己碰 I2C。
- `bus_port` 從一支鎖拆成 `spi1` / `i2c1` 兩對。
- IDLE 用 **WFI** 睡覺，`wfi on|off` 可即時切換當對照組 —— 配 `power` 就是一組現成的省電量測。
- `i2cscan` 掃 0x08~0x77，掃不到就順手做一次 bus recovery。

**Phase 5 — 量測、優化與韌性**
- 自建 CPU 統計取代 kernel run-time stats（見上方章節）。
- I2C 400kHz + 中斷驅動讀取（見上方章節）。
- 六個 bug 修正（見下方）。

**Phase 6 — 把剩下的忙等收乾淨、供電邊界下的降級（本版）**
- **SPI1 DMA 等待改成睡眠**：`spi1_wait_done()` 不再忙等，DMA2_Stream3 的 ISR 喚醒等待的
  task；同時開了 `TEIE`，DMA 出錯也會喚醒（以前錯誤發生時 TC 永遠不來，只能白白耗完逾時）。
  這件事的前提是 Phase 5 已把該 stream 的 NVIC 優先權從 0 改成 6，ISR 才能合法呼叫
  `...FromISR()`。
- **SD 區塊寫入改走 DMA**：data-out 期間卡不在 MISO 上說話，正好符合 TX-only DMA 的前提。
- **「等卡」迴圈改成先轉後睡**（`spi1_poll_yield()`）—— 上面那張表的第一列。
- **供電邊界下的可診斷性與降級模式**（見下方韌性第二節）。
- `power` 改印最近 10 筆（1 秒）平均，單筆離群值不再誤導 wfi on/off 這種對照量測。

---

## 韌性一：六個「安靜地騙人」的缺陷

實際踩到一次「logger 卡住」：`status` 說 LOGGING、OLED 有畫面、`tasks` 的 CPU 分布完全
正常，但計數再也不動、一筆都沒寫進卡片。根因是供電不穩讓 SD 寫入逾時，但**系統對它的
反應是安靜地騙人** —— 那才是要修的東西。

| # | 缺陷 | 修法 |
|---|---|---|
| 1 | **datalogger 寫入失敗完全靜默**。FatFs 一次失敗就把錯誤鎖進 `fp->err`，之後每次 `f_write` 都在函式開頭直接返回、連卡都不碰（所以 logger 只用 0.7% CPU 卻什麼都沒寫）。原碼只是「成功才 `count++`」。 | 新增 `DL_ERROR` 狀態，第一次失敗就翻過去；`status` 多 `write err` + FRESULT，OLED 顯示 `!ERR`。`f_sync` 的回傳值也一併接住。 |
| 2 | `SD_WRITE_TIMEOUT_MS` 500ms 偏緊（失敗當下 logger Peak% 從 2.5% 跳到 90.8%，就是那個忙等）| → 1000ms。卡片內部 erase 停幾百毫秒是正常行為，而逾時的代價是整個 session 作廢。 |
| 3 | `spi1_wait_done()` 兩個**沒有上限**的忙等，而它是在握著 `spi1_lock` 時呼叫的 —— 一旦不返回，display 永久空轉抱著鎖，logger 跟著死，外觀上系統還活著 | 加 100ms 上限，逾時關掉 DMA stream、放掉 busy 旗標 |
| 4 | `mpu6050_read_all()` 是 `void`，丟掉 I2C 錯誤 → 匯流排壞掉時把上一次的殘值當新樣本寫進 CSV | 改回傳狀態；Sensor 該筆不進佇列，連續 5 次失敗才做 `i2c1_bus_recovery()` + 重新 init（recovery 是幾十微秒的 bit-bang，只能在 task 脈絡做）|
| 5 | I2C ER handler 沒清 `OVR`（`ITERREN` 同時致能 BERR/ARLO/AF/OVR）| 補上 |
| 6 | DMA2_Stream3 / Stream7 的 NVIC 優先權是 **0**，能打斷 `taskENTER_CRITICAL()` 與其他 prio-6 ISR → CPU 統計的 `s_acc[ISR] += d` 會 lost update | → 6 |

另外修好一個 async 交易的起手式問題：原本「先開中斷、再發 START」，而 SR1 的錯誤位元是
**準位觸發**的 —— 只要殘留 `AF`/`BERR`，`ITERREN` 一開，ER 中斷就在 START 之前當場觸發，
交易「還沒開始就結束」，而沒人收拾的 START 留在匯流排上讓下一筆更容易出錯，**自我延續**。
改成發起前先清旗標，並把 START 移到開中斷之前（`SB` 同樣是準位觸發，不會漏事件）。

---

## 韌性二：供電邊界下的降級模式

板子目前靠 ST-LINK USB 供電，板載 3.3V LDO 的餘裕本來就小（MCU @168MHz 就吃掉一半以上），
而 SD 卡寫入瞬間電流可達 100~200mA。實測連續出現三個**彼此獨立**的間歇性故障：

- SD 寫入逾時 → logger 假死
- INA219 偶爾讀不到 → 最後開機就完全偵測不到
- `f_mount` 偶爾失敗，`FRESULT=3 (FR_NOT_READY)`

三顆周邊、兩條匯流排、全部間歇 → 共同上游是電。**這一節不修根因**（根因是硬體：SD 獨立
供電 + 就近加去耦電容），而是讓系統在邊界狀態下**可診斷、可存活**。

| # | 做法 | 為什麼 |
|---|---|---|
| 1 | **I2C 失敗依類型分開計數**，印在 `status`：`nack` / `bus` / `timeout` | 三者指向完全不同的方向，混成一個數字就沒辦法對症下藥。`nack`＝slave 沒回應（供電、接觸、位址）；`bus`＝BERR/ARLO/OVR（電氣雜訊，或 ISR 被延遲太久 —— 400kHz 下每 22.5µs 就來一個 byte）；`timeout`＝狀態機沒被推動。計數放在 driver 層，上層的重試不會把真實失敗率藏起來。 |
| 2 | **INA219 讀取重試一次** | `ina219_read()` 是兩筆交易，任一筆失敗整個讀取就作廢，一次 glitch 就讓那一幀顯示 `--`。麵包板上偶爾出錯是常態，一次重試蓋掉絕大多數；失敗率仍看得到（第 1 項）。代價：失敗時多約 500µs，10Hz 下可忽略。 |
| 3 | **async 交易發 START 前先等 `BUSY`** | RM0090 要求 START 只能在 BUSY=0 時發起。`ina219_read()` 兩筆交易背靠背，第一筆的 STOP 在線上還要約一個 SCL 週期才成形，而 task 被喚醒後可能更快就發下一個 START。只有 INA219 會踩到（MPU6050 一次 lock 只做一筆交易）。上限抓短（數百 µs），真卡住由既有的 50ms 逾時接手。 |
| 4 | **SD 掛載重試 3 次**，間隔 200ms，且**重試過就顯示出來** | `FRESULT=3` 代表 CMD0/ACMD41 沒走完，SD 在初始化期間對供電最敏感；因為是「偶爾」而非確定性失敗，重試很有效（技術上可行是因為 `disk_initialize()` 沒有快取狀態，每次都真的重跑 `sd_init()` 全套）。但「一次就過」才是健康的，所以 `status` 會講：`mounted after 1 retry (power margin?)` —— 這是早期警訊，不是「修好了」。 |
| 5 | **初始化失敗不再 `vTaskSuspend`**（最重要的一項）| 原本 MPU6050 或 `f_mount` 失敗就把 bootstrap 掛起，而**建立工作 task 是在那之後** —— 結果「SD 掛不起來 → console task 從沒被建立 → 連 CLI 都沒有」，正好在最需要 `i2cscan` / `status` 診斷的時候工具沒了。改成照樣把所有 task 建起來，只有 `app_datalogger_start()` 會因未掛載而失敗，`status` 第一行就講清楚：`SD  NOT MOUNTED (logging disabled, FRESULT 3)`。只有 heap 不夠建不出 state mutex 時仍然 suspend —— 那個沒得降級。 |

> 硬體不穩時最需要的是一台失敗了還能跟你對話的機器。

`status` 現在是一張完整的健康表，理想狀況下所有計數都是 0：

```
> status
  state     LOGGING
  file      log_003.csv
  samples   12480
  write err 0
  overruns  0
  i2c recov 0
  i2c err   nack 0  bus 0  timeout 0
  chip      41.2 C  (VDDA 2.98 V)
```

---

## 檔案地圖（相對 exp13 的差異）

| 檔案 | 說明 |
|---|---|
| `Src/app_tasks.c` · `Inc/app_tasks.h` | **新增**：五 task + Queue + bootstrap + ISR→task 通知接線 |
| `Src/app_button.c` · `Inc/app_button.h` | **新增**：PA0 EXTI0 事件驅動按鈕（去抖留給上層 task）|
| `Inc/bus_port.h` · `Src/bus_port_freertos.c` | **新增**：RTOS 中立的匯流排 porting 介面 —— `spi1_lock`/`i2c1_lock`、SPI/I2C 非同步交易的 wait/signal，以及等慢速裝置時讓出 CPU 的 `spi1_poll_yield()` |
| `Inc/isr_port.h` · `Inc/isr_port_config.h` | **新增**：ISR 計時掛勾，預設展開成空（仿 `traceXXX`），bare-metal 零成本 |
| `Src/app_cpu_stats.c` · `Inc/app_cpu_stats.h` | **新增**：自建 CPU 佔用統計（task / `[ISR]` / `[SCHED]` 三桶 + 10Hz 取樣器 + 環形歷史）|
| `Src/app_runtime_stats.c` | **新增**：TIM2 @100kHz 自由計數器，兩份統計共用的計時源 |
| `Src/device_ina219.c` · `Inc/device_ina219.h` | **新增**：INA219 功耗量測（讀取失敗重試一次）|
| `Src/device_tempsensor.c` | **新增**：晶片內建溫度感測（ADC，不走 I2C）|
| `Src/driver_i2c1.c` | polling API 原封不動；**新增**中斷驅動的 `i2c1_read_regs_async()` + 兩支 ISR；400kHz fast mode |
| `Src/app_datalogger.c` | 拆取樣/寫入、加鎖、狀態快照；`DL_ERROR` 與寫入錯誤回報；`dl_dump/dl_list/dl_clear` |
| `Src/app_cli.c` | `tasks` / `power` / `wfi` / `i2cscan` 指令；`status` 加 `write err`、`i2c recov`、`i2c err` 分類與掛載狀態；`power` 改印 10 筆平均 |
| `Src/driver_spi1.c` | DMA 等待從忙等改成睡眠（ISR 喚醒）+ 100ms 上限；開啟 `TEIE` 讓傳輸錯誤也喚醒；NVIC 優先權修正 |
| `Src/device_sdcard.c` | block 寫入的 512 byte data-out 改走 TX-only DMA；「等 token / 等卡 busy」兩個迴圈改成先轉 64 次再 `spi1_poll_yield()`；掛載期由上層重試 |
| `Src/driver_usart1.c` | RX IDLE/DMA 中斷 → callback 喚醒 console |
| `Inc/FreeRTOSConfig.h` | CM4F 設定、handler alias、heap 28KB、run-time stats 計時源、CPU 統計的 trace hook |
| `Src/{tasks,queue,list,timers,port,heap_4}.c` 等 | **新增**：vendored FreeRTOS-Kernel V11.1.0 |

---

## 建置 / 執行

1. STM32CubeIDE 開專案 → Build（managed build 自動納入 `Src/` 下所有 `.c`，含 kernel）。
2. 燒錄，UART 接 **115200** baud（USART1 = PB6/PB7，接 CP2102）。

```
commands:
  start          begin logging (new log_NNN.csv)
  stop           stop logging
  status         logger state + chip temp
  dump [file]    print a csv (default: last session)
  ls             list files + free space
  clear          delete all logs
  tasks          FreeRTOS task monitor (CPU% / peak / stack)
  tasks hist [t] CPU% timeline, last 6.4s (t = task name)
  tasks reset    clear peaks + timeline
  power          INA219 V / I / P (avg of last 10 samples)
  wfi [on|off]   idle WFI sleep (power saving)
  i2cscan        scan I2C1 bus (0x08..0x77)
  help           this list
```

3. CLI 指令或按 **PA0（B1）** 都能 toggle start/stop。
4. `status` 的健康指標理想全為 0：`overruns`（佇列掉拍）、`write err`（SD 寫入失敗次數
   + FatFs FRESULT）、`i2c recov`（I2C 匯流排卡住並自動復原的次數）、`i2c err` 的
   `nack`/`bus`/`timeout`。開頭若出現 `SD NOT MOUNTED` 或 `mounted after N retry`，
   先查供電（見上方韌性第二節）。
   `power` 印的是最近 10 筆（1 秒）平均並附樣本數 —— 開機第一秒內看到 `3 samples`
   就知道這個數字先別當真；要看瞬時值看 OLED 第 3 行。
5. 若 heap/stack 不夠，UART 會印 `[PANIC] ...` / `[ASSERT] file:line`，不再靜默卡死。

---

## 已知限制 / 後續

**量測精度**
- `tasks hist` 的時間軸刻度會漂：取樣掛在 Display 上，而 Display 用 `vTaskDelay`（非
  `vTaskDelayUntil`）且是最低優先權，錄製時會被 `spi1_lock` 擋住。**百分比本身正確**
  （分母是實測 TIM2 差值），只有「往左數幾格 = 幾秒前」不準。
- TIM2 @100kHz 一格 10µs，而單次 ISR 才 1~5µs。相位隨機所以長期平均會收斂，但**單桶值
  與 `[ISR]` 的 Peak% 不可信**。要看清得換 DWT→CYCCNT（5.95ns）。
- ISR 巢狀只認最外進、最內出（用旗標而非深度計數，為了容忍 `portYIELD_FROM_ISR` 內含的
  多餘 `traceISR_EXIT`）。
- `elapsed` 是 `uint32`，開機超過 11.9 小時後 Avg% 失效。
- CPU 統計 slot 用盡（>12 task）時該 task 的時間會靜默消失。

**Phase 6 尚未上板驗證**
- SPI 睡眠等待、SD DMA 寫入、`spi1_poll_yield()`、`power` 平均 —— 四項都還沒在板子上跑過。
  下面這些數字是**算出來的，不是量到的**：display 預期從 4.2% 降到約 2.7%（省下來的流回
  IDLE 去 WFI，所以是省電不是省吞吐 —— Display 是 prio 1，它空轉本來就擋不住誰；真正的
  收益是不再抱著 `spi1_lock` 空轉）；logger 的 Peak% 預期從 90.8% 掉到個位數。
  上板後要重跑 `tasks` 對帳。

**還沒做的優化**
- **SD 讀取仍是逐 byte polling**（`sd_read_block` 的 512 byte 迴圈）。寫入能用 TX-only DMA
  是因為 data-out 期間不必收 MISO；讀取要收，得另外開 RX DMA 或全雙工雙 stream，工程量
  不是同一個等級。`dump` 大檔時感受最明顯。
- SD 多 block 讀寫（CMD18/25）未做，每個 block 都要重付一次命令 + 等 token 的開銷。
- `spi1_poll_yield()` 固定睡 1ms，沒有 backoff。對 SD 夠用，但它是 `bus_port` 的公開介面，
  之後若有等待尺度差很多的裝置要用，語意會不夠。
- **I2C 只有讀做了 async**，`i2c1_write_reg` / `write_reg16` 仍是 polling（都是一次性的
  初始化，不在熱路徑）。
- `configGENERATE_RUN_TIME_STATS` 仍為 1，kernel 每次 context switch 白算一份沒人讀的
  數字。不能直接關：`portCONFIGURE_TIMER_FOR_RUN_TIME_STATS` 是 TIM2 的唯一初始化點，
  要先把它移到 app 層自己呼叫。
- I2C 的 `len==1` 路徑真機未驗證（專案裡用到單 byte 讀的只有 WHO_AM_I，仍走 polling）。
- `ISR_PORT_*` 這個命名看起來像 FreeRTOS 的 port 層 API，容易誤會。

**架構**
- 單 master 假設：`ARLO` 有清旗標但沒有重新競爭的邏輯。
- CPU 統計的 `s_a` / `s_acc` 不上鎖，依賴兩個不變式：呼叫端持有對應的 bus lock、
  所有被 instrument 的 ISR 優先權 ≥ 6。**第二條靠約定維持，編譯器不會警告**
  （上面第 6 個 bug 就是它被違反）。
- HardFault handler（fault stack dump）未做。

**硬體**
- 供電根因未解：SD 獨立供電 + 就近去耦電容還沒做。韌性第二節做的是「可診斷、可存活」，
  不是把問題修掉 —— `mounted after N retry` 與 `i2c err` 非 0 都還是會出現。
