# FreeRTOS IMU 資料記錄器（exp13 的 RTOS 版）

> STM32F407VGTX · bare-metal register-level driver · **FreeRTOS-Kernel V11.1.0 (LTS)**
> 把 [`exp13_sd_imu_oled`](../exp13_sd_imu_oled)（co-operative super-loop）移植到 FreeRTOS，
> 從「一個 `while(1)` 輪流做事」變成 **四個 task 並行**，展示資源競爭與同步機制。

硬體接線、SD driver、FatFs 整合與踩坑紀錄都和 exp13 相同 —— 那部分請看
[exp13 的 README](../exp13_sd_imu_oled/README.md)。本文只談 **RTOS 移植**。

---

## 專案一句話

同一台 IMU 記錄器（MPU6050 100Hz → CSV/SD、OLED 顯示、UART CLI），但底層換成 FreeRTOS：
取樣、寫卡、顯示、命令列各自獨立成 task，靠 **Queue + Mutex** 解耦與保護共用資源。

---

## 為什麼值得做（這個專案在展示什麼）

- **bare-metal → RTOS 的真實移植**：不是從零開新專案，而是把一個能跑的系統搬上 RTOS，
  處理 SysTick 接管、時間基準、初始化時序等實際問題。
- **共用 SPI1 的資源競爭**：OLED 和 SD 卡共用同一條 SPI1。單執行緒時靠「輪流」天然避開，
  多 task 並行後必須用 mutex 保護 —— 這是整個系統最核心的同步設計。
- **生產者/消費者解耦**：SD 寫入偶爾會 stall（卡內部 GC），用 Queue 把高頻取樣和慢速寫卡分開，
  取樣節奏不被寫卡拖垮。
- **優先權繼承**：低優先 Display 持有 SPI 鎖時擋住中優先 Logger，FreeRTOS mutex 的優先權繼承
  可緩解優先權反轉。

---

## Task 架構（SPEC §7）

```
 Sensor (prio4, 100Hz) ──imu_sample_t──> [ qSamples ] ──> Logger (prio3, 事件驅動)
   I2C 讀 MPU6050                          佇列 64 筆        pop → 寫 CSV/SD (bus_lock)
   只在 LOGGING 取樣                                          每 1s f_sync
 Display(prio1, 10Hz)   取狀態快照 → bus_lock → 畫 OLED → bus_unlock
 Console(prio2, ~10ms)  UART CLI + PA0 按鈕 → start/stop/ls/dump/clear
 bootstrap(最高, 跑一次) 周邊+掛載初始化 → 建上面四個 task → 自刪
```

| Task | 優先 | 週期 | 職責 | 同步 |
|---|---|---|---|---|
| **Sensor** | 4（最高）| 100Hz 固定（`vTaskDelayUntil`）| 讀 MPU → 丟 Queue | Queue（生產者）|
| **Logger** | 3 | 事件驅動（等 Queue）| Queue → 累積/寫 SD、定期 sync | Queue（消費者）+ bus mutex |
| **Console** | 2 | ~10ms 輪詢 | UART CLI、按鈕去抖 | bus mutex（FatFs 指令）|
| **Display** | 1（最低）| ~10Hz | 狀態快照 → 更新 OLED | bus mutex + state mutex |

---

## 三個同步原語（面試重點）

| 原語 | 角色 | 解決什麼 |
|---|---|---|
| **`qSamples`** Queue | Sensor → Logger | 把 100Hz 取樣與慢速 SD 寫入解耦；卡 stall 時樣本進佇列緩衝（64 筆 ≈ 640ms），Sensor 不掉拍。佇列滿才計 `overrun`（理想為 0）。 |
| **`mBus`** Mutex | SPI1 匯流排 | OLED 與 SD/FatFs 共用 SPI1。一支鎖序列化「整個 transaction」（OLED 整次更新，或一個 FatFs 操作）。 |
| **`mState`** Mutex | 狀態快照 | 保護 `state/count/filename`。Logger/start/stop 寫、Display/Console 讀，避免讀到半更新的欄位。 |

### 為什麼是「一支」bus mutex？

FatFs → SD → SPI1 是同一條路，OLED 也掛在 SPI1 上。**一支 `mBus` 同時解決兩件事**：

1. **匯流排爭用**：SD 寫入和 OLED 更新不會在 SPI1 上撞車（CS、資料交錯）。
2. **FatFs 非執行緒安全**（`FF_FS_REENTRANT=0`）：每個 FatFs 呼叫都包在 `mBus` 內，
   等於把所有 FatFs 操作序列化，Logger 寫檔和 Console `ls/dump` 不會同時動 volume。

全系統只有這一支鎖會「持有時再要鎖」，**鎖的取得順序唯一 → 不可能死鎖**。
（`mState` 只被 writer 短暫巢狀在 `mBus` 內，讀者只取 `mState`，順序固定 `mBus → mState`。）

> 取捨：這是**粗粒度**鎖 —— `dump` 大檔期間 OLED 會短暫凍結。換來的是「絕對不會死鎖、
> 容易論證正確性」。更細粒度的作法是開 `FF_FS_REENTRANT` + diskio/ssd1306 各自的
> per-transaction 鎖，但設定更多、debug 風險更高，這個系統不需要。

---

## 移植兩階段

**Phase 1 — 排程器 bring-up（先確認 RTOS 能跑、功能不退化）**
- Kernel 原始碼丟 `Src/`、headers 丟 `Inc/`：STM32CubeIDE managed build 自動收編，**零 `.cproject` 改動**。
- port = `GCC/ARM_CM4F`（Cortex-M4F、hard float）、`heap_4`、16MHz HSI、1ms tick。
- **SysTick 交接**：`SVC/PendSV/SysTick_Handler` 改由 port 經 `FreeRTOSConfig.h` alias 接管；
  `systick.c` 移除自己的 handler，`systick_get_ms()`→`xTaskGetTickCount()`、
  `delay_ms()`→`vTaskDelay()`，上層 driver 一行不用改。
- 所有初始化移進 task（排程器啟動後 `delay_ms` 才有效）。

**Phase 2 — 拆成四個 task（本版）**
- `app_datalogger` 拆「取樣 / 寫入」：刪 `tick()`，加 `write_sample()` / `sync()` / `get_status()`，
  FatFs 操作內建 `bus_lock`、狀態用 `mState`。
- 新增 `app_tasks.c`：Queue / mutex / 四個 task body + bootstrap。
- `main.c` 瘦成 `app_tasks_start()` + `vTaskStartScheduler()`。

---

## 檔案地圖（相對 exp13 的差異）

| 檔案 | 說明 |
|---|---|
| `Src/app_tasks.c` · `Inc/app_tasks.h` | **新增**：四 task + Queue + `mBus` + `bus_lock()` |
| `Src/app_datalogger.c` | 拆取樣/寫入、加鎖、狀態快照（`mState`）|
| `Src/app_cli.c` | `ls/dump/clear` 包 `bus_lock`；`status` 讀快照、印 `overruns` |
| `Src/main.c` | 瘦身成建 task + 啟動排程器 |
| `Src/systick.c` | 時間基準改接 FreeRTOS tick |
| `Inc/FreeRTOSConfig.h` | **新增**：CM4F 設定、handler alias、heap 20KB |
| `Src/{tasks,queue,list,timers,port,heap_4}.c` 等 | **新增**：vendored FreeRTOS-Kernel V11.1.0 |

---

## 建置 / 執行

1. STM32CubeIDE 開專案 → Build（managed build 自動納入 `Src/` 下所有 `.c`，含 kernel）。
2. 燒錄，UART 接 1200 baud，會看到：
   ```
   === IMU datalogger (FreeRTOS, 4 tasks) ===
   [OK]   sensors + FAT32 ready
   CLI ready: start | stop | status | dump [file] | ls | clear
   ```
3. CLI / 按鈕操作同 exp13；`status` 多一行 `overruns=`（觀察佇列是否掉拍，理想 0）。

---

## 後續（SPEC §8，未做）

MPU6050 raw → g / deg·s scaling、CSV 緩衝與 `f_sync` 策略微調、HardFault handler
（fault stack dump）、DMA error recovery。
