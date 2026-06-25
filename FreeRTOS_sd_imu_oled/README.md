# FreeRTOS IMU 資料記錄器（exp13 的 RTOS 版）

> STM32F407VGTX · bare-metal register-level driver · **FreeRTOS-Kernel V11.1.0 (LTS)**
> 把 [`exp13_sd_imu_oled`](../exp13_sd_imu_oled)（co-operative super-loop）移植到 FreeRTOS，
> 從「一個 `while(1)` 輪流做事」變成 **五個事件驅動 task 並行**，展示資源競爭與同步機制。

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
 Sensor (prio4, 事件驅動+100Hz) ─imu_sample_t─> [ qSamples ] ─> Logger (prio3, 事件驅動)
   notify 喚醒 → vTaskDelayUntil 100Hz 取樣        佇列 64 筆      pop → 寫 CSV/SD (bus_lock)
   IDLE 睡死在 ulTaskNotifyTake（0% CPU）                           每 1s f_sync
 Display(prio1, 10Hz)    取狀態快照 → bus_lock → 畫 OLED → bus_unlock
 Console(prio2, 事件驅動) UART RX IDLE/DMA 中斷喚醒 → app_cli_poll()：start/stop/status/ls/dump/clear
 Button (prio2, 事件驅動) PA0 EXTI0 邊緣喚醒 → 去抖（20ms 重讀）→ toggle start/stop
 bootstrap(最高, 跑一次)  周邊+掛載初始化 → 建上面五個 task + 接 ISR 通知接線 → 自刪
```

| Task | 優先 | 觸發 | 職責 | 同步 |
|---|---|---|---|---|
| **Sensor** | 4（最高）| 事件驅動：notify 喚醒後 100Hz（`vTaskDelayUntil`）| 讀 MPU → 丟 Queue | Queue（生產者）+ task notify |
| **Logger** | 3 | 事件驅動（等 Queue）| Queue → 累積/寫 SD、定期 sync | Queue（消費者）+ bus mutex |
| **Console** | 2 | 事件驅動（UART RX IDLE/DMA notify）| UART CLI | bus mutex（經 `dl_*` 委派）|
| **Button** | 2 | 事件驅動（EXTI0 邊緣 notify）| 去抖 → toggle start/stop | 走 `app_datalogger` API |
| **Display** | 1（最低）| ~10Hz | 狀態快照 → 更新 OLED | bus mutex + state mutex |

> **全系統事件驅動**：除了 Display 固定 10Hz 刷畫面，其餘 task 平時都 block 在通知上、
> 0% CPU；UART/EXTI 的 ISR 與 `start()` 用 `...NotifyGive` 把對應 task 喚醒。狀態旗標
> （`DL_IDLE/LOGGING`）只剩「被查詢」職責，沒有任何 task 在輪詢它。

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
- port = `GCC/ARM_CM4F`（Cortex-M4F、hard float）、`heap_4`、PLL 168MHz、1ms tick。
  時脈在 `driver_rcc.c`（`rcc_clock_init_168mhz()`，main() 最前面、排程器之前呼叫）；
  來源以外部 **HSE 8MHz** 晶振為主，HSERDY timeout 自動 fallback 回 **HSI 16MHz**
  （永不卡死，兩條路都到 168MHz）。PCLK1=42MHz / PCLK2=84MHz，分頻常數已對齊。
- **SysTick 交接**：`SVC/PendSV/SysTick_Handler` 改由 port 經 `FreeRTOSConfig.h` alias 接管；
  `systick.c` 移除自己的 handler，`systick_get_ms()`→`xTaskGetTickCount()`、
  `delay_ms()`→`vTaskDelay()`，上層 driver 一行不用改。
- 所有初始化移進 task（排程器啟動後 `delay_ms` 才有效）。

**Phase 2 — 拆成四個 task**
- `app_datalogger` 拆「取樣 / 寫入」：刪 `tick()`，加 `write_sample()` / `sync()` / `get_status()`，
  FatFs 操作內建 `bus_lock`、狀態用 `mState`。
- 新增 `app_tasks.c`：Queue / mutex / 四個 task body + bootstrap。
- `main.c` 瘦成 `app_tasks_start()` + `vTaskStartScheduler()`。

**Phase 3 — 從輪詢到事件驅動（本版）**

把「定期醒來檢查」改成「有事才被喚醒」，平時 0% CPU。

- **Button 獨立成第 5 個 task + EXTI 外部中斷**（`app_button.c`）：PA0（Discovery B1，
  active-high）的 rising edge 走 **EXTI0** 中斷。ISR 只做兩件事 —— 清 pending +
  `vTaskNotifyGiveFromISR` 喚醒 button task。**去抖刻意不放 ISR**：邊緣只當「該去看一下」
  的觸發，真正去抖（等 ~20ms 接點沉澱後重讀電位，仍按著才採信）放在被喚醒的 task 脈絡，
  那裡才能安全 `vTaskDelay`、也能呼叫會走 FatFs 的 start/stop。平時 block，0% CPU。
- **Console 改 UART RX IDLE / DMA 喚醒**：不再每 10ms 輪詢 RX ring buffer。USART **IDLE
  line 中斷**（一筆傳輸結束）與 DMA RX 半滿/全滿中斷在 ISR 內 `vTaskNotifyGiveFromISR`
  喚醒 console，醒來一次把 buffer 抽乾、跑完所有完整指令。
- **Sensor 也改 notify 驅動**：平時 block 在 `ulTaskNotifyTake`（IDLE 0% CPU、0 次取樣）；
  `app_datalogger_start()` 成功從 IDLE→LOGGING 時 `xTaskNotifyGive` 喚醒它進 100Hz 內層
  迴圈；stop 翻狀態，內層迴圈下次檢查就退出、回頂端睡死。
- **ISR→task 通知接線**集中在 bootstrap，且**在 task 建立之後**才註冊 callback；註冊前若有
  中斷觸發，driver 內 callback 指標還是 NULL，只清旗標/pending，安全。承載 `...FromISR`
  的三個 IRQ（USART1 / DMA2_Stream2 / EXTI0）優先權都設 **6 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`(5)**，
  ISR 內才能合法呼叫 FreeRTOS 的 `...FromISR` API。
- **CLI 與 FatFs 解耦**：`app_cli.c` 不再 include `ff.h`、也不再自己持 `bus_lock`；所有卡片
  存取改走 `app_datalogger` 的 `dl_dump` / `dl_list` / `dl_clear`（callback 串流輸出），
  鎖與 FatFs 全收進 datalogger，CLI 只負責「讀哪個檔」與排版。
- **故障可視化**：`vApplicationMallocFailedHook` / `StackOverflowHook` / `configASSERT`
  從靜默死迴圈改成**先從 UART 印原因再卡住** —— heap 剩量（`freeNow`/`minEver`，幫你定
  `configTOTAL_HEAP_SIZE`）、爆 stack 的 task 名、assert 的檔案:行號。輸出只用 polling UART
  （poll TXE），不靠中斷/DMA/RTOS，所以排程死掉或關中斷後仍送得出去。

---

## 檔案地圖（相對 exp13 的差異）

| 檔案 | 說明 |
|---|---|
| `Src/app_tasks.c` · `Inc/app_tasks.h` | **新增**：五 task + Queue + `mBus` + `bus_lock()` + ISR→task 通知接線 |
| `Src/app_button.c` · `Inc/app_button.h` | **新增**：PA0 EXTI0 事件驅動按鈕 driver（去抖留給上層 task）|
| `Src/app_datalogger.c` | 拆取樣/寫入、加鎖、狀態快照（`mState`）；卡片存取 API `dl_dump/dl_list/dl_clear`；`start()` notify 喚醒 sensor |
| `Src/app_cli.c` | 解耦：不碰 `ff.h`/`bus_lock`，`ls/dump/clear` 委派 `dl_*`；`status` 讀快照、印 `overruns` |
| `Src/driver_usart1.c` | RX IDLE/DMA 中斷 → callback 喚醒 console（事件驅動）|
| `Src/main.c` | 建 task + 啟動排程器；panic hook（malloc/stack/assert）從 UART 印故障 |
| `Src/systick.c` | 時間基準改接 FreeRTOS tick |
| `Inc/FreeRTOSConfig.h` | **新增**：CM4F 設定、handler alias、heap 28KB、`configASSERT`→`vAssertCalled` |
| `Src/{tasks,queue,list,timers,port,heap_4}.c` 等 | **新增**：vendored FreeRTOS-Kernel V11.1.0 |

---

## 建置 / 執行

1. STM32CubeIDE 開專案 → Build（managed build 自動納入 `Src/` 下所有 `.c`，含 kernel）。
2. 燒錄，UART 接 **115200** baud（USART1 = PB6/PB7，接 CP2102），會看到：
   ```
   === IMU datalogger (FreeRTOS, 5 tasks) ===
   [OK]   sensors + FAT32 ready
   CLI ready: start | stop | status | dump [file] | ls | clear
   ```
3. CLI 打指令或按 **PA0（B1）** 都能 toggle start/stop；`status` 多一行 `overruns=`
   （觀察佇列是否掉拍，理想 0）。
4. 若 heap/stack 不夠，UART 會印 `[PANIC] ...` / `[ASSERT] file:line` 指出原因，不再靜默卡死。

---

## 後續（SPEC §8，未做）

MPU6050 raw → g / deg·s scaling、CSV 緩衝與 `f_sync` 策略微調、HardFault handler
（fault stack dump）、DMA error recovery。
