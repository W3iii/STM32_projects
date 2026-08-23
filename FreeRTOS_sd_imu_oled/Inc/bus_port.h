/*
 * bus_port.h
 *
 *  匯流排互斥的 porting 介面 —— RTOS 中立。
 *  ─────────────────────────────────────────────
 *  仿 FatFs ff_mutex_* 的風格：這裡只宣告「具名函式」，不 include 任何
 *  RTOS 標頭。實作由各專案自己的 porting 檔提供（link-time 綁定）：
 *    - FreeRTOS 版：bus_port_freertos.c，每條匯流排一支 mutex。
 *    - bare-metal 版：可寫成 no-op（單執行緒）或關中斷（若 ISR 也上匯流排）。
 *
 *  這樣底層 bus driver（SD / OLED / MPU6050 / INA219）與上層模組
 *  （app_datalogger 等）都只需 #include "bus_port.h" 並呼叫 xxx_lock()/
 *  xxx_unlock()，不必碰 semphr.h，即維持可移植（bare-metal 與 FreeRTOS 都能編）。
 *
 *  兩條匯流排各有獨立的鎖，名字直接寫明鎖的是誰 —— 兩者互不阻擋
 *  （Sensor 讀 I2C 時 Logger 照樣能寫 SD）：
 *
 *    SPI1  OLED(SSD1306) + SD/FatFs
 *    I2C1  MPU6050 + INA219
 */

#ifndef BUS_PORT_H_
#define BUS_PORT_H_

/* 建立底層同步原語（兩支鎖一起）。在使用任何 lock 之前呼叫一次
 * （通常在啟動排程器前）。回傳 0 成功、非 0 失敗。bare-metal no-op 版直接回 0。 */
int  bus_port_init(void);

/* SPI1 匯流排互斥鎖：包住「整個」SPI1 transaction（OLED 整次更新，或一個
 * FatFs 操作）。app_datalogger.c / Display task 共用。non-recursive，
 * 不可巢狀對自己再 lock。 */
void spi1_lock(void);
void spi1_unlock(void);

/* I2C1 匯流排互斥鎖：包住「整個」I2C1 transaction 序列。單筆 read_regs /
 * write_reg 固然要包，但「寫 reg 再讀回來」這種多筆組合更是要整段包起來，
 * 中間被別的 task 插進一次 START 就會讀到別人的資料。
 * Sensor task（MPU6050）/ CLI（i2cscan）/ 之後的 INA219 共用。non-recursive。 */
/* ── SPI1 DMA 傳輸的等待 / 喚醒 ──────────────────────────────
 * spi1_send_buffer() 的 DMA 傳輸從 busy-wait 換成阻塞：啟動 DMA 後 task
 * 睡覺，DMA 的 TC/TE 中斷來了才喚醒。
 *
 * 為什麼值得：OLED 一次更新是 1024 bytes，5.25MHz 下約 1.6ms，而一次
 * 睡眠/喚醒來回約 3~8µs，比例 300:1。這裡跟 I2C 那組的動機不同 ——
 * Display 是 prio 1（最低），它空轉並不會擋住任何人，所以省下來的是
 * 「流回 IDLE 去 WFI」的電流，不是吞吐量。真正不能忍的是它**握著
 * spi1_lock 空轉**：logger 要寫 SD 就得排在後面。
 *
 * 注意粒度：只有「整個 buffer」這個層級才值得睡。SPI 一個 byte 在
 * 5.25MHz 下只要 1.52µs，比切換成本還短，所以逐 byte 的
 * spi1_transfer_byte() 維持 busy-wait 是對的。
 *
 * spi1_xfer_reset()   啟動 DMA 之前呼叫，清掉上一筆殘留的完成通知。
 * spi1_xfer_wait()    阻塞直到 ISR 呼叫 signal；有 timeout。
 *                     回傳 0 = 正常被喚醒，非 0 = 逾時。
 * spi1_xfer_signal()  由 DMA2_Stream3 的 ISR 呼叫（中斷脈絡）。
 *
 * bare-metal 版：reset/signal 寫成空，wait 寫成 while (spi1_is_busy());
 * 行為就退回原本的忙等。 */
void spi1_xfer_reset(void);
int  spi1_xfer_wait(void);
void spi1_xfer_signal(void);

void i2c1_lock(void);
void i2c1_unlock(void);

/* ── I2C1 非同步交易的等待 / 喚醒 ──────────────────────────────
 * 中斷驅動的 i2c1_read_regs_async()（driver_i2c1.c）用這一對把「等匯流排」
 * 從 busy-wait 換成阻塞：發起交易後 task 睡覺，ISR 把整筆交易推完才喚醒一次。
 *
 * 為什麼值得：MPU6050 的 14-byte burst 在 400kHz 下約 400µs，而一次
 * 睡眠/喚醒來回的成本約 3~8µs，比例 50:1 以上。更重要的是 Sensor 是
 * prio 4（全系統最高），它 busy-wait 時其他 task 全部被鎖在外面動不了，
 * 改成睡覺才真的把 CPU 讓出去 —— 這是本專案唯一能買到「真並行」的地方。
 * （SPI/SD 那兩處改成睡覺只換到省電，因為它們不是最高優先權。）
 *
 * 反過來說，粒度太細就不划算：逐 byte 睡覺（3µs 的等待付 3~8µs 的代價）
 * 是淨虧，所以只有「整筆交易」這個層級才這樣做。
 *
 * i2c1_xfer_reset()   發起交易「之前」呼叫，清掉上一筆殘留的完成通知
 *                     （前一次逾時、ISR 事後才送達的情況），否則這次的
 *                     wait 會立刻假性返回。
 * i2c1_xfer_wait()    阻塞直到 ISR 呼叫 signal；有 timeout，避免狀態機
 *                     卡住時把 Sensor task 一起拖死。回傳 0 = 正常被喚醒，
 *                     非 0 = 逾時。
 * i2c1_xfer_signal()  由 I2C1 的 ISR 呼叫（跑在中斷脈絡）。
 *
 * bare-metal 版：reset/signal 寫成空，wait 寫成 while (i2c1_async_busy());
 * 行為就退回成原本的 busy-wait，driver 其餘部分不用改。 */
void i2c1_xfer_reset(void);
int  i2c1_xfer_wait(void);
void i2c1_xfer_signal(void);

#endif /* BUS_PORT_H_ */
