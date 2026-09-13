/*
 * driver_i2c1.h
 *
 * I2C1 driver for STM32F407
 *   - PB8 = SCL (AF4)   ← 從 PB6 改來，把 PB6 讓給 USART1_TX
 *   - PB9 = SDA (AF4)
 *   - Standard mode 100kHz @ APB1 42MHz（PLL 168MHz / 4）
 *
 * 所有 transaction API 有 timeout 保護 + 回傳 status code，
 * sensor 掉線時不會卡死。
 *
 * NOTE:
 *   STM32F4 的 I2C 硬體狀態機複雜（ST 後來在 F7/H7 重新設計），
 *   每個 flag 等待都可能因 bus 異常卡死，所以這份 driver 用
 *   SysTick 做 timeout，並提供 bus recovery。
 *
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#ifndef DRIVER_I2C1_H_
#define DRIVER_I2C1_H_

#include <stdint.h>

/* ========================================================================= */
/*                          Status codes                                     */
/* ========================================================================= */
typedef enum {
    I2C_OK = 0,
    I2C_ERR_TIMEOUT,        /* 等 flag 超時（線斷、bus stuck） */
    I2C_ERR_NACK,           /* slave 沒回 ACK（位址錯、晶片沒上電） */
    I2C_ERR_BUS,            /* bus error（arbitration lost、SDA stuck） */
    I2C_ERR_PARAM,          /* buf == NULL 或 len == 0 */
    I2C_ERR_SCL_STUCK,      /* SCL 被拉在 LOW —— clock stretch 卡死，搖 clock 救不了 */
} i2c_status_t;


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

/*
 * 初始化 I2C1
 *   - 開 GPIOB / I2C1 clock
 *   - PB8 SCL / PB9 SDA 設為 AF4 open-drain + pull-up
 *   - Standard mode 100kHz
 */
void i2c1_init(void);


/* ========================================================================= */
/*                          Transaction API                                  */
/* ========================================================================= */

/*
 * 寫單一 register
 *   流程：START → addr+W → reg → data → STOP
 */
i2c_status_t i2c1_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/*
 * 寫 16-bit register（MSB first）
 *   流程：START → addr+W → reg → data[15:8] → data[7:0] → STOP
 *
 * 給暫存器寬度是 16-bit 的晶片用（INA219 的 config / calibration）。
 * MPU6050 那種 8-bit 暫存器用上面的 i2c1_write_reg() 即可。
 */
i2c_status_t i2c1_write_reg16(uint8_t slave_addr, uint8_t reg_addr, uint16_t data);

/*
 * 讀多個連續 register（burst read）
 *   流程：START → addr+W → reg → RESTART → addr+R → read N → STOP
 *
 *   slave - 7-bit I2C address
 *   reg   - 起始 register
 *   buf   - 接收 buffer，至少 len bytes
 *   len   - 要讀的 byte 數（>= 1）
 */
i2c_status_t i2c1_read_regs(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len);


/* ========================================================================= */
/*                          Bus utility                                      */
/* ========================================================================= */

/*
 * I2C bus recovery
 *   當 SDA 被 slave 卡在 LOW（電源 glitch 後常見），
 *   手動送最多 9 個 clock pulse 強迫 slave 釋放 SDA，
 *   再產生一次 STOP condition，最後重新 init I2C。
 *
 * 何時呼叫：
 *   - 收到 I2C_ERR_TIMEOUT 後嘗試恢復
 *   - 系統啟動時保險呼叫一次
 *
 * 回傳：
 *   I2C_OK             bus 釋放成功
 *   I2C_ERR_BUS        送完 9 clock 後 SDA 仍 LOW（slave 真的卡死）
 *   I2C_ERR_SCL_STUCK  SCL 被拉在 LOW。這種情況搖 clock 完全無效（我們寫
 *                      ODR high，線卻還是 low，產生不了任何邊緣），slave 收
 *                      不到 clock 就永遠不會放開 SDA。指向硬體問題：slave
 *                      當機、SCL 短路到地、或 SCL 缺上拉。
 */
/* ── 中斷驅動版的 read_regs（非同步）────────────────────────────
 * 匯流排時序與 i2c1_read_regs() 完全相同，差別只在等待方式：polling 版在
 * while 迴圈裡空轉，這支發起交易後就阻塞（透過 bus_port.h 的
 * i2c1_xfer_wait()），由 I2C1 的 ISR 把整筆交易推完才喚醒一次。
 *
 * 給誰用：Sensor task 的 100Hz 熱路徑。MPU6050 的 14-byte burst 在 400kHz
 * 下約 400µs，polling 版那 400µs 全部算成 CPU 時間，而且 Sensor 是 prio 4
 * （全系統最高），空轉期間其他 task 全被鎖在外面 —— 這是本專案唯一能靠
 * 「改成睡覺」買到真並行的地方。
 *
 * 誰不該用：i2cscan / bus_recovery / 寫暫存器，以及 bare-metal 專案。
 * 那些沿用上面的 polling 版，行為完全沒變。
 *
 * 前提：呼叫端必須已持有 i2c1_lock()（與 polling 版相同的約定）。
 * 回傳值與 i2c1_read_regs() 相同；額外多一種 I2C_ERR_TIMEOUT 的來源：
 * 狀態機在 bus stuck 時沒推完，等待逾時後周邊會被收乾淨再回報。 */
i2c_status_t i2c1_read_regs_async(uint8_t slave, uint8_t reg, uint8_t *buf, uint8_t len);

/* 非同步交易是否進行中。給 bare-metal 版的 i2c1_xfer_wait() 盯著用
 * （退化成 busy-wait），一般應用不需要呼叫。 */
int i2c1_async_busy(void);

/* 非同步交易的累計失敗次數，依類型分開（任一指標可為 NULL）。
 * 三種失敗指向完全不同的方向：
 *   nack     slave 沒回應 —— 供電、接觸、位址
 *   bus      BERR/ARLO/OVR —— 電氣雜訊，或 ISR 被延遲太久
 *   timeout  狀態機沒被推動 —— 匯流排卡住
 * 理想全為 0；偶爾跳 nack 在麵包板上算正常，持續累積就要查硬體。 */
void i2c1_async_err_counts(uint32_t *nack, uint32_t *bus, uint32_t *timeout);

i2c_status_t i2c1_bus_recovery(void);


/* ========================================================================= */
/*                          Fault injection (test only)                      */
/* ========================================================================= */

/* 故障注入只在 Debug build 編進去（.cproject 的 Debug 組態會定義 DEBUG）。
 * 要強制關掉就在 build 加 -DI2C1_FAULT_INJECT=0。 */
#ifndef I2C1_FAULT_INJECT
#  ifdef DEBUG
#    define I2C1_FAULT_INJECT  1
#  else
#    define I2C1_FAULT_INJECT  0
#  endif
#endif

#if I2C1_FAULT_INJECT
/*
 * 故意把匯流排搞卡，用來驗證 i2c1_bus_recovery() 真的有效。
 *
 * 為什麼不能「MCU 自己把 SDA 拉低」來測：bus_recovery 一開頭就把 PB9 的
 * ODR 寫成 1，等於自己放開自己，迴圈第一次讀 IDR 就是 high，直接回
 * I2C_OK —— 假陽性，什麼都沒測到。必須讓真的 slave 去抓 SDA。
 *
 * 手法：bit-bang 送 START + (slave<<1|R) 的 8 個 bit，然後停在第 9 個
 * （ACK）clock 的 high phase。slave 此刻正拉低 SDA 表示 ACK，而規格上要
 * 到下一個 SCL 下降沿才能釋放 —— 我們不給那個下降沿，SDA 就卡住了，
 * 跟電源 glitch 造成的 stuck 在電氣上無法區分。
 *
 * 離開時 I2C1 是 disabled、腳位停在 GPIO mode、SCL 停在 high，
 * 由後續的 i2c1_bus_recovery()（或 Sensor task 的自動路徑）負責收拾。
 *
 * 前提：呼叫端必須持有 i2c1_lock()，且只能在 task 脈絡呼叫。
 *
 * 回傳：
 *   1  已卡住（IDR 讀到 SDA 仍是 low）
 *   0  注入失敗 —— slave 沒回 ACK（位址錯 / 沒供電）。此時匯流排沒被弄壞，
 *      但周邊仍是 disabled，呼叫端還是要跑一次 recovery 或 i2c1_init()。
 */
int i2c1_inject_sda_stuck(uint8_t slave);
#endif

/*
 * I2C bus scanner
 *   掃描 0x08 ~ 0x77 的 7-bit 位址，
 *   對每個位址送 START + addr+W，回 ACK 代表有裝置。
 *
 * 參數：
 *   found    - 呼叫端提供的 buffer，存找到的位址
 *   max      - found buffer 的大小
 *
 * 回傳：找到的裝置數量（<= max）
 *
 * 用法：
 *   uint8_t addrs[8];
 *   int n = i2c1_bus_scan(addrs, 8);
 *   for (int i = 0; i < n; i++) { ... addrs[i] ... }
 */
int i2c1_bus_scan(uint8_t *found, int max);


#endif /* DRIVER_I2C1_H_ */