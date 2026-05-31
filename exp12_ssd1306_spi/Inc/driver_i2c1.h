/*
 * driver_i2c1.h
 *
 * I2C1 driver for STM32F407
 *   - PB6 = SCL (AF4)
 *   - PB9 = SDA (AF4)
 *   - Standard mode 100kHz @ APB1 16MHz
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
} i2c_status_t;


/* ========================================================================= */
/*                          Initialization                                   */
/* ========================================================================= */

/*
 * 初始化 I2C1
 *   - 開 GPIOB / I2C1 clock
 *   - PB6 SCL / PB9 SDA 設為 AF4 open-drain + pull-up
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
 *   I2C_OK       bus 釋放成功
 *   I2C_ERR_BUS  送完 9 clock 後 SDA 仍 LOW（slave 真的卡死）
 */
i2c_status_t i2c1_bus_recovery(void);

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
 *   int n = i2c1_scan(addrs, 8);
 *   for (int i = 0; i < n; i++) { ... addrs[i] ... }
 */
int i2c1_scan(uint8_t *found, int max);


#endif /* DRIVER_I2C1_H_ */