/*
 * mpu6050.c
 *
 *  MPU6050 driver implementation
 *  ----------------------------------------------------------
 *  Register map (RM 提到的常用幾個):
 *    0x1A  CONFIG          DLPF 設定
 *    0x1B  GYRO_CONFIG     full scale: ±250 / 500 / 1000 / 2000 °/s
 *    0x1C  ACCEL_CONFIG    full scale: ±2g / 4g / 8g / 16g
 *    0x3B  ACCEL_XOUT_H    加速度 6 bytes (X, Y, Z, MSB first)
 *    0x43  GYRO_XOUT_H     角速度 6 bytes
 *    0x6B  PWR_MGMT_1      bit6 = SLEEP (重置後 = 1，要清 0 叫醒)
 *    0x75  WHO_AM_I        固定回 0x68
 *
 *  Default sensitivity（重置後 ±2g / ±250°/s）:
 *    1 g    = 16384 LSB
 *    1 °/s  = 131   LSB
 * ----------------------------------------------------------
 * 
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#include "device_mpu6050.h"
#include "driver_i2c1.h"

/* MPU6050_ADDR 定義在 device_mpu6050.h（故障注入的 CLI 也要用）。 */

/* ---- Register address ---- */
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_WHO_AM_I        0x75

/* ---- 期望的 WHO_AM_I 值 ---- */
#define MPU6050_WHO_AM_I_VAL  0x68

/* ============================================================
 *                          內部 helper
 * ============================================================ */

/**
 * 把 6 bytes (XH, XL, YH, YL, ZH, ZL) 組合成三個 int16
 */
static void parse_axes(const uint8_t *buf, mpu6050_axes_t *axes)
{
    axes->x = (int16_t)((buf[0] << 8) | buf[1]);
    axes->y = (int16_t)((buf[2] << 8) | buf[3]);
    axes->z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ============================================================
 *                          Public API
 * ============================================================ */

uint8_t mpu6050_who_am_i(void)
{
    uint8_t id;
    i2c1_read_regs(MPU6050_ADDR, REG_WHO_AM_I, &id, 1);
    return id;
}

int mpu6050_init(void)
{
    uint8_t id;

    /* Step 1: 讀 WHO_AM_I。
     *
     * 以前收到 TIMEOUT 會在這裡自己呼叫 i2c1_bus_recovery() —— 讓一顆晶片
     * 的 driver 去決定整條匯流排要不要重置，INA219 那邊卻沒有，兩套邏輯
     * 各管各的。現在只回報錯誤種類，動不動匯流排由 app_i2c1_supervisor 統一決定。 */
    i2c_status_t st = i2c1_read_regs(MPU6050_ADDR, REG_WHO_AM_I, &id, 1);
    if (st != I2C_OK) {
        return (int)st;
    }
    if (id != MPU6050_WHO_AM_I_VAL) {
        return (int)I2C_ERR_NACK;   /* 有回應但不是 MPU6050：當成「不在」*/
    }

    /* Step 2: 喚醒。以前不看結果，寫失敗也回 OK —— 晶片停在 sleep，
     * 之後讀到的全是 0，看起來像「不動」而不是「壞了」。 */
    return (int)i2c1_write_reg(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00);
}

void mpu6050_read_accel(mpu6050_axes_t *acc)
{
    uint8_t buf[6];
    i2c1_read_regs(MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, 6);
    parse_axes(buf, acc);
}

void mpu6050_read_gyro(mpu6050_axes_t *gyro)
{
    uint8_t buf[6];
    i2c1_read_regs(MPU6050_ADDR, REG_GYRO_XOUT_H, buf, 6);
    parse_axes(buf, gyro);
}

int mpu6050_read_all(mpu6050_axes_t *acc, mpu6050_axes_t *gyro)
{
    uint8_t buf[14];

    /* MPU6050 register 佈局是連續的：
     *   0x3B~0x40  accel (6 bytes)
     *   0x41~0x42  temperature (2 bytes)
     *   0x43~0x48  gyro (6 bytes)
     * 一次 burst read 14 bytes 比兩次 6 bytes 快很多
     */
    /* 用非同步版：這支是 Sensor task 的 100Hz 熱路徑，400kHz 下這筆交易
     * 約 400µs。polling 版會讓 prio 4 的 Sensor 空轉整整 400µs，期間
     * Logger/Display/Console 全部被鎖在外面；async 版把那段時間讓出去。
     * 其餘 mpu6050_* 函式（WHO_AM_I、init）不在熱路徑，繼續用 polling。 */
    i2c_status_t st = i2c1_read_regs_async(MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, 14);
    if (st != I2C_OK) {
        return (int)st;           /* acc/gyro 保持不動，交給呼叫端處理 */
    }

    parse_axes(&buf[0], acc);     // buf[0~5]
    parse_axes(&buf[8], gyro);    // buf[8~13]（跳過 buf[6~7] 的溫度）
    return 0;
}
