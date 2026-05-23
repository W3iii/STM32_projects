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

/* ---- I2C 7-bit address ---- */
#define MPU6050_ADDR        0x68

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
    /* Step 1: 確認 WHO_AM_I 是不是真的有接到 MPU6050 */
    if (mpu6050_who_am_i() != MPU6050_WHO_AM_I_VAL) {
        return -1;
    }

    /* Step 2: 喚醒 (PWR_MGMT_1 = 0x00 → 清 SLEEP bit) */
    i2c1_write_reg(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00);

    /* 之後想加 DLPF / 變更量程，在這裡多寫幾個 reg 就好 */

    return 0;
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

void mpu6050_read_all(mpu6050_axes_t *acc, mpu6050_axes_t *gyro)
{
    uint8_t buf[14];

    /* MPU6050 register 佈局是連續的：
     *   0x3B~0x40  accel (6 bytes)
     *   0x41~0x42  temperature (2 bytes)
     *   0x43~0x48  gyro (6 bytes)
     * 一次 burst read 14 bytes 比兩次 6 bytes 快很多
     */
    i2c1_read_regs(MPU6050_ADDR, REG_ACCEL_XOUT_H, buf, 14);

    parse_axes(&buf[0], acc);     // buf[0~5]
    parse_axes(&buf[8], gyro);    // buf[8~13]（跳過 buf[6~7] 的溫度）
}
