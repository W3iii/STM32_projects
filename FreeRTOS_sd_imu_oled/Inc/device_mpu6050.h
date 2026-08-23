/*
 * mpu6050.h
 *
 *  Created on: May 18, 2026
 *      Author: xushu
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

/* ---- 資料結構：三軸 raw 值 ---- */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_axes_t;

/* ---- 預設量程靈敏度（重置後 ±2g / ±250°/s）----
 * raw int16 → 物理單位：g = raw / LSB_PER_G、°/s = raw / LSB_PER_DPS。
 * 若之後改寫 ACCEL_CONFIG / GYRO_CONFIG 量程，這兩個值要一起改。 */
#define MPU6050_ACC_LSB_PER_G      16384.0f   /* ±2g   量程 */
#define MPU6050_GYRO_LSB_PER_DPS   131.0f     /* ±250°/s 量程 */

/* ---- 初始化 ----
 *  回傳值：0 = OK, -1 = WHO_AM_I 不對（沒接好 / 模組壞了）
 */
int mpu6050_init(void);

/* ---- 讀取 ---- */
void mpu6050_read_accel(mpu6050_axes_t *acc);
void mpu6050_read_gyro (mpu6050_axes_t *gyro);

/* ---- 一次讀完加速度 + 角速度（最高效）---- */
/* 一次 14-byte burst 讀出 accel + gyro。
 * 回傳 0 = OK，非 0 = I2C 交易失敗（i2c_status_t 的值）。
 * 失敗時 acc/gyro 不會被寫入 —— 呼叫端必須檢查回傳值，不能把上一次的
 * 殘值當成新樣本記錄下去。 */
int mpu6050_read_all(mpu6050_axes_t *acc, mpu6050_axes_t *gyro);

/* ---- 探測：回傳 WHO_AM_I 的值（正常 = 0x68）---- */
uint8_t mpu6050_who_am_i(void);

#endif /* MPU6050_H_ */
