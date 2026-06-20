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

/* ---- 初始化 ----
 *  回傳值：0 = OK, -1 = WHO_AM_I 不對（沒接好 / 模組壞了）
 */
int mpu6050_init(void);

/* ---- 讀取 ---- */
void mpu6050_read_accel(mpu6050_axes_t *acc);
void mpu6050_read_gyro (mpu6050_axes_t *gyro);

/* ---- 一次讀完加速度 + 角速度（最高效）---- */
void mpu6050_read_all(mpu6050_axes_t *acc, mpu6050_axes_t *gyro);

/* ---- 探測：回傳 WHO_AM_I 的值（正常 = 0x68）---- */
uint8_t mpu6050_who_am_i(void);

#endif /* MPU6050_H_ */
