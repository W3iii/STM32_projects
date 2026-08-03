/*
 * device_ina219.h
 *
 *  INA219 電流／功率監測器（I2C1，預設位址 0x40）。
 *  ─────────────────────────────────────────────
 *  和 MPU6050 並聯在 PB8(SCL)/PB9(SDA) 上，用 i2cscan 確認位址後才接得起來。
 *
 *  量測原理：晶片內部量兩個東西——
 *    - shunt voltage：分流電阻兩端的壓降（LSB 10µV，有號）→ 除以電阻值就是電流
 *    - bus voltage  ：負載端對 GND 的電壓（LSB 4mV，暫存器 bit15:3）
 *  電流與功率由本驅動在軟體算，不使用晶片的 Calibration/Current/Power 暫存器：
 *  少寫一支 calibration register、少一組要跟著 shunt 值改的魔術數，
 *  而且 raw → 物理單位的換算全部攤在 C 裡看得見（和 mpu6050 / tempsensor 一致）。
 *
 *  Created on: Aug 3, 2026
 *      Author: xushu
 */

#ifndef DEVICE_INA219_H_
#define DEVICE_INA219_H_

#include <stdint.h>

/* 7-bit I2C 位址。A1/A0 兩腳都接 GND = 0x40（模組出廠預設）；
 * 兩腳可各接 GND/VS+/SDA/SCL，組出 0x40~0x4F 共 16 個位址。 */
#define INA219_ADDR         0x40

/* 分流電阻值（Ω）。GY-219 / Adafruit 模組上焊的是 R100 = 0.1Ω。
 * 換了電阻只要改這一個數字，電流與功率自動跟著對。 */
#define INA219_R_SHUNT_OHM  0.1f

/* 一次量測的結果（物理單位，換算已在驅動內完成）。 */
typedef struct {
    float bus_v;        /* 匯流排（負載）電壓，V   */
    float shunt_mv;     /* 分流電阻壓降，mV        */
    float current_ma;   /* 電流，mA（可為負：反向）*/
    float power_mw;     /* 功率 = bus_v × current  */
} ina219_data_t;

/*
 * 初始化：寫入 config（32V 量程 / ±320mV PGA / 12-bit / 連續量測），
 * 再讀回來比對當作「在線偵測」——INA219 沒有 WHO_AM_I 暫存器，
 * config read-back 是慣用的替代做法。
 *
 * 回傳：0 = OK，-1 = 沒回應或讀回值不符（沒接、位址不對、線鬆）
 */
int ina219_init(void);

/*
 * 讀一次電壓／電流／功率。
 *
 * 注意：INA219 的暫存器指標「不會」自動遞增，所以 shunt(0x01) 與 bus(0x02)
 *       必須分兩次交易讀，不能一次 burst 4 bytes（那只會拿到同一個暫存器兩遍）。
 *       兩筆之間若被別的 task 插進來上 bus，讀到的會是不同時刻的值 →
 *       呼叫端要把整支函式包在 i2c1_lock() 內（見 bus_port.h）。
 *
 * 回傳：0 = OK，-1 = I2C 失敗（out 不會被寫）
 */
int ina219_read(ina219_data_t *out);

#endif /* DEVICE_INA219_H_ */
