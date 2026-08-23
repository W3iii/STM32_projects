/*
 * device_ina219.c
 *
 *  INA219 driver implementation
 *
 *  Created on: Aug 3, 2026
 *      Author: xushu
 */

#include "device_ina219.h"
#include "driver_i2c1.h"

/* ---- 暫存器位址（全部 16-bit 寬）---- */
#define REG_CONFIG      0x00
#define REG_SHUNT_V     0x01
#define REG_BUS_V       0x02
#define REG_POWER       0x03    /* 未使用：功率在軟體算 */
#define REG_CURRENT     0x04    /* 未使用：電流在軟體算 */
#define REG_CALIB       0x05    /* 未使用：不走晶片的 calibration 路徑 */

/*
 * Config = 0x399F，逐段拆解：
 *   bit15    RST  = 0      不重置
 *   bit13    BRNG = 1      匯流排電壓量程 32V
 *   bit12:11 PG   = 11     分流 PGA ÷8 → ±320mV（0.1Ω 時 = ±3.2A）
 *   bit10:7  BADC = 0011   bus  ADC 12-bit，轉換 532µs
 *   bit6:3   SADC = 0011   shunt ADC 12-bit，轉換 532µs
 *   bit2:0   MODE = 111    shunt + bus 連續量測
 * 這剛好也是晶片 reset 後的預設值，但明寫一次比較保險——
 * 前一版韌體可能把它留在 power-down 模式。
 */
#define CONFIG_VALUE    0x399F

/* ---- raw → 物理單位的刻度（資料手冊定義，不可改）---- */
#define SHUNT_LSB_MV    0.01f       /* shunt LSB = 10µV = 0.01mV */
#define BUS_LSB_V       0.004f      /* bus   LSB = 4mV           */

/* 讀一個 16-bit 暫存器（MSB first）。
 *
 * 用非同步版有兩個理由：
 *   1) Display task 每 100ms 讀兩次（shunt + bus），400kHz 下每筆交易約 250µs，
 *      polling 的話這 0.5ms 全算成 display 的 CPU 時間。
 *   2) len==2 會走 I2C 的 POS 路徑 —— 那是 F4 這顆周邊最容易出錯的地方，
 *      而專案裡原本只有 mpu6050 的 len=14 在跑 async，POS 那條只有 host
 *      測試驗過、真機上從沒執行。掛在 10Hz 的 Display 上等於持續驗證。
 *
 * 呼叫端（bootstrap 的 ina219_init、Display 的 ina219_read）都已持有
 * i2c1_lock，且都在排程器啟動後才跑，符合 async 版的前提。 */
static int ina219_read16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    if (i2c1_read_regs_async(INA219_ADDR, reg, buf, 2) != I2C_OK) {
        return -1;
    }
    *out = (uint16_t)((uint16_t)buf[0] << 8 | buf[1]);
    return 0;
}

int ina219_init(void)
{
    if (i2c1_write_reg16(INA219_ADDR, REG_CONFIG, CONFIG_VALUE) != I2C_OK) {
        return -1;                  /* NACK：位址不對或根本沒接 */
    }

    /* read-back 當在線偵測：INA219 沒有 WHO_AM_I 可讀。 */
    uint16_t cfg;
    if (ina219_read16(REG_CONFIG, &cfg) != 0) {
        return -1;
    }
    return (cfg == CONFIG_VALUE) ? 0 : -1;
}

int ina219_read(ina219_data_t *out)
{
    uint16_t raw_shunt, raw_bus;

    /* 兩次獨立交易：暫存器指標不會自動遞增（見 .h 的說明）。 */
    if (ina219_read16(REG_SHUNT_V, &raw_shunt) != 0) {
        return -1;
    }
    if (ina219_read16(REG_BUS_V, &raw_bus) != 0) {
        return -1;
    }

    /* shunt：16-bit 二補數，直接轉 int16_t 就拿到有號值（反接電流為負）。 */
    out->shunt_mv = (float)(int16_t)raw_shunt * SHUNT_LSB_MV;

    /* bus：資料在 bit15:3，低 3 bit 是旗標（bit1 CNVR 轉換完成、bit0 OVF
     * 數學溢位）。OVF 只在使用晶片 calibration 路徑時才有意義，這裡算在
     * 軟體所以直接忽略；連續量測模式下 CNVR 也不必等。 */
    out->bus_v = (float)(raw_bus >> 3) * BUS_LSB_V;

    /* 歐姆定律：mV / Ω = mA，單位剛好對上不用再換算。 */
    out->current_ma = out->shunt_mv / INA219_R_SHUNT_OHM;

    /* V × mA = mW。 */
    out->power_mw = out->bus_v * out->current_ma;

    return 0;
}
