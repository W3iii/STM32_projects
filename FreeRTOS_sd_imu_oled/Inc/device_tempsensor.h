/*
 * device_tempsensor.h
 *
 *  STM32F407 晶片內建溫度感測器（die temperature），走 ADC1 內部通道 16。
 *  ─────────────────────────────────────────────
 *  不佔任何 GPIO：溫度感測器與 VREFINT 掛在 ADC 內部，靠 ADC_COMMON->CCR 的
 *  TSVREFE 位元開啟。轉換讀到的電壓與溫度成線性關係（datasheet 典型值）：
 *      V_sense = adc / 4095 × VREF
 *      T(°C)   = (V_sense - V25) / Avg_Slope + 25
 *  F407 沒有出廠校正值（不像 L4/F0），只能用 datasheet 典型值 V25=0.76V、
 *  Avg_Slope=2.5mV/°C，精度 ±1.5°C 起跳。
 *
 *  ⚠ 量的是「晶片自身溫度」，不是室溫：168MHz 全速 + 自身發熱，讀值通常
 *    30~45°C。當晶片溫度監控很好用，別當室溫計。
 *
 *  單通道、單次觸發、12-bit、無 DMA/中斷 —— 由 Sensor task 在 100Hz 迴圈裡
 *  阻塞讀一次（<50us，不影響節奏），刻意不獨立成 task。
 *
 *  Created on: Aug 2, 2026
 *      Author: xushu
 */

#ifndef DEVICE_TEMPSENSOR_H_
#define DEVICE_TEMPSENSOR_H_

#include <stdint.h>     /* tempsensor_read_raw() 回傳 uint16_t */

/* 初始化 ADC1 + 開啟內建溫度感測器（channel 16）。bootstrap 初始化階段呼叫一次。
 * 開啟後需 ~10us 穩定時間，bootstrap 內的 delay_ms(10) 已覆蓋。 */
void tempsensor_init(void);

/* 觸發一次轉換並回傳晶片溫度（°C，阻塞等 EOC，典型 <50us）。
 * EOC 逾時仍會回目前 DR 換算值，不卡死 Sensor task。 */
float tempsensor_read_c(void);

/* 觸發一次轉換並回傳溫度通道 12-bit raw（0..4095）。除錯用。 */
uint16_t tempsensor_read_raw(void);

/* 用 VREFINT 反推目前實際的 VDDA（V）。除錯用：看供電是否真的 3.3V。
 * 讀不到 VREFINT 時回 0。 */
float tempsensor_vdda(void);

#endif /* DEVICE_TEMPSENSOR_H_ */
