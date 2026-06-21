/*
 * driver_rcc.h
 *
 *  系統時脈：PLL → 168MHz SYSCLK。來源以外部 8MHz 晶振(HSE)為主，HSE 在
 *  timeout 內沒起振就自動 fallback 回內建 16MHz RC(HSI)，永不卡死。
 *  ─────────────────────────────────────────────
 *  必須在啟動 FreeRTOS 排程器之前呼叫（main() 最前面），因為 port 會用
 *  configCPU_CLOCK_HZ（= 168e6）去算 SysTick reload；硬體實際頻率得先到位。
 *
 *  升頻後各匯流排頻率（其他 driver 的分頻常數要跟著對齊）：
 *    SYSCLK / HCLK(AHB) = 168 MHz
 *    PCLK1 (APB1)       =  42 MHz   → I2C1 / TIM2-5
 *    PCLK2 (APB2)       =  84 MHz   → USART1 / SPI1
 *
 *  Created on: Jun 21, 2026
 *      Author: xushu
 */

#ifndef DRIVER_RCC_H_
#define DRIVER_RCC_H_

/*
 * HSE 8MHz → 168MHz。內含：HSE 起振、PWR VOS scale1、flash 5WS + cache、
 * bus 分頻（APB1/4、APB2/2）、PLL(M=8,N=336,P=2,Q=7) 起振、切 SYSCLK 到 PLL。
 * 全程用 polling 等 RDY flag，不依賴 SysTick，可在排程器啟動前安全呼叫。
 */
void rcc_clock_init_168mhz(void);

/* 目前實際的匯流排頻率（Hz）。rcc_clock_init_168mhz() 內已從暫存器算好快取，
 * 之後讀取零成本。driver 算 baud/timing 時用這些，不要再寫死頻率常數：
 *   USART1/SPI1 在 APB2 → rcc_pclk2_hz()；I2C1 在 APB1 → rcc_pclk1_hz()。
 * 改了 PLL 設定也不必動 driver——值是從硬體算出來的。 */
uint32_t rcc_hclk_hz(void);    /* AHB  (HCLK)  */
uint32_t rcc_pclk1_hz(void);   /* APB1 (PCLK1) */
uint32_t rcc_pclk2_hz(void);   /* APB2 (PCLK2) */

#endif /* DRIVER_RCC_H_ */
