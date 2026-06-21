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

#endif /* DRIVER_RCC_H_ */
