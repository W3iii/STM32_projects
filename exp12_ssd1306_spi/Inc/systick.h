/*
 * systick.h
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

void systick_init(uint32_t ticks);
void delay_ms(uint32_t ms);

/* 回傳開機後經過的毫秒數（= SysTick 中斷累計次數）
 * 給其他 driver 做 timeout 用，例如：
 *   uint32_t start = systick_get_ms();
 *   while (...) { if (systick_get_ms() - start > 100) timeout; }
 */
uint32_t systick_get_ms(void);

#endif /* SYSTICK_H_ */
