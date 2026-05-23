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

#endif /* SYSTICK_H_ */
