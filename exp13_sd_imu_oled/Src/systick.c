/*
 * systick.c
 *
 *  Created on: May 13, 2026
 *      Author: xushu
 */

#include "systick.h"
#include "stm32f407xx.h"

static volatile uint32_t tick = 0;

void SysTick_Handler(void)
{
    tick++;
}

void systick_init(uint32_t ticks)
{
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL  = 0;
    SYSTICK->CTRL = 0x7;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = tick;

    while ((tick - start) < ms) {
    }
}

uint32_t systick_get_ms(void)
{
    return tick;
}
