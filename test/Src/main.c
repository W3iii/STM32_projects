#include <stdint.h>
#include "stm32f407xx.h"

int main(void)
{
    SYSTICK->LOAD = 16000 - 1;
    SYSTICK->VAL  = 0;
    SYSTICK->CTRL = 0x7;

    volatile uint32_t ctrl_readback = SYSTICK->CTRL;
    volatile uint32_t load_readback = SYSTICK->LOAD;
    volatile uint32_t cfgr_val      = RCC->CFGR;

    while (1) {}
}
