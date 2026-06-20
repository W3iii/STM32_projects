/*
 * app_button.c
 *
 *  User button（PA0, STM32F407-Discovery B1）time-based 去抖。
 *  ─────────────────────────────────────────────
 *  純粹的去抖 + 邊緣偵測，只回報「按下」事件，不碰 datalogger——
 *  按下要做什麼由 main 決定（driver 只報狀態，方便重用）。
 *
 *  co-operative 主迴圈每圈呼叫 app_button_poll()，不走 EXTI 中斷：
 *  輪詢去抖在這個架構下最簡單，也省掉「ISR 裡碰 FatFs」的同步問題。
 *
 *  去抖法：原始電位必須連續穩定 BTN_DEBOUNCE_MS 才採信；穩定值由
 *  release→press 翻轉時，才回報一次「確認的按下」。
 *
 *  Created on: Jun 19, 2026
 *      Author: xushu
 */

#include "app_button.h"
#include "systick.h"
#include "stm32f407xx.h"

#define BTN_PIN          0U      /* PA0 = B1 user button */
#define BTN_DEBOUNCE_MS  20U     /* 原始電位要穩定這麼久才採信 */

static uint8_t  s_stable;        /* 去抖後狀態：1=按下 */
static uint8_t  s_last_raw;      /* 上次取樣的原始電位 */
static uint32_t s_last_change;   /* 原始電位最後一次變化的時刻 */

/* B1 按下接 VDD（板上外接下拉）→ active-high，1 = 按下 */
static uint8_t button_raw(void)
{
    return (GPIOA->IDR & (1U << BTN_PIN)) ? 1U : 0U;
}

void app_button_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        /* usart1_init 多半已開，保險再開一次 */
    GPIOA->MODER &= ~(0x3U << (BTN_PIN * 2));   /* PA0 = input */
    GPIOA->PUPDR &= ~(0x3U << (BTN_PIN * 2));   /* 無內部 pull（板上已有外接下拉）*/

    s_last_raw    = button_raw();
    s_stable      = s_last_raw;
    s_last_change = systick_get_ms();
}

int app_button_poll(uint32_t now_ms)
{
    uint8_t raw = button_raw();

    if (raw != s_last_raw) {            /* 還在彈跳 → 重新計時，先不採信 */
        s_last_raw    = raw;
        s_last_change = now_ms;
        return 0;
    }

    /* 原始電位穩定夠久，且和目前去抖狀態不同 → 確認一次狀態轉換 */
    if ((now_ms - s_last_change) >= BTN_DEBOUNCE_MS && raw != s_stable) {
        s_stable = raw;
        if (s_stable == 1U) {           /* 只回報 release→press 的確認邊緣 */
            return 1;
        }
    }
    return 0;
}
