/*
 * app_button.c
 *
 *  User button（PA0, STM32F407-Discovery B1）EXTI 事件驅動版。
 *  ─────────────────────────────────────────────
 *  與輪詢版的差別：不再每 10ms 取樣 pin，而是讓 PA0 的 rising edge 走 EXTI0
 *  中斷，ISR 內只清 pending + 呼叫上層註冊的 callback（通常是 vTaskNotifyGiveFromISR
 *  喚醒 button task）。平時 0 次取樣、0% CPU。
 *
 *  為什麼去抖不放在這裡：
 *    接點彈跳會在按下/放開瞬間產生多個 EXTI 邊緣。正確做法是邊緣只當「該去看一下」
 *    的觸發，真正的去抖（等 ~20ms 沉澱後重讀電位確認）放在被喚醒的 task 脈絡——
 *    那裡才能安全地 vTaskDelay、也能呼叫會走 FatFs 的 start/stop。在 ISR 內做這些
 *    是大忌。放開時的彈跳同樣靠「沉澱後重讀，仍按著才採信」濾掉。
 *
 *  Created on: Jun 19, 2026
 *      Author: xushu
 */

#include "app_button.h"
#include "stm32f407xx.h"
#include "isr_port.h"          /* ISR 計時掛勾；沒有 isr_port_config.h 時展開成空 */

#define BTN_PIN   0U     /* PA0 = B1 user button，對應 EXTI line 0 / IRQ_EXTI0 */

/* EXTI0 邊緣 callback（ISR 脈絡）。由上層 app_button_set_callback() 注入。 */
static void (*s_btn_cb)(void) = 0;

void app_button_init(void)
{
    /* --- clock：GPIOA（讀 pin）、SYSCFG（EXTI 來源選擇）--- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* --- PA0 = input，無內部 pull（板上已有外接下拉，active-high）--- */
    GPIOA->MODER &= ~(0x3U << (BTN_PIN * 2));   /* input */
    GPIOA->PUPDR &= ~(0x3U << (BTN_PIN * 2));   /* no pull */

    /* --- 把 EXTI0 的來源接到 port A ---
     * SYSCFG_EXTICR1（EXTICR[0]）的 bit[3:0] 選 EXTI0 的來源 port，0b0000 = PA。 */
    SYSCFG->EXTICR[0] &= ~(0xFU << 0);

    /* --- EXTI line 0：rising edge（按下）觸發，unmask --- */
    EXTI->RTSR |=  (1U << BTN_PIN);     /* rising：放開→按下 */
    EXTI->FTSR &= ~(1U << BTN_PIN);     /* 不收 falling，避免放開也觸發 */
    EXTI->PR    =  (1U << BTN_PIN);     /* 寫 1 清掉可能的殘留 pending */
    EXTI->IMR  |=  (1U << BTN_PIN);     /* interrupt 不遮罩 */

    /* --- NVIC：EXTI0 會在 ISR 內呼叫 s_btn_cb（可能呼叫 ...FromISR()），
     * 優先權數值必須 >= MAX_SYSCALL(=5)。設 6，再 enable。 --- */
    NVIC_SetPriority(IRQ_EXTI0, 6);
    NVIC_EnableIRQ(IRQ_EXTI0);
}

void app_button_set_callback(void (*cb)(void))
{
    s_btn_cb = cb;
}

/* B1 按下接 VDD（板上外接下拉）→ active-high，1 = 按下 */
int app_button_is_down(void)
{
    return (GPIOA->IDR & (1U << BTN_PIN)) ? 1 : 0;
}

/*
 * EXTI line 0 中斷 handler（覆寫 startup 的 weak symbol）。
 *   只做兩件事：清 pending、呼叫 callback。去抖在被喚醒的 task 做。
 */
void EXTI0_IRQHandler(void)
{
    ISR_PORT_ENTER();

    if (EXTI->PR & (1U << BTN_PIN)) {
        EXTI->PR = (1U << BTN_PIN);     /* 寫 1 清除 pending（必須清，否則反覆觸發）*/
        if (s_btn_cb) {
            s_btn_cb();                 /* 通知 button task：有邊緣，去去抖確認 */
        }
    }

    ISR_PORT_EXIT();
}