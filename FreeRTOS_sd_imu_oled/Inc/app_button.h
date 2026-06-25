/*
 * app_button.h
 *
 *  User button（STM32F407-Discovery B1 = PA0, active-high）。
 *  ─────────────────────────────────────────────
 *  事件驅動版：PA0 接 EXTI0，按下（rising edge）觸發中斷 → 呼叫註冊的 callback。
 *  driver 不依賴 RTOS、也不決定按下要做什麼；callback 與去抖策略由上層決定。
 *
 *  去抖不在 driver 做：EXTI 只負責「邊緣到了」這個事件，實際去抖（等接點沉澱
 *  後重讀電位確認）放在喚醒後的 task 脈絡，避免在 ISR 內做耗時或碰 FatFs。
 *
 *  Created on: Jun 19, 2026
 *      Author: xushu
 */

#ifndef APP_BUTTON_H_
#define APP_BUTTON_H_

/* 初始化 user button：
 *   - 開 GPIOA / SYSCFG clock，PA0 設為 input（板上已有外接下拉）
 *   - SYSCFG 把 EXTI0 接到 port A，設 rising-edge trigger
 *   - NVIC：EXTI0 優先權設為 6（>= MAX_SYSCALL，ISR 內才能呼叫 ...FromISR()），再 enable
 *   注意：要在註冊 callback、建立 button task 之前呼叫；callback 未註冊前
 *   EXTI 觸發只會清 pending，不會通知任何人（安全）。 */
void app_button_init(void);

/* 註冊 EXTI0 邊緣 callback（ISR 脈絡）。RTOS 版可在 callback 內用
 * vTaskNotifyGiveFromISR() 喚醒 button task。傳 NULL 取消。 */
void app_button_set_callback(void (*cb)(void));

/* 讀目前按鈕原始電位：1 = 按下（PA0 high），0 = 放開。
 * 去抖用：被喚醒後等沉澱時間，再用這個重讀確認是不是真的按著。 */
int app_button_is_down(void);

#endif /* APP_BUTTON_H_ */